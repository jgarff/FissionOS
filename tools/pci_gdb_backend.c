#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <netdb.h>
#include <sys/mman.h>
#include <pci/pci.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>

#include "swd.h"
#include "swd_bitbang.h"
#include "swd_target.h"
#include "swd_gdb_backend.h"

#include "pci.h"


#define GDB_BIND_ADDR                            "0.0.0.0"
#define GDB_TCP_PORT                             3333
#define GDB_TCP_BUFLEN                           1500


typedef struct {
    volatile sysregs_t *regs;
    int listen_fd;
    int connected_fd;
    int mem_fd;
} driver_t;

typedef struct swd_gdb_client {
    driver_t *driver;
} swd_gdb_client_t;


// The following are implemented for the SWD bitbanging interface
void swd_bitbang_clk_dir(struct swd *instance, int output) {
    // Nothing to do, clock is always output
}

void swd_bitbang_clk_set(struct swd *instance, int high) {
    driver_t *driver = (driver_t *)instance->driver_private;
    volatile sysregs_t *sysregs = driver->regs;

    if (high) {
        sysregs->debug |= SYSREGS_DEBUG_SWDCLKTCK;
    } else {
        sysregs->debug &= ~SYSREGS_DEBUG_SWDCLKTCK;
    }
}

void swd_bitbang_dio_dir(struct swd *instance, int output) {
    // Nothing to do, the input/output is driven by the SWD core in the ARM IP
}

void swd_bitbang_dio_set(struct swd *instance, int high) {
    driver_t *driver = (driver_t *)instance->driver_private;
    volatile sysregs_t *sysregs = driver->regs;

    if (high) {
        sysregs->debug |= SYSREGS_DEBUG_SWDITMS;
    } else {
        sysregs->debug &= ~SYSREGS_DEBUG_SWDITMS;
    }
}

int swd_bitbang_dio_get(struct swd *instance) {
    driver_t *driver = (driver_t *)instance->driver_private;
    volatile sysregs_t *sysregs = driver->regs;
    int output = 0;

    output = sysregs->debug & SYSREGS_DEBUG_SWDO ? 1 : 0;

    return output;
}

void swd_bitbang_udelay(uint32_t usecs) {
    usleep(usecs);
}

void swd_gdb_write(swd_gdb_client_t *client, char *data, uint32_t len) {
    driver_t *driver = client->driver;
    int fd = driver->connected_fd;
    int result;

    result = write(fd, data, len);

    // All errors will cause a select failure, so it'll hit next time around the loop
    (void)result;
}

/*
 * Platform specific initialization
 */
int swd_driver_init(swd_t *instance, driver_t *driver)
{
    instance->driver_private = driver;

    // Start and stop sequence functions, from the common bitbang driver
    instance->start = swd_bitbang_start;
    instance->stop = swd_bitbang_stop;
    instance->flush = swd_bitbang_flush;

    // Request functions from the common bitbang driver
    instance->request_recv = swd_bitbang_request_recv;
    instance->request_xmit = swd_bitbang_request_xmit;

    return 0;
}



uint64_t pci_device_base(void) {
    struct pci_access *pacc;
    struct pci_dev *dev;
    uint64_t base_addr = 0;

    pacc = pci_alloc();

    pci_init(pacc);
    pci_scan_bus(pacc);

    for (dev = pacc->devices; dev; dev = dev->next) {
        pci_fill_info(dev, PCI_FILL_IDENT | PCI_FILL_BASES | PCI_FILL_CLASS);

        if ((dev->vendor_id == PCI_VEND_ID) && (dev->device_id == PCI_DEV_ID)) {
            base_addr = (uint64_t)dev->base_addr[0];
        }
    }

    pci_cleanup(pacc);

    return base_addr;
}

volatile uint8_t *device_map(uint64_t base_addr, int length, int *fd) {
    size_t pagesize = sysconf(_SC_PAGE_SIZE);
    off_t page_base = (base_addr / pagesize) * pagesize;
    volatile uint8_t *memory;
    
    *fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (*fd < 0) {
        perror("open() failed");
            return NULL;
    }
    
    memory = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, *fd, page_base);
    if (memory == MAP_FAILED) {
        close(*fd);
        perror("mmap() failed");
        return NULL;
    }

    return memory;
}

int socket_setup(void) {
    struct sockaddr_in saddr;
    struct hostent *hent;
    int fd, result;

    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(GDB_TCP_PORT);
    
    hent = gethostbyname(GDB_BIND_ADDR);
    if (!hent) {
        perror("gethostbyname()");
        return -1;
    }

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("socket()");
        return -1;
    }

    result = bind(fd, (struct sockaddr *)&saddr, sizeof(saddr));
    if (result < 0) {
        perror("bind()");
        close(fd);
        return -1;
    }

    result = listen(fd, 1);
    if (result < 0) {
        perror("listen()");
        close(fd);
        return -1;
    }

    return fd;
}

void mainloop(driver_t *driver) {
    volatile sysregs_t *sysregs = driver->regs;
    int done = 0, maxfd;
    fd_set readfds;
    swd_t swd;

    // Take the CPU out of reset
    sysregs->debug |= SYSREGS_DEBUG_CPU_RESET;

    swd_driver_init(&swd, driver);

    maxfd = driver->listen_fd;

    while (!done) {
        int n;

        FD_ZERO(&readfds);
        FD_SET(driver->listen_fd, &readfds);
        if (driver->connected_fd != -1) {
            FD_SET(driver->connected_fd, &readfds);
        }

        n = select(maxfd + 1, &readfds, NULL, NULL, NULL);
        if (n == -1) {
            perror("select()");   // TODO:  Handle broken pipes
            return;
        }

        if (n) {
            // New connection?
            if (FD_ISSET(driver->listen_fd, &readfds)) {
                struct sockaddr_in addr;
                unsigned addrlen = sizeof(addr);
                int newfd;
                
                newfd = accept(driver->listen_fd, (struct sockaddr *)&addr, &addrlen);
                if (newfd < 0) {
                    perror("accept()");
                }

                if (driver->connected_fd == -1) {
                    // New client connection
                    driver->connected_fd = newfd;
                    if (newfd > maxfd) {
                        maxfd = newfd;
                    }

                    // Halt target on new connections
                    swd_target_halt(&swd);
                } else {
                    // Reject the new connection, already have a client
                    close(newfd);
                }
            }

            // Client connection?
            if ((driver->connected_fd >= 0) && (FD_ISSET(driver->connected_fd, &readfds))) {
                uint8_t readbuf[GDB_TCP_BUFLEN];
                swd_gdb_client_t client = {
                    .driver = driver,
                };
                int n;

                n = read(driver->connected_fd, readbuf, sizeof(readbuf));
                if (n <= 0) {
                    close(driver->connected_fd);
                    driver->connected_fd = -1;
                }

                gdb_stream_process(&client, &swd, readbuf, n);
            }

        }

        // We got a signal to shutdown?  We don't have timeouts.
        if (n == 0) {
            done = 1;
        }
    }
}

int main(int argc, char *argv[]) {
    volatile uint8_t *bar;
    driver_t driver;
    uint64_t base_addr;
    int memfd, tcpfd;

    base_addr = pci_device_base();
    if (!base_addr) {
        fprintf(stderr, "Device not found!\n");
        return -1;
    }

    bar = device_map(base_addr, REGION_MAP_LEN, &memfd);
    if (!bar) {
        return -1;
    }

    tcpfd = socket_setup();
    if (tcpfd < 0) {
        close(memfd);
        return -1;
    }

    driver.regs = (sysregs_t *)(bar + REGION_SYSREGS_OFFSET);
    driver.mem_fd = memfd;
    driver.connected_fd = -1;
    driver.listen_fd = tcpfd;

    mainloop(&driver);

    munmap((uint8_t *)bar, REGION_MAP_LEN);
    close(memfd);
    close(tcpfd);

    return 0;
}
