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


char *short_opts = "o:i:r:p:";

void usage(char *argv0)
{
    printf("\n");
    printf("%s: <-o filename> | <-i filename> | <-r <on | off> > | <-p <on | off> >", argv0);
    printf("\n");
    printf("Commands :\n");
    printf("    -i                 Load Memory from Filename\n");
    printf("    -o                 Read Memory to Filename\n");
    printf("    -r  <on | off>     CPU Reset (on), Release CPU Reset (off)\n");
    printf("    -p  <on | off>     Reset Phy (on), Release Phy Reset (off)\n");
    printf("\n");
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

#define MAX_READ_SIZE                            4096
int file_to_mem(char *filename, volatile uint8_t *bar) {
    int fd = open(filename, O_RDONLY);
    uint8_t buf[MAX_READ_SIZE];
    int result, offset = 0;

    if (fd < 0) {
        perror("open()");
        return -1;
    }

    result = read(fd, buf, MAX_READ_SIZE);
    while ((result > 0) && (offset < PCI_MEM_SIZE)) {
        if ((offset + result) > PCI_MEM_SIZE) {
            result = PCI_MEM_SIZE - offset;
        }
        memcpy((uint8_t *)&bar[offset], buf, result);
        offset += result;
        result = read(fd, buf, MAX_READ_SIZE);
    }

    close(fd);

    return 0;
}

int mem_to_file(char *filename, volatile uint8_t *bar) {
    return 0;
}

void reset_hold(volatile uint8_t *bar) {
    volatile sysregs_t *sysregs = (volatile sysregs_t *)(bar + REGION_SYSREGS_OFFSET);

    sysregs->debug &= ~SYSREGS_DEBUG_CPU_RESET;
}

void reset_release(volatile uint8_t *bar) {
    volatile sysregs_t *sysregs = (volatile sysregs_t *)(bar + REGION_SYSREGS_OFFSET);

    sysregs->debug |= SYSREGS_DEBUG_CPU_RESET;
}

void phy_reset_hold(volatile uint8_t *bar) {
    volatile sysregs_t *sysregs = (volatile sysregs_t *)(bar + REGION_SYSREGS_OFFSET);

    sysregs->eth &= ~SYSREGS_ETH_PHY_RESETN;
}

void phy_reset_release(volatile uint8_t *bar) {
    volatile sysregs_t *sysregs = (volatile sysregs_t *)(bar + REGION_SYSREGS_OFFSET);

    sysregs->eth |= SYSREGS_ETH_PHY_RESETN;
}

int main(int argc, char *argv[]) {
    volatile uint8_t *bar;
    uint64_t base_addr;
    char *filename = NULL;
    int opt, memfd;
    int input = 0, output = 0, reset = 0, phy_reset = 0;
    int cpu_reset = 0;
    int phy_reset_val = 0;

    while ((opt = getopt(argc, argv, short_opts)) != -1) {
        switch (opt) {
            case 'i':
                input = 1;
                filename = optarg;
                break;

            case 'o':
                output = 1;
                filename = optarg;
                break;

            case 'r':
                reset = 1;
                if (!strcmp("on", optarg)) {
                    cpu_reset = 1;
                }
                break;

            case 'p':
                phy_reset = 1;
                if (!strcmp("on", optarg)) {
                    phy_reset_val = 1;
                }
                break;

            default:
                usage(argv[0]);
                return -1;
        }
    }

    if (!input && !output && !reset && !phy_reset) {
        usage(argv[0]);
        return -1;
    }

    base_addr = pci_device_base();
    if (!base_addr) {
        fprintf(stderr, "Device not found!\n");
        return -1;
    }

    bar = device_map(base_addr, REGION_MAP_LEN, &memfd);
    if (!bar) {
        return -1;
    }

    if (input) {
        file_to_mem(filename, bar);
    }

    if (output) {
    }

    if (reset) {
        if (cpu_reset) {
            reset_hold(bar);
        } else {
            reset_release(bar);
        }
    }

    if (phy_reset) {
        if (phy_reset_val) {
            phy_reset_hold(bar);
        } else {
            phy_reset_release(bar);
        }
    }

    munmap((uint8_t *)bar, REGION_MAP_LEN);
    close(memfd);

    return 0;
}

