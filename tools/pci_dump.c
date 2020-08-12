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


#define REGION_SYSREGS_OFFSET                    0x20000
#define REGION_SYSREGS_LEN                       (256 * 1024)

#define GDB_BIND_ADDR                            "0.0.0.0"
#define GDB_TCP_PORT                             3333
#define GDB_TCP_BUFLEN                           1500

#define PCI_VEND_ID                              0xb05e
#define PCI_DEV_ID                               0x0001


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

int main(int argc, char *argv[]) {
    volatile uint8_t *bar;
    uint64_t base_addr;
    int memfd;
    int i;

    base_addr = pci_device_base();
    if (!base_addr) {
        fprintf(stderr, "Device not found!\n");
        return -1;
    }

    bar = device_map(base_addr, REGION_SYSREGS_LEN, &memfd);
    if (!bar) {
        return -1;
    }

    //*(uint32_t *)bar = 0x12345678; 

    i = 0;
    while (i < 256) {
        uint32_t *addr = (uint32_t *)bar;

        if (i % 16 == 0) {
            printf("%08x: ", i);
        }

        printf("%08x ", addr[i]);

        if ((i + sizeof(uint32_t)) % 16 == 0) {
            printf("\n");
        }

        i += sizeof(uint32_t);
    }

    munmap((uint8_t *)bar, REGION_SYSREGS_LEN);
    close(memfd);

    return 0;
}
