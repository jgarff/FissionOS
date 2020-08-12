#ifndef __PCI_H__
#define __PCI_H__


#define REGION_MAP_LEN                           (1024 * 1024)

#define REGION_SHARED_OFFSET                     0x00000000
#define REGION_SHARED_LEN                        (64 * 1024)

#define REGION_SYSREGS_OFFSET                    0x00020000
#define REGION_SYSREGS_LEN                       (4 * 1024)

#define PCI_VEND_ID                              0xb05e
#define PCI_DEV_ID                               0x0001

#define PCI_MEM_SIZE                             (64 * 1024)

typedef struct {
    uint32_t debug;
#define SYSREGS_DEBUG_SWDCLKTCK                  (1 << 0)   // Write the clock line
#define SYSREGS_DEBUG_SWDITMS                    (1 << 1)   // Write the data value
#define SYSREGS_DEBUG_SWDO                       (1 << 2)   // Read the value of the SWDO pin
#define SYSREGS_DEBUG_SWDOEN                     (1 << 3)   // Read the output state of the SWDO bit
#define SYSREGS_DEBUG_CPU_RESET                  (1 << 31)  // 1 to take the CPU out of reset
    uint32_t eth;
#define SYSREGS_ETH_PHY_RESETN                   (1 << 31)  // 0 to hold the phy in reset
} __attribute__((packed)) sysregs_t;


#endif /* __PCI_H__ */
