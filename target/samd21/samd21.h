/*
 * samd.h
 *
 *
 * Copyright (c) 2017 Western Digital Corporation or its affiliates.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the copyright holder nor the names of its contributors may not
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jeremy Garff <jeremy.garff@sandisk.com>
 *
 */


#ifndef __SAMD_H__
#define __SAMD_H__


#define GCLK0_HZ                                 48000000
#define GCLK0                                    0
#define GCLK1                                    1

#define SPI_DEV                                  SERCOM3_SPI
#define SPI_CLOCK                                GCLK_SERCOM3_CORE
#define SPI_CLOCK_BAUD                           1000000
#define SPI_PERIPHERAL                           PERIPHERAL_ID_SERCOM3
#define SPI_DEVNUM                               3
#define SPI_DIPO                                 3
#define SPI_DOPO                                 0
#define SPI_FORM                                 0
#define SPI_MOSI_PORT                            PORTA
#define SPI_MOSI_PIN                             16
#define SPI_MOSI_MUX                             3
#define SPI_SCK_PORT                             PORTA
#define SPI_SCK_PIN                              17
#define SPI_SCK_MUX                              3
#define SPI_MISO_PORT                            PORTA
#define SPI_MISO_PIN                             19
#define SPI_MISO_MUX                             3
#define SPI_SS_PORT                              PORTA
#define SPI_SS_PIN                               18

#define RFRST_N_PORT                             PORTA
#define RFRST_N_PIN                              15

#define LED1_PORT                                PORTA
#define LED1_PIN                                 7

#define USB_DP_PORT                              PORTA
#define USB_DP_PIN                               24
#define USB_DP_MUX                               6

#define USB_DN_PORT                              PORTA
#define USB_DN_PIN                               25
#define USB_DN_MUX                               6

#define VBUS_PORT                                PORTA
#define VBUS_PIN                                 27
#define VBUS_MUX                                 0
#define VBUS_INTNUM                              15

#define LED_PORT                                 PORTA
#define LED_PIN                                  6
#define LED_MUX                                  4
#define LED_TCC                                  TCC1
#define LED_TCC_CHANNEL                          0
#define STATUS_MAX                               16
#define LED_TCC_MAX                              (1 << STATUS_MAX)

#define BOOTLOADER_SIZE                          (16 * 1024)
#define FLASH_SIZE_BYTES                         (256 * 1024)

#define BOOTCFG_ADDR                             (FLASH_SIZE_BYTES - (8 * 1024))
#define BOOTCFG_MAGIC                            0x90187340


#define SRAM_ADDR                                0x20000000


typedef struct imghdr
{
    uint32_t stack_ptr;
    uint32_t start_addr;
} __attribute__ ((packed)) imghdr_t;

#define IMGHDR                                   ((volatile imghdr_t *)BOOTLOADER_SIZE)


typedef struct bootcfg
{
    uint32_t magic;
    uint32_t len;
    uint32_t crc;
    uint32_t resvd_0x0c[13]; // PAD to page size
} bootcfg_t;


#endif /* __SAMD_H__ */
