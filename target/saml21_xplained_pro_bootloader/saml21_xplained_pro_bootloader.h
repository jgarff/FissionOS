/*
 * saml21_xplained_pro.h
 *
 *
 * Copyright (c) 2013-2017 Western Digital Corporation or its affiliates.
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


#ifndef __SAML21_XPLAINED_PRO_H__
#define __SAML21_XPLAINED_PRO_H__


#define GCLK0_HZ                                 4000000

#define LED_PORT                                 PORTB
#define LED_PIN                                  10
#define LED_TC                                   TC1
#define LED_TC_MUX                               4
#define LED_GCLK                                 0
#define LED_GCLK_PERIPHERAL                      GCLK_TC1

#define DBG_UART_GCLK                            0
#define DBG_UART_BAUDRATE                        38400

#ifdef DBG_UART_PA45
  #define DBG_UART_ID                            0
  #define DBG_UART_GCLK_PERIPHERAL               GCLK_SERCOM0_CORE
  #define DBG_UART_PERIPHERAL_ID                 PERIPHERAL_ID_SERCOM0
  #define DBG_UART_PORT                          PORTA
  #define DBG_UART_PORT_TX_PIN                   4
  #define DBG_UART_PORT_TX_MUX                   3
  #define DBG_UART_TX_PAD                        SERCOM_USART_CTRLA_TXPO_PAD0
  #define DBG_UART_PORT_RX_PIN                   5
  #define DBG_UART_PORT_RX_MUX                   3
  #define DBG_UART_RX_PAD                        SERCOM_USART_CTRLA_RXPO_PAD1
#else /* DBG_UART_PA45 */
  #define DBG_UART_ID                            3
  #define DBG_UART_GCLK_PERIPHERAL               GCLK_SERCOM3_CORE
  #define DBG_UART_PERIPHERAL_ID                 PERIPHERAL_ID_SERCOM3
  #define DBG_UART_PORT                          PORTA
  #define DBG_UART_PORT_TX_PIN                   22
  #define DBG_UART_PORT_TX_MUX                   2
  #define DBG_UART_TX_PAD                        SERCOM_USART_CTRLA_TXPO_PAD0
  #define DBG_UART_PORT_RX_PIN                   23
  #define DBG_UART_PORT_RX_MUX                   2
  #define DBG_UART_RX_PAD                        SERCOM_USART_CTRLA_RXPO_PAD1
#endif /* DBG_UART_PA45 */

#define BOOTLOADER_SIZE                          (16 * 1024)
#define FLASH_SIZE_BYTES                         (256 * 1024)

#define BOOTCFG_ADDR                             (FLASH_SIZE_BYTES - (8 * 1024))
#define BOOTCFG_MAGIC                            0x90187340


#define LPSRAM_ADDR                              0x30000000

#define SRAM_SIZE                                (32 * 1024)
#define RESET_CONFIG_ADDR                        ((volatile uint8_t *)SRAM_BASE_ADDRESS + SRAM_SIZE - \
                                                                       sizeof(uint32_t))
#define RESET_CONFIG                             ((volatile uint32_t *)RESET_CONFIG_ADDR)


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


#endif /* __SAML21_XPLAINED_PRO_H__ */
