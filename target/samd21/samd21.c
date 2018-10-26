/*
 * samd20.c
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


#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "saml_clocks.h"
#include "saml_port.h"
#include "saml_sercom.h"
#include "saml_tc.h"
#include "saml_nvm.h"
#include "saml_reset.h"
#include "saml_usb.h"
#include "saml_tcc.h"

#include <vectors.h>
#include <systick.h>
#include <workqueue.h>
#include <console.h>
#include <usb.h>

#include "usb_config.h"
#include "usb_serial.h"
#include "usb_vendor.h"

#include "rfm69hcw.h"

#include "samd21_version.h"
#include "samd21.h"


int cmd_config(console_t *console, int argc, char *argv[]);


volatile uint32_t *reset_config = RESET_CONFIG;

#define RFFREECOUNT                              8
rfbuf_t rfbufs[RFFREECOUNT];
rf69_spi_pkt_t rfpktbufs[RFFREECOUNT];

console_t console;

cmd_entry_t cmd_table[] =
{
    {
        .cmdstr = "config",
        .callback = cmd_config,
        .usage = "  config < show >\r\n",
        .help =
            "  Device Configuration.\r\n"
            "    show     : Show the device configuration.\r\n"
    },
    CONSOLE_CMD_HELP,
    CONSOLE_CMD_RF,
    CONSOLE_CMD_RESET,
    CONSOLE_CMD_USB,
};

int cmd_config(console_t *console, int argc, char *argv[])
{
    if (argc < 2)
    {
        cmd_help_usage(console, argv[0]);
        return 0;
    }

    if (!strcmp(argv[1], "show"))
    {
        console_print(console, "Magic   : %08x\r\n", CONFIG->magic);
        console_print(console, "Version : %d.%d.%d.%d\r\n",
                CONFIG->version.major,
                CONFIG->version.minor,
                CONFIG->version.micro,
                CONFIG->version.nano);
        console_print(console, "Flags   : %08x\r\n", CONFIG->flags);
        console_print(console, "Serial  : %08x%08x\r\n",
                ((uint32_t *)&CONFIG->serial)[1],
                ((uint32_t *)&CONFIG->serial)[0]);
        console_print(console, "CRC     : %08x\r\n", CONFIG->crc);
    } else {
        cmd_help_usage(console, argv[0]);
        return 0;
    }

    return 0;
}

//
// TODO:  Remove the following place holder functions when suitable functions
// are available.
//
void thread_switch_handler(void)
{
}

void clock_init(void)
{
    volatile uint32_t tmp;

    // CAUTION: Assuming 3.3 volt operation.
    // Flash must be configured for 1 wait state @3v3 before going full speed.
    // At 1v8, 3 wait states must be used.
    NVMCTRL->ctrlb = NVMCTRL_CTRLB_MANW | NVMCTRL_CTRLB_RWS(3);

    // Turn on the DLL in open mode and wait for ready
    tmp = SYSCTRL_DFLLCTRL_ENABLE;
    SYSCTRL->dfllctrl = tmp;
    while (!(SYSCTRL->pclksr & SYSCTRL_PCLKSR_DFLLRDY))
        ;

    // Setup the factory tuned coarse correction value
    tmp = SYSCTRL_DFLLVAL_COARSE(DFLLCTRL_DFLL_COARSE_VAL) |
          SYSCTRL_DFLLVAL_FINE(DFLLCTRL_DFLL_FINE_VAL);
    SYSCTRL->dfllval = tmp;
    while (!(SYSCTRL->pclksr & SYSCTRL_PCLKSR_DFLLRDY))
        ;

    // Switch to the DFLL clock for the main frequency
    gclk_setup(GCLK0, GCLK_GENCTRL_SRC_DFLL48M, 0);
}

void clock_usb(void)
{
    volatile uint32_t tmp;

    // Turn on the DLL and wait for ready
    tmp = SYSCTRL_DFLLCTRL_ENABLE |
          SYSCTRL_DFLLCTRL_MODE |
          SYSCTRL_DFLLCTRL_USBCRM;
    SYSCTRL->dfllctrl = tmp;
    while (!(SYSCTRL->pclksr & SYSCTRL_PCLKSR_DFLLRDY))
        ;

    tmp = SYSCTRL_DFLLMUL_MUL(48000000 / 1000) |
          SYSCTRL_DFLLMUL_FSTEP(0xff / 4) |
          SYSCTRL_DFLLMUL_CSTEP(0x1f / 4);
    SYSCTRL->dfllmul = tmp;
    while (!(SYSCTRL->pclksr & (SYSCTRL_PCLKSR_DFLLLCKC | SYSCTRL_PCLKSR_DFLLLCKF)))
        ;
}

void clock_open(void)
{
    volatile uint32_t tmp;

    // Turn on the DLL and wait for ready
    tmp = SYSCTRL_DFLLCTRL_ENABLE;
    SYSCTRL->dfllctrl = tmp;
    while (!(SYSCTRL->pclksr & SYSCTRL_PCLKSR_DFLLRDY))
        ;

    // Setup the factory tuned coarse correction value
    tmp = SYSCTRL_DFLLVAL_COARSE(DFLLCTRL_DFLL_COARSE_VAL) |
          SYSCTRL_DFLLVAL_FINE(DFLLCTRL_DFLL_FINE_VAL);
    SYSCTRL->dfllval = tmp;
    while (!(SYSCTRL->pclksr & SYSCTRL_PCLKSR_DFLLRDY))
        ;
}


void vbus_callback(void *arg)
{
    if (port_get(VBUS_PORT, VBUS_PIN))
    {
        usb_attach();
        clock_usb();
    }
    else
    {
        usb_detach();
        clock_open();
    }
}

ext_int_t vbus =
{
    .callback = vbus_callback,
    .arg = NULL,
};


void status_worker(void *arg);
workqueue_t status_wq =
{
    .callback = status_worker,
    .arg = NULL,
};

int status_direction = 1;
int status_value = 0;
void status_worker(void *arg)
{
    if (!status_direction)
    {
        status_value--;
        if (!status_value)
        {
            status_direction = 1;
        }
    }
    else
    {
        status_value++;
        if (status_value >= (STATUS_MAX - 1))
        {
            status_direction = 0;
        }
    }

    tcc_pwm_duty(LED_TCC, LED_TCC_CHANNEL, (status_value * LED_TCC_MAX) / STATUS_MAX); 

    workqueue_add(&status_wq, (SYSTICK_FREQ * (1000 / STATUS_MAX)) / 1000);
}

spi_drv_t spi_drv;

rf_port_t rf_ports[4];
rf_kv_t rf_kvs[1];

void rf_kv_led_send(void)
{
    rf_kv_pkt_t kvpkt = {
        .key = 1,
        .flags = RF69_KV_FLAGS_SET,
        .len = 0,
    };

    rf69_tx(&spi_drv, RF69_BROADCAST_ADDR, 1, 
            0, 0, 
            &kvpkt, sizeof(kvpkt),
            NULL, NULL);
}

void rf_kv_led_handler(volatile rfbuf_t *buf,
                       rf69_pkt_header_t *hdr,
                       rf_kv_pkt_t *kv,
                       void *arg)
{
    rfbuf_free(buf);
}

uint8_t rssiconfig[] = { RFM69_REG_RSSICONFIG_START };
uint8_t rxbw[] = { RFM69_REG_RXBW_DCC_FREQ(0x2) | RFM69_REG_RXBW_MANT(0x2) |
                   RFM69_REG_RXBW_EXP(0x5)};
uint8_t afcbw[] = { RFM69_REG_AFCBW_DCC_FREQ(0x4) | RFM69_REG_AFCBW_MANT(0x1) |
                    RFM69_REG_AFCBW_EXP(0x3)};
uint8_t diomapping2[] = { RFM69_REG_DIOMAPPING2_CLKOUT_OFF };
uint8_t rssithresh[] = { 0xe4 };
uint8_t syncvalue[] = RF69_NETWORK_ID;
uint8_t fifothresh[] = { RFM69_REG_FIFOTHRESH_TXSTARTCONDITION |
                         RFM69_REG_FIFOTHRESH_FIFOTHRESHOLD(0xf) };
uint8_t bcastaddr[] = { 0xff };
uint8_t testdagc[] = { 0x30 };
uint8_t lna[] = { RFM69_REG_LNA_ZIN };
uint8_t packetconfig1[] = { RFM69_REG_PACKETCONFIG1_PACKETFORMAT | 
                          RFM69_REG_PACKETCONFIG1_DCFREE_WHITEN |
                          RFM69_REG_PACKETCONFIG1_CRCON |
                          RFM69_REG_PACKETCONFIG1_ADDR_NODEBCAST };

rf69_reg_init_t rf69_regs[] = {
    {
        .addr = RFM69_REG_RXBW,
        .data = rxbw,
        .len = sizeof(rxbw),
    },
    {
        .addr = RFM69_REG_AFCBW,
        .data = afcbw,
        .len = sizeof(afcbw),
    },
    {
        .addr = RFM69_REG_DIOMAPPING2,
        .data = diomapping2,
        .len = sizeof(diomapping2),
    },
    {
        .addr = RFM69_REG_RSSITHRESH,
        .data = rssithresh,
        .len = sizeof(rssithresh),
    },
    {
        .addr = RFM69_REG_SYNCVALUE,
        .data = syncvalue,
        .len = sizeof(syncvalue),
    },
    {
        .addr = RFM69_REG_FIFOTHRESH,
        .data = fifothresh,
        .len = sizeof(fifothresh),
    },
    {
        .addr = RFM69_REG_TESTDAGC,
        .data = testdagc,
        .len = sizeof(testdagc),
    },
    {
        .addr = RFM69_REG_LNA,
        .data = lna,
        .len = sizeof(lna),
    },
    {
        .addr = RFM69_REG_PACKETCONFIG1,
        .data = packetconfig1,
        .len = sizeof(packetconfig1),
    },
    {
        .addr = RFM69_REG_BROADCASTADRS,
        .data = bcastaddr,
        .len  = sizeof(bcastaddr),
    },
};

void rf_worker(void *arg);
workqueue_t rf_wq = {
    .callback = rf_worker,
    .arg = NULL,
};

#define RF_WORKER_RESET                          0
#define RF_WORKER_RESET_DONE                     1
#define RF_WORKER_INIT                           2
#define RF_WORKER_TX                             3
int rf_worker_state = RF_WORKER_RESET;
void rf_worker(void *arg)
{
    switch (rf_worker_state)
    {
        case RF_WORKER_RESET:
            port_set(RFRST_N_PORT, RFRST_N_PIN, 1);
            rf_worker_state++;
            break;

        case RF_WORKER_RESET_DONE:
            port_set(RFRST_N_PORT, RFRST_N_PIN, 0);
            rf_worker_state++;
            break;

        case RF_WORKER_INIT:
            rf69_regs_init(&spi_drv, rf69_regs, ARRAY_SIZE(rf69_regs));
            rf_worker_state++;
            break;

        case RF_WORKER_TX:
            rf_kv_led_send();
            workqueue_add(&rf_wq, SYSTICK_FREQ / 5);
            return;

        default:
            return;
    }

    workqueue_add(&rf_wq, SYSTICK_FREQ);
}

void dio0_cb(void *arg)
{
    rf69_dio0_int(&spi_drv);
}

ext_int_t dio0 =
{
    .callback = dio0_cb,
    .arg = NULL,

};

//
// Main initialization
//
int main(int argc, char *argv[])
{
    clock_init();

    systick_init(GCLK0_HZ);

    PM->ahbmask |= PM_AHBMASK_USB;
    PM->apbbmask |= PM_APBBMASK_USB;
    gclk_peripheral_enable(GCLK0, GCLK_USB);
    port_peripheral_enable(USB_DP_PORT, USB_DP_PIN, USB_DP_MUX);
    port_peripheral_enable(USB_DN_PORT, USB_DN_PIN, USB_DN_MUX);
    usb_init();

    // TODO:  Validate the configuration CRC
    usb_serial_fixup(CONFIG->serial);
    usb_serial_init(usb_console_rx_callback, &console);
    usb_control0_init((char *)&usb_desc, usb_desc_len,
                      (char *)&usb_config, usb_config_len,
                      usb_str_desc);
    usb_vendor_init();

    console_init(&console, cmd_table, ARRAY_SIZE(cmd_table),
                 (console_send_t)usb_console_send_wait, 
                 (console_recv_t)usb_console_recv,
                 NULL);

    // Red LED
    PM->apbcmask |= PM_APBCMASK_TCC1;
    gclk_peripheral_enable(GCLK0, GCLK_TCC0_TCC1);
    port_peripheral_enable(LED_PORT, LED_PIN, LED_MUX);
    tcc_pwm_init(LED_TCC, TCC_CTRLA_PRESCALER_DIV1,
                 0, LED_TCC_MAX - 2);
    status_worker(NULL);

    // Green LED
    port_peripheral_disable(LED1_PORT, LED1_PIN);
    port_dir(LED1_PORT, LED1_PIN, 1);
    port_set(LED1_PORT, LED1_PIN, 1);

    // RF Reset -- Turn on internal pull-up
    port_peripheral_disable(RFRST_N_PORT, RFRST_N_PIN);
    port_dir(RFRST_N_PORT, RFRST_N_PIN, 1);
    port_set(RFRST_N_PORT, RFRST_N_PIN, 0);

    //
    // Setup RF external interrupt
    //
    PM->apbamask |= PM_APBAMASK_EIC;
    gclk_peripheral_enable(GCLK0, GCLK_EIC);
    port_peripheral_enable(DIO0_PORT, DIO0_PIN, DIO0_MUX);
    port_dir(DIO0_PORT, DIO0_PIN, 0);
    eic_int_setup(DIO0_INTNUM, &dio0, EIC_LEVEL_HIGH);

    //
    // Setup SPI
    //
    PM->apbcmask |= PM_APBCMASK_SERCOM3;
    gclk_peripheral_enable(GCLK0, GCLK_SERCOMx_SLOW);
    gclk_peripheral_enable(GCLK0, SPI_CLOCK);
    port_peripheral_enable(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_MUX);
    port_peripheral_enable(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_MUX);
    port_peripheral_enable(SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_MUX);
    port_peripheral_disable(SPI_SS_PORT, SPI_SS_PIN);
    port_dir(SPI_SS_PORT, SPI_SS_PIN, 1);
    spi_master_init(SPI_DEVNUM, &spi_drv, SPI_PERIPHERAL,
                           GCLK0_HZ, SPI_CLOCK_BAUD, 
                           SPI_SS_PORT, SPI_SS_PIN,
                           SPI_DIPO, SPI_DOPO,
                           SPI_FORM);

    rf_kv_init(rf_kvs, ARRAY_SIZE(rf_kvs));
    rf_kv_register(1, rf_kv_led_handler, NULL);

    rf_port_init(rf_ports, ARRAY_SIZE(rf_ports));
    rf_port_register(0, rf_kv_recv, NULL);

    rf69_init(&spi_drv, rfbufs, rfpktbufs, ARRAY_SIZE(rfbufs), DIO0_INTNUM);

    //
    // Setup external interrupts
    //
    gclk_peripheral_enable(GCLK0, GCLK_EIC);
    port_peripheral_enable(VBUS_PORT, VBUS_PIN, VBUS_MUX);
    port_dir(VBUS_PORT, VBUS_PIN, 0);  // Input

    eic_int_setup(VBUS_INTNUM, &vbus, EIC_EDGE_BOTH);
    eic_int_enable(VBUS_INTNUM);

    eic_enable();

    //
    // Check for USB voltage
    //
    vbus_callback(NULL);

    //
    // Start radio processing
    //
    rf_worker(NULL);

    // Mainloop
    while (1)
    {
        if (!workqueue_handle_next())
        {
            cpu_sleep();
        }
    }

    return 0;
}

