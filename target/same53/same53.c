/*
 * same53.c
 *
 * Copyright (c) 2017-2018 Jeremy Garff
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
 * Author: Jeremy Garff <jer@jers.net>
 *
 */


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "saml_clocks.h"
#include "saml_port.h"
#include "saml_sercom.h"
#include "saml_nvm.h"
#include "saml_reset.h"
#include "saml_usb.h"
#include "saml_tcc.h"
#include "saml_adc.h"
#include "saml_power.h"
#include "reg.h"
#include "saml_gmac.h"

#include <vectors.h>
#include <systick.h>
#include <workqueue.h>
#include <console.h>
#include <semaphore.h>
#include <mailbox.h>
#include <usb.h>
#include <adc_calc.h>

#include "lwip/ip.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/stats.h"
#include "ethernetif.h"

#include "http.h"

#include "usb_config.h"
#include "usb_serial.h"
#include "usb_vendor.h"

#include "same53_version.h"
#include "same53.h"

#define ADC_ADJUST_MAX                           SYSTICK_FREQ

#define PHY_ADDR                                 0x00
uint8_t mac_addr[6] = { 0x00, 0x27, 0xeb, 0x05, 0x45, 0xd5 };

gmac_txbuf_desc_t gmac_txdesc[2];
uint8_t gmac_txpool[GMAC_TXBUF_SIZE * ARRAY_SIZE(gmac_txdesc)];
gmac_rxbuf_desc_t gmac_rxdesc[2];
uint8_t gmac_rxpool[GMAC_RXBUF_SIZE * ARRAY_SIZE(gmac_rxdesc)];
gmac_drv_t gmac =
{
    .txdesc = gmac_txdesc,
    .txbuf_count = ARRAY_SIZE(gmac_txdesc),
    .rxdesc = gmac_rxdesc,
    .rxbuf_count = ARRAY_SIZE(gmac_rxdesc),
};

/* Network interface global variables */
static struct ip4_addr ipaddr, netmask;
static struct ip4_addr gw;


#define ADC_STATE_SAMPLE                         0
#define ADC_STATE_ADJUST                         1

uint32_t adc_state = ADC_STATE_SAMPLE;
uint32_t adc_adjust_count = 0;
uint64_t adc_samples = 0;

#define PWM_MODE_LINEAR                          0
#define PWM_MODE_GAMMA                           1
#define PWM_MODE_DISABLED                        2

char *pwm_modestr[] = {
    [PWM_MODE_LINEAR]   = "Linear",
    [PWM_MODE_GAMMA]    = "Gamma",
    [PWM_MODE_DISABLED] = "Disabled",
};

console_t console;

typedef struct 
{
    int average_seconds;
    int pwm_mode;
    float pwm_gamma_exponent;
} settings_t;

settings_t current_settings =
{
    .average_seconds = ADC_ADJUST_MAX,
    .pwm_mode = PWM_MODE_GAMMA,
    .pwm_gamma_exponent = GAMMA_EXPONENT,
};


int cmd_ipstats(console_t *console, int argc, char *argv[]);
int cmd_threads(console_t *console, int argc, char *argv[]);
int cmd_status(console_t *console, int argc, char *argv[]);
//
// Console Commands
//
uart_drv_t dbg_uart;
cmd_entry_t cmd_table[] =
{
    CONSOLE_CMD_ETH,
    CONSOLE_CMD_HELP,
    {
        .cmdstr = "ip",
        .callback = cmd_ipstats,
    },
    CONSOLE_CMD_NVM,
    //CONSOLE_CMD_RESET,
    {
        .cmdstr = "status",
        .callback = cmd_status,
        .usage = "  status < show | set <key> <value> >\r\n",
        .help =
            "  Show values of the analog channels.\r\n"
            "    show       : Show values in mV.\r\n"
            "    set        : Set <key> to <value>.\r\n"
            "\r\n"
            "  Key/Values:\r\n"
            "    pwmmode    : linear, gamma, disable\r\n"
            "    pwmgamma   : <exponent value>\r\n"
            "                 Default exponent value is 2.2\r\n"
            "    avgsecs    : <seconds>\r\n"
            "                 Number of seconds for light change transisiton\r\n",
    },
    {
        .cmdstr = "threads",
        .callback = cmd_threads,
    },
    CONSOLE_CMD_USB,
};

int cmd_ipstats(console_t *console, int argc, char *argv[])
{
    stats_display();
    console_print(console, "\r\n");
    return 0;
}

void http_net_xml_callback(struct netconn *client, struct netbuf *rxbuf, char **querystr, void *arg)
{
    int i;

    http_content_type_send(client, "application/xml");

    http_printf(client, "<netconfig>");
    http_printf(client, "<dhcp>true</dhcp>");
    http_printf(client, "<hostname>same53</hostname>");
    http_printf(client, "<macaddr>");
    for (i = 0; i < sizeof(mac_addr); i++) {
        http_printf(client, "%02x", mac_addr[i]);
    }
    http_printf(client, "</macaddr>");
    http_printf(client, "<ipaddr></ipaddr>");
    http_printf(client, "<netmask></netmask>");
    http_printf(client, "<gateway></gateway>");
    http_printf(client, "</netconfig>");

    netbuf_delete(rxbuf);
}

const http_cgi_table_t http_cgi[] =
{
    {
        .name = "net_xml.cgi",
        .callback = http_net_xml_callback,
        .arg = NULL,
    },
};
const uint32_t http_cgi_table_count = ARRAY_SIZE(http_cgi);


adc_desc_t adc0;
const adc_calc_voltage_divider_t adc_calc_10_1_divider =
{
    .mvref = ADVREF_mV,
    .r1 = 10000,
    .r2 = 1000,
};
uint32_t adc0_ain0_value;
adc_drv_t adc0_ain0_vsense =
{
    .value = &adc0_ain0_value,
    .channel = 12,
    .calc = adc_calc_divider_12bit_unsigned,
    .calc_arg = (void *)&adc_calc_10_1_divider,
};

uint32_t adc0_ain1_value;
adc_drv_t adc0_ain1_vsense =
{
    .value = &adc0_ain1_value,
    .channel = 13,
    .calc = adc_calc_divider_12bit_unsigned,
    .calc_arg = (void *)&adc_calc_10_1_divider,
};
uint32_t adc0_ain2_value;
adc_drv_t adc0_ain2_vsense =
{
    .value = &adc0_ain2_value,
    .channel = 14,
    .calc = adc_calc_divider_12bit_unsigned,
    .calc_arg = (void *)&adc_calc_10_1_divider,
};
uint32_t adc0_ain3_value;
adc_drv_t adc0_ain3_vsense =
{
    .value = &adc0_ain3_value,
    .channel = 15,
    .calc = adc_calc_divider_12bit_unsigned,
    .calc_arg = (void *)&adc_calc_10_1_divider,
};

adc_drv_t *adc0_sensors[] =
{
    &adc0_ain0_vsense,
    &adc0_ain1_vsense,
    &adc0_ain2_vsense,
    &adc0_ain3_vsense,
};

uint32_t *adc_values[] =
{
    &adc0_ain0_value,
    &adc0_ain1_value,
    &adc0_ain2_value,
    &adc0_ain3_value,
};

uint32_t pwmvals[ARRAY_SIZE(adc_values)];
uint32_t pwmprevs[ARRAY_SIZE(adc_values)];
uint32_t adc_compute_pwmval(uint32_t adc_value)
{
    uint32_t pwmval;
    float value;
    float gamma;

    switch (current_settings.pwm_mode)
    {
        case PWM_MODE_LINEAR:
            pwmval = (adc_value * TCC0_MAX) / 10000;  // Convert mV to percent of TCC0_MAX
            break;

        case PWM_MODE_GAMMA:
            value = adc_value / (float)ADCMAX_mV;  // Convert mV to percent
            gamma = powf(value, current_settings.pwm_gamma_exponent);
            pwmval = gamma * TCC0_MAX;
            break;

        default:
            pwmval = 0;
            break;
    }

    return pwmval;
}

void adc_complete(adc_drv_t **adc, uint32_t count, void *arg)
{
    workqueue_t *wq = (workqueue_t *)arg;
    int i;

    for (i = 0; i < ARRAY_SIZE(adc_values); i++)
    {
        pwmprevs[i] = pwmvals[i];
        pwmvals[i] = adc_compute_pwmval(*adc_values[i]);
    }

    adc_state = ADC_STATE_ADJUST;
    adc_samples++;

    workqueue_add(wq, 0);
}

adc_queue_entry_t adc_queue = 
{
    .adcs = adc0_sensors,
    .count = ARRAY_SIZE(adc0_sensors),
    .complete = adc_complete,
};

void adc_worker(void *arg);
workqueue_t adc_wq =
{
    .callback = adc_worker,
    .arg = NULL,
};
void adc_worker(void *arg)
{
    int irqstate = irq_save();
    int i;

    switch (adc_state)
    {
        case ADC_STATE_ADJUST:
            for (i = 0; i < ARRAY_SIZE(adc_values); i++)
            {
                float value, stepsize;

                if (pwmvals[i] > pwmprevs[i])
                {
                    stepsize = ((pwmvals[i] - pwmprevs[i]) /
                               (float)current_settings.average_seconds) *
                               adc_adjust_count;
                    value = pwmprevs[i] + stepsize;
                }
                else
                {
                    stepsize = ((pwmprevs[i] - pwmvals[i]) /
                               (float)current_settings.average_seconds) *
                               adc_adjust_count;
                    value = pwmprevs[i] - stepsize;
                }

                if (adc_adjust_count == (current_settings.average_seconds - 1))
                {
                    value = pwmvals[i];
                }

                tcc_pwm_duty(TCC0, i, value);
            }

            adc_adjust_count++;
            if (adc_adjust_count < current_settings.average_seconds)
            {
                workqueue_add(&adc_wq, 1);
                break;
            }

            adc_adjust_count = 0;
            adc_state = ADC_STATE_SAMPLE;

            workqueue_add(&adc_wq, 1);
            break;

        case ADC_STATE_SAMPLE:
            adc_queue.complete_arg = &adc_wq;
            adc_start(&adc0, &adc_queue);
            break;

        default:
            break;
    }

    irq_restore(irqstate);
}

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

int cmd_status(console_t *console, int argc, char *argv[])
{
    int irqstate;

    if ((argc == 2) && !strcmp(argv[1], "show"))
    {
        int i;

        console_print(console, "\r\n");
        console_print(console, "PWM\r\n");
        console_print(console, "  Mode      : %s\r\n", pwm_modestr[current_settings.pwm_mode]);
        console_print(console, "  Exponent  : %d.%d\r\n", 
                (int)current_settings.pwm_gamma_exponent,
                (int)(current_settings.pwm_gamma_exponent * 1000) % 1000);
        console_print(console, "\r\n");
        console_print(console, "ADC Samples : %ld\r\n", adc_samples);
        console_print(console, "ADC Average : %d\r\n", current_settings.average_seconds /
                      SYSTICK_FREQ);
        console_print(console, "ADC Status  :\r\n");
        for (i = 0; i < ARRAY_SIZE(adc_values); i++)
        {
            int value = *adc_values[i];
            console_print(console, "  %2d (%05d): %d.%03d, %d\r\n", 
                          i, value, value / 1000, value % 1000, pwmvals[i]);
        }
        console_print(console, "\r\n");
    }
    else if ((argc == 4) && !strcmp(argv[1], "set"))
    {
        if (!strcmp(argv[2], "pwmmode"))
        {
            if (!strcmp(argv[3], "linear"))
            {
                current_settings.pwm_mode = PWM_MODE_LINEAR;
            }
            else if (!strcmp(argv[3], "gamma"))
            {
                current_settings.pwm_mode = PWM_MODE_GAMMA;
            }
            else if (!strcmp(argv[3], "disable"))
            {
                current_settings.pwm_mode = PWM_MODE_DISABLED;
            }
            else
            {
                cmd_help_usage(console, argv[0]);
            }
        }
        else if (!strcmp(argv[2], "pwmgamma"))
        {
            current_settings.pwm_gamma_exponent = strtof(argv[3], NULL);
        }
        else if (!strcmp(argv[2], "avgsecs"))
        {
            current_settings.average_seconds = strtod(argv[3], NULL) * SYSTICK_FREQ;

            irqstate = irq_save();
            workqueue_remove(&adc_wq);
            adc_state = ADC_STATE_ADJUST;
            adc_adjust_count = 0;
            workqueue_add(&adc_wq, 1);
            irq_restore(irqstate);
        }
        else
        {
            cmd_help_usage(console, argv[0]);
        }
    }
    else
    {
        cmd_help_usage(console, argv[0]);
    }

    return 0;
}

void vbus_callback(void *arg)
{
    if (port_get(VBUS_PORT, VBUS_PIN))
    {
        usb_attach();
    }
    else
    {
        usb_detach();
    }
}

ext_int_t vbus =
{
    .callback = vbus_callback,
    .arg = NULL,
};

mailbox_t net_start;
void gmac_link_change_worker(void *arg);
static workqueue_t link_wq =
{
    .callback = gmac_link_change_worker,
};
void gmac_link_change_worker(void *arg)
{
    if (gmac_link_change_handle())
    {
        workqueue_add(&link_wq, SYSTICK_FREQ);
        return;
    }

    mailbox_send(&net_start, NULL);
}

void clock_init(void)
{
    volatile oscctrl_t *osc = OSCCTRL;
    volatile xosc32k_t *osc32 = XOSC32K;
    uint32_t tmp32;
    uint16_t tmp16;
    uint8_t tmp8;

    /*
     *
     * Setup the DFLL for full speed using closed loop mode referenced
     * from a external 32Khz crystal.  Then use that to generate the 120Mhz CPU
     * clock.
     *
     * OSC32K --> GCLK1 --> DFLL (48Mhz) --> GCLK3 (2Mhz) --> DPLL0 (120Mhz)
     *                           |                                  |
     *                           --> GCLK2 (48Mhz)                  --> GCLK0 (120Mhz) --> CPU
     *
     * The following are the clock sources available to peripherals after configuration:
     *
     * GCLK0 - 120Mhz
     * GCLK1 - 32Khz
     * GCLK2 - 48Mhz
     * GCLK3 - 2Mhz
     *
     */

    //
    // 32Khz XOSC
    //
    // Setup XOSC32 from external crystal
    tmp16 = XOSC32K_XOSC32K_XTALEN |
            XOSC32K_XOSC32K_EN32K |
            XOSC32K_XOSC32K_STARTUP(2) |
            XOSC32K_XOSC32K_CGM_HS;
    write16(&osc32->xosc32k, tmp16);

    tmp16 |= XOSC32K_XOSC32K_ENABLE;
    write16(&osc32->xosc32k, tmp16);
    while (!(osc32->status & XOSC32K_STATUS_XOSC32KRDY))
        ;
    
    // Loop back the 32K reference to the DFLL
    gclk_setup(LOWSPEED_GCLK, GCLK_GENCTRL_SRC_XOSC32K, 0);
    gclk_peripheral_enable(LOWSPEED_GCLK, GCLK_DFLL48M_REF);

    // Setup the CPU to run off the slow clock temporarily while we setup
    // the DFLL in closed loop mode
    gclk_setup(CLK_CORE, GCLK_GENCTRL_SRC_XOSC32K, 0);

    //
    // 48 Mhz DFLL Config
    //
    // Turn off first
    tmp8 = 0;
    write8(&osc->dfllctrla, tmp8);

    // See datasheet for multiplier values
    tmp32 = OSCCTRL_DFLLMUL_MUL(1464) | OSCCTRL_DFLLVAL_COARSE(1) |
          OSCCTRL_DFLLVAL_DIFF(1);
    write32(&osc->dfllmul, tmp32);

    tmp8 = OSCCTRL_DFLLCTRLB_MODE;  // Closed loop mode
    write8(&osc->dfllctrlb, tmp8);

    // Turn it on
    write8(&osc->dfllctrla, OSCCTRL_DFLLCTRLA_ENABLE);
    while (!(osc->status & OSCCTRL_STATUS_DFLLRDY))
        ;

    // Setup the 48Mhz clock source
    gclk_setup(CLK48MHZ_GCLK, GCLK_GENCTRL_SRC_DFLL, 0);
    
    //
    // 2Mhz GCLK
    //
    // Run the 120Mhz clock from the 48Mhz reference
    gclk_setup(CLK2MHZ_GCLK, GCLK_GENCTRL_SRC_DFLL, 24);  // 2 Mhz

    //
    // 120 Mhz DPLL0 Config
    //
    // Set the DPLL0 as a output from the 2Mhz clock for reference
    gclk_peripheral_enable(CLK2MHZ_GCLK, GCLK_FDPLL0);

    // Turn the PLL off
    write8(&osc->dpll0ctrla, 0);
    while ((osc->dpll0syncbusy & OSCCTRL_DPLL0SYNCBUSY_ENABLE))
        ;

    // Setup the PLL to use the 2Mhz GCLK reference from above, and multiply
    // by 60 to get to 120Mhz
    write32(&osc->dpll0ctrlb, OSCCTRL_DPLL0CTRLB_REFCLK_GCLK);
    write32(&osc->dpll0ratio, OSCCTRL_DPLL0RATIO_LDR(60 - 1));
    while ((osc->dpll0syncbusy & OSCCTRL_DPLL0SYNCBUSY_DPLLRATIO))
        ;

    // Fire it up and wait for register sync
    write8(&osc->dpll0ctrla, OSCCTRL_DPLL0CTRLA_ENABLE);
    while ((osc->dpll0syncbusy & OSCCTRL_DPLL0SYNCBUSY_ENABLE))
        ;

    // Wait for the clock to lock and become ready
    while (!(osc->dpll0status & OSCCTRL_DPLL0STATUS_CLKRDY))
        ;

    // Set the CPU to use the DPLL at 120Mhz
    gclk_setup(CLK_CORE, GCLK_GENCTRL_SRC_DPLL0, 0);
}

//
// Main initialization
//
int main(int argc, char *argv[])
{
    volatile mclk_t *mclk = MCLK;
    volatile int delay = 100000;
    struct netconn *http_netconn;
    void *msg;

    clock_init();
    systick_init(GCLK0_HZ);

    //
    // Setup the UART
    //
    mclk->apbamask |= MCLK_APBAMASK_SERCOM0;
    gclk_peripheral_enable(CLK48MHZ_GCLK, DBG_UART_GCLK_CORE);
    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_TX_PIN, DBG_UART_PORT_TX_MUX);
    port_peripheral_enable(DBG_UART_PORT, DBG_UART_PORT_RX_PIN, DBG_UART_PORT_RX_MUX);
    sercom_usart_async_init(DBG_UART_ID, &dbg_uart, DBG_UART_PERIPHERAL_ID,
                            UART_SRC_HZ, DBG_UART_BAUDRATE,
                            SERCOM_USART_CTRLB_CHSIZE_8BITS,
                            SERCOM_USART_CTRLB_SBMODE_1BIT,
                            SERCOM_USART_CTRLA_FORM_FRAME, SERCOM_USART_CTRLB_PMODE_EVEN,
                            DBG_UART_TX_PAD, DBG_UART_RX_PAD);
    // UART debugging
#ifdef UART_CONSOLE
    uart_console(&console, &dbg_uart);
#endif
    
    //
    // Setup external interrupts
    //
    mclk->apbamask |= MCLK_APBAMASK_EIC;
    gclk_peripheral_enable(CLK48MHZ_GCLK, GCLK_EIC);
    port_peripheral_enable(VBUS_PORT, VBUS_PIN, VBUS_MUX);
    port_dir(VBUS_PORT, VBUS_PIN, 0);  // Input

    eic_int_setup(VBUS_INTNUM, &vbus, EIC_EDGE_BOTH);
    eic_int_enable(VBUS_INTNUM);

    eic_enable();

    //
    // Setup Ethernet Port and Clocks
    //
    mclk->ahbmask |= MCLK_AHBMASK_GMAC;
    mclk->apbcmask |= MCLK_APBCMASK_GMAC;

    port_peripheral_enable(GMAC_GTXCK_PORT, GMAC_GTXCK_PIN, GMAC_MUX);
    port_peripheral_enable(GMAC_CRS_PORT, GMAC_CRS_PIN, GMAC_MUX);
    port_peripheral_enable(GMAC_GTXEN_PORT, GMAC_GTXEN_PIN, GMAC_MUX);
    port_peripheral_enable(GMAC_GTX0_PORT, GMAC_GTX0_PIN, GMAC_MUX);
    port_peripheral_enable(GMAC_GTX1_PORT, GMAC_GTX1_PIN, GMAC_MUX);
    port_peripheral_enable(GMAC_GRX0_PORT, GMAC_GRX0_PIN, GMAC_MUX);
    port_peripheral_enable(GMAC_GRX1_PORT, GMAC_GRX1_PIN, GMAC_MUX);
    port_peripheral_enable(GMAC_GRXER_PORT, GMAC_GRXER_PIN, GMAC_MUX);
    port_peripheral_enable(GMAC_MDC_PORT, GMAC_MDC_PIN, GMAC_MUX);
    port_peripheral_enable(GMAC_MDIO_PORT, GMAC_MDIO_PIN, GMAC_MUX);

    port_strength(GMAC_GTXCK_PORT, GMAC_GTXCK_PIN, PORT_DRIVE_LOW);
    port_strength(GMAC_GTXEN_PORT, GMAC_GTXEN_PIN, PORT_DRIVE_LOW);
    port_strength(GMAC_GTX0_PORT, GMAC_GTX0_PIN, PORT_DRIVE_LOW);
    port_strength(GMAC_GTX1_PORT, GMAC_GTX1_PIN, PORT_DRIVE_LOW);
    port_strength(GMAC_MDC_PORT, GMAC_MDC_PIN, PORT_DRIVE_LOW);
    port_strength(GMAC_MDIO_PORT, GMAC_MDIO_PIN, PORT_DRIVE_LOW);

    //
    // Release Phy Reset
    //
    port_peripheral_disable(PRST_PORT, PRST_PIN);
    port_strength(GMAC_MDIO_PORT, GMAC_MDIO_PIN, PORT_DRIVE_HIGH);
    port_dir(PRST_PORT, PRST_PIN, 1);
    port_set(PRST_PORT, PRST_PIN, 0);
    while (delay)
    {
        delay--;
    }
    port_set(PRST_PORT, PRST_PIN, 1);

    gmac_init(&gmac, mac_addr, PHY_ADDR, gmac_txpool, gmac_rxpool);

    //
    // Setup ADC
    //
    mclk->apbdmask |= MCLK_APBDMASK_ADC0;
    // Enable the internal 1v0 bandgap reference
    SUPC->vref = SUPC_VREF_VREFOE | SUPC_VREF_ONDEMAND;
    // Setup clocks and pins
    gclk_peripheral_enable(CLK48MHZ_GCLK, GCLK_ADC0);
    port_peripheral_enable(ADC0_AIN0_PORT, ADC0_AIN0_PIN, ADC_MUX);
    port_peripheral_enable(ADC0_AIN1_PORT, ADC0_AIN1_PIN, ADC_MUX);
    port_peripheral_enable(ADC0_AIN2_PORT, ADC0_AIN2_PIN, ADC_MUX);
    port_peripheral_enable(ADC0_AIN3_PORT, ADC0_AIN3_PIN, ADC_MUX);

    adc_init(0, &adc0);

    //
    // Setup PWM
    //
    mclk->apbbmask |= MCLK_APBBMASK_TCC0;
    gclk_peripheral_enable(CLK48MHZ_GCLK, GCLK_TCC0_1);
    port_peripheral_enable(PWM0_PORT, PWM0_PIN, PWM_MUX);
    port_peripheral_enable(PWM1_PORT, PWM1_PIN, PWM_MUX);
    port_peripheral_enable(PWM2_PORT, PWM2_PIN, PWM_MUX);
    port_peripheral_enable(PWM3_PORT, PWM3_PIN, PWM_MUX);

    // Invert channels 0, 1, 2, and 3 with 0xf
    tcc_pwm_init(TCC0, TCC_CTRLA_PRESCALER_DIV1, 0xf, TCC0_MAX - 2);

    //
    // Setup the USB port
    //
    mclk->apbbmask |= MCLK_APBBMASK_USB;
    gclk_peripheral_enable(CLK48MHZ_GCLK, GCLK_USB);
    port_peripheral_enable(USB_DP_PORT, USB_DP_PIN, USB_DP_MUX);
    port_peripheral_enable(USB_DN_PORT, USB_DN_PIN, USB_DN_MUX);
    usb_init();

    //
    // Setup the control and serial endpoints
    //
#ifndef UART_CONSOLE
    usb_serial_init(usb_console_rx_callback, &console);
#endif
    usb_control0_init((char *)&usb_desc, usb_desc_len,
                      (char *)&usb_config, usb_config_len,
                      usb_str_desc);
    usb_vendor_init();

    //
    // Setup the console
    //
    // USB Console
#ifndef UART_CONSOLE
    console_init(&console, cmd_table, ARRAY_SIZE(cmd_table),
                 (console_send_t)usb_console_send_wait, 
                 (console_recv_t)usb_console_recv,
                 NULL);
#else
    // Serial Console Debugging
    console_init(&console, cmd_table, ARRAY_SIZE(cmd_table),
                 (console_send_t)uart_send_wait, 
                 (console_recv_t)uart_recv,
                 &dbg_uart);
#endif /* UART_CONSOLE */

    thread_init(&console);

    //
    // Status LED
    //
    // Turn on the status LED
    mclk->apbbmask |= MCLK_APBBMASK_TCC1;
    port_strength(LED_PORT, LED_PIN, PORT_DRIVE_HIGH);
    port_peripheral_enable(LED_PORT, LED_PIN, LED_MUX);

    tcc_pwm_init(LED_TCC, TCC_CTRLA_PRESCALER_DIV1,
                 (1 << LED_TCC_CHANNEL), LED_TCC_MAX - 2);
    status_worker(NULL);

    // Setup the USB state
    vbus_callback(NULL);

    // Start the ADC sampler and PWM generator based on levels
    adc_worker(NULL);

    // Start the ethernet link worker
    gmac_link_change_worker(NULL);
    mailbox_recv(&net_start, &msg, 0);

    tcpip_init(NULL, NULL);
    ip4_addr_set_zero(&ipaddr);
    ip4_addr_set_zero(&netmask);
    ip4_addr_set_zero(&gw);
    netif_set_hostname(&gmac.netif, "same53");
    netif_add(&gmac.netif, &ipaddr, &netmask, &gw, NULL, ethernetif_init, tcpip_input);
    netif_set_default(&gmac.netif);
    netif_set_up(&gmac.netif);

    dhcp_start(&gmac.netif);

    // Mainloop
    http_netconn = http_service_start();
    while (1)
    {
        http_mainloop(http_netconn);
    }

    return 0;
}

