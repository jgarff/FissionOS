/*
 * saml_sercom_spi.c
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


#include <stdint.h>

#include <vectors.h>

#include "saml_port.h"

#include "saml_sercom.h"


static spi_drv_t *spi_drv[SERCOM_COUNT];

void spibusy_wait(sercom_spi_t *dev)
{
    while (dev->syncbusy)
        ;
}

static void sercom_spi_int_handler(spi_drv_t *drv)
{
    drv->rxbuf[drv->len++] = drv->dev->data;
    if (drv->len >= drv->xferlen)
    {
        port_set(drv->ssport, drv->sspin, 1);

        if (drv->cb)
        {
            drv->cb(drv, drv->len, drv->rxbuf, drv->txbuf, drv->arg);
        }

        return;
    }

    drv->dev->data = drv->txbuf[drv->len];
}

void spi_wait(spi_drv_t *drv)
{
    while (drv->len < drv->xferlen)
        ;
}

// Interrupt handler callbacks
static void sercom_spi0_int_handler(void)
{
    sercom_spi_int_handler(spi_drv[0]);
}

static void sercom_spi1_int_handler(void)
{
    sercom_spi_int_handler(spi_drv[1]);
}

static void sercom_spi2_int_handler(void)
{
    sercom_spi_int_handler(spi_drv[2]);
}

static void sercom_spi3_int_handler(void)
{
    sercom_spi_int_handler(spi_drv[3]);
}

static void sercom_spi4_int_handler(void)
{
    sercom_spi_int_handler(spi_drv[4]);
}

static void sercom_spi5_int_handler(void)
{
    sercom_spi_int_handler(spi_drv[5]);
}

typedef struct
{
    volatile sercom_spi_t *dev;
    void (*vector)(void);
} spi_map_t;

const spi_map_t spi_map[] = {
    {
        SERCOM0_SPI,
        sercom_spi0_int_handler,
    },
    {
        SERCOM1_SPI,
        sercom_spi1_int_handler,
    },
    {
        SERCOM2_SPI,
        sercom_spi2_int_handler,
    },
    {
        SERCOM3_SPI,
        sercom_spi3_int_handler,
    },
    {
        SERCOM4_SPI,
        sercom_spi4_int_handler,
    },
    {
        SERCOM5_SPI,
        sercom_spi5_int_handler,
    },
};

int spi_transfer(spi_drv_t *drv, int len,
                        uint8_t *rxbuf, uint8_t *txbuf,
                        spi_callback_t cb, void *arg)
{
    uint32_t irq_state = irq_save();

    drv->xferlen = len;
    drv->len = 0;
    drv->cb = cb;
    drv->arg = arg;

    drv->txbuf = txbuf;
    drv->rxbuf = rxbuf;

    port_set(drv->ssport, drv->sspin, 0);

    if (drv->xferlen)
    {
        drv->dev->data = txbuf[0];
    }
    else
    {
        port_set(drv->ssport, drv->sspin, 1);

        if (drv->cb)
        {
            drv->cb(drv, len, rxbuf, txbuf, arg);
        }
    }

    irq_restore(irq_state);

    return 0;
}

spi_drv_t *spi_master_init(int devnum,
                          spi_drv_t *drv,
                          uint8_t peripheral_id,
                          uint32_t sysclock,
                          uint32_t baud,
                          volatile port_t *ssport,
                          uint8_t sspin,
                          uint8_t dipo,
                          uint8_t dopo,
                          int32_t form)
{
    uint32_t ctrla = SERCOM_SPI_CTRLA_IBON |
                     SERCOM_SPI_CTRLA_MODE_MASTER |
                     SERCOM_SPI_CTRLA_RUNSTDBY |
                     SERCOM_SPI_DOPO(dopo) |
                     SERCOM_SPI_DIPO(dipo) |
                     SERCOM_SPI_FORM(form);
    volatile sercom_spi_t *dev = spi_map[devnum].dev;

    spi_drv[devnum] = drv;
    drv->dev = spi_map[devnum].dev;

    drv->ssport = ssport;
    drv->sspin = sspin;

    dev->ctrla = SERCOM_SPI_CTRLA_SWRST;
    spibusy_wait(dev);

    dev->ctrla = ctrla;
    spibusy_wait(dev);

    dev->baud = (sysclock / (2 * baud)) - 1;
    dev->intenset |= SERCOM_SPI_INTENSET_RXC;
    dev->ctrlb = SERCOM_SPI_CTRLB_RXEN;
    spibusy_wait(dev);

    port_set(drv->ssport, drv->sspin, 1);

    // Enable spi
    dev->ctrla |= SERCOM_SPI_CTRLA_ENABLE;
    spibusy_wait(dev);

    nvic_callback_set(peripheral_id, spi_map[devnum].vector);
    nvic_enable(peripheral_id);

    return spi_drv[devnum];
}

