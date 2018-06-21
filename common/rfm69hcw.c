/*
 * rf69hcw.c
 *
 *
 * Copyright (c) 2018 Jeremy Garff
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

#ifdef __ATSAMD21__

#include <stdint.h>
#include <string.h>

#include "vectors.h"
#include "saml_sercom.h"
#include "console.h"

#include "rfm69hcw.h"

typedef struct
{
    char *name;
    uint8_t reg;
} rfregs_t;

rfregs_t rfregs[] = {
    {
        .name = "OpMode",
        .reg  = RFM69_REG_OPMODE,
    },
    {
        .name = "DataModul",
        .reg  = RFM69_REG_DATAMODUL,
    },
    {
        .name = "BitRateMsb",
        .reg  = RFM69_REG_BITRATEMSB,
    },
    {
        .name = "BitRateLsb",
        .reg  = RFM69_REG_BITRATELSB,
    },
    {
        .name = "FDevMsb",
        .reg  = RFM69_REG_FDEVMSB,
    },
    {
        .name = "FDevLsb",
        .reg  = RFM69_REG_FDEVLSB,
    },
    {
        .name = "FrfMsb",
        .reg  = RFM69_REG_FRFMSB,
    },
    {
        .name = "FrfMid",
        .reg  = RFM69_REG_FRFMID,
    },
    {
        .name = "FrfLsb",
        .reg  = RFM69_REG_FRFLSB,
    },
    {
        .name = "Osc1",
        .reg  = RFM69_REG_OSC1,
    },
    {
        .name = "AfcCtrl",
        .reg  = RFM69_REG_AFCCTRL,
    },
    {
        .name = "Listen1",
        .reg  = RFM69_REG_LISTEN1,
    },
    {
        .name = "Listen2",
        .reg  = RFM69_REG_LISTEN2,
    },
    {
        .name = "Listen3",
        .reg  = RFM69_REG_LISTEN3,
    },
    {
        .name = "Version",
        .reg  = RFM69_REG_VERSION,
    },
    {
        .name = "PaLevel",
        .reg  = RFM69_REG_PALEVEL,
    },
    {
        .name = "PaRamp",
        .reg  = RFM69_REG_PARAMP,
    },
    {
        .name = "Ocp",
        .reg  = RFM69_REG_OCP,
    },
    {
        .name = "Lna",
        .reg  = RFM69_REG_LNA,
    },
    {
        .name = "RxBw",
        .reg  = RFM69_REG_RXBW,
    },
    {
        .name = "AfcBw",
        .reg  = RFM69_REG_AFCBW,
    },
    {
        .name = "OokPeak",
        .reg  = RFM69_REG_OOKPEAK,
    },
    {
        .name = "OokAvg",
        .reg  = RFM69_REG_OOKAVG,
    },
    {
        .name = "OokFix",
        .reg  = RFM69_REG_OOKFIX,
    },
    {
        .name = "AfcFei",
        .reg  = RFM69_REG_AFCFEI,
    },
    {
        .name = "AfcMsb",
        .reg  = RFM69_REG_AFCMSB,
    },
    {
        .name = "AfcLsb",
        .reg  = RFM69_REG_AFCLSB,
    },
    {
        .name = "FeiMsb",
        .reg  = RFM69_REG_FEIMSB,
    },
    {
        .name = "FeiLsb",
        .reg  = RFM69_REG_FEILSB,
    },
    {
        .name = "RssiConfig",
        .reg  = RFM69_REG_RSSICONFIG,
    },
    {
        .name = "RssiValue",
        .reg  = RFM69_REG_RSSIVALUE,
    },
    {
        .name = "DioMapping1",
        .reg  = RFM69_REG_DIOMAPPING1,
    },
    {
        .name = "DioMapping2",
        .reg  = RFM69_REG_DIOMAPPING2,
    },
    {
        .name = "IrqFlags1",
        .reg  = RFM69_REG_IRQFLAGS1,
    },
    {
        .name = "IrqFlags2",
        .reg  = RFM69_REG_IRQFLAGS2,
    },
    {
        .name = "RssiThresh",
        .reg  = RFM69_REG_RSSITHRESH,
    },
    {
        .name = "RxTimeout1",
        .reg  = RFM69_REG_RXTIMEOUT1,
    },
    {
        .name = "RxTimeout2",
        .reg  = RFM69_REG_RXTIMEOUT2,
    },
    {
        .name = "PreambleMsb",
        .reg  = RFM69_REG_PREAMBLEMSB,
    },
    {
        .name = "PreambleLsb",
        .reg  = RFM69_REG_PREAMBLELSB,
    },
    {
        .name = "SyncConfig",
        .reg  = RFM69_REG_SYNCCONFIG,
    },
    {
        .name = "SyncValue1",
        .reg  = RFM69_REG_SYNCVALUE1,
    },
    {
        .name = "SyncValue2",
        .reg  = RFM69_REG_SYNCVALUE2,
    },
    {
        .name = "SyncValue3",
        .reg  = RFM69_REG_SYNCVALUE3,
    },
    {
        .name = "SyncValue4",
        .reg  = RFM69_REG_SYNCVALUE4,
    },
    {
        .name = "SyncValue5",
        .reg  = RFM69_REG_SYNCVALUE5,
    },
    {
        .name = "SyncValue6",
        .reg  = RFM69_REG_SYNCVALUE6,
    },
    {
        .name = "SyncValue7",
        .reg  = RFM69_REG_SYNCVALUE7,
    },
    {
        .name = "SyncValue8",
        .reg  = RFM69_REG_SYNCVALUE8,
    },
    {
        .name = "PacketConfig1",
        .reg  = RFM69_REG_PACKETCONFIG1,
    },
    {
        .name = "PayloadLength",
        .reg  = RFM69_REG_PAYLOADLENGTH,
    },
    {
        .name = "NodeAdrs",
        .reg  = RFM69_REG_NODEADRS,
    },
    {
        .name = "BroadcastAdrs",
        .reg  = RFM69_REG_BROADCASTADRS,
    },
    {
        .name = "AutoModes",
        .reg  = RFM69_REG_AUTOMODES,
    },
    {
        .name = "FifoThresh",
        .reg  = RFM69_REG_FIFOTHRESH,
    },
    {
        .name = "PacketConfig2",
        .reg  = RFM69_REG_PACKETCONFIG2,
    },
    {
        .name = "AesKey1",
        .reg  = RFM69_REG_AESKEY1,
    },
    {
        .name = "AesKey2",
        .reg  = RFM69_REG_AESKEY2,
    },
    {
        .name = "AesKey3",
        .reg  = RFM69_REG_AESKEY3,
    },
    {
        .name = "AesKey4",
        .reg  = RFM69_REG_AESKEY4,
    },
    {
        .name = "AesKey5",
        .reg  = RFM69_REG_AESKEY5,
    },
    {
        .name = "AesKey6",
        .reg  = RFM69_REG_AESKEY6,
    },
    {
        .name = "AesKey7",
        .reg  = RFM69_REG_AESKEY7,
    },
    {
        .name = "AesKey8",
        .reg  = RFM69_REG_AESKEY8,
    },
    {
        .name = "AesKey9",
        .reg  = RFM69_REG_AESKEY9,
    },
    {
        .name = "AesKey10",
        .reg  = RFM69_REG_AESKEY10,
    },
    {
        .name = "AesKey11",
        .reg  = RFM69_REG_AESKEY11,
    },
    {
        .name = "AesKey12",
        .reg  = RFM69_REG_AESKEY12,
    },
    {
        .name = "AesKey13",
        .reg  = RFM69_REG_AESKEY13,
    },
    {
        .name = "AesKey14",
        .reg  = RFM69_REG_AESKEY14,
    },
    {
        .name = "AesKey15",
        .reg  = RFM69_REG_AESKEY15,
    },
    {
        .name = "AesKey16",
        .reg  = RFM69_REG_AESKEY16,
    },
    {
        .name = "Temp1",
        .reg  = RFM69_REG_TEMP1,
    },
    {
        .name = "Temp2",
        .reg  = RFM69_REG_TEMP2,
    },
    {
        .name = "TestLna",
        .reg  = RFM69_REG_TESTLNA,
    },
    {
        .name = "TestPa1",
        .reg  = RFM69_REG_TESTPA1,
    },
    {
        .name = "TestPa2",
        .reg  = RFM69_REG_TESTPA2,
    },
    {
        .name = "TestDagc",
        .reg  = RFM69_REG_TESTDAGC,
    },
    {
        .name = "TestAfc",
        .reg  = RFM69_REG_TESTAFC,
    },
};

#define RF69_STATE_IDLE                          0
#define RF69_STATE_TX                            1
#define RF69_STATE_LISTEN                        2
#define RF69_STATE_RX                            3

spi_drv_t *rf69_drv = NULL;
int rf69_state = RF69_STATE_IDLE;

volatile rfbuf_t rfbuf_freelist = {
    .next = &rfbuf_freelist,
    .prev = &rfbuf_freelist,
};
volatile rfbuf_t rfbuf_txlist = {
    .next = &rfbuf_txlist,
    .prev = &rfbuf_txlist,
};
volatile rfbuf_t rfbuf_rxlist = {
    .next = &rfbuf_rxlist,
    .prev = &rfbuf_rxlist,
};


void rfbuf_append(volatile rfbuf_t *head, volatile rfbuf_t *entry)
{
    entry->next = head;
    entry->prev = head->prev;

    head->prev->next = entry;
    head->prev = entry;
}

void rfbuf_unlink(volatile rfbuf_t *entry)
{
    // Make sure it's on a list already
    if (!entry->prev || !entry->next)
    {
        return;
    }

    entry->prev->next = entry->next;
    entry->next->prev = entry->prev;

    entry->next = NULL;
    entry->prev = NULL;
}

int rfbuf_empty(void)
{
    volatile rfbuf_t *entry = rfbuf_freelist.next;

    if (entry->next == entry)
    {
        return 1;
    }

    return 0;
}

volatile rfbuf_t *rfbuf_alloc(void)
{
    volatile rfbuf_t *entry = rfbuf_freelist.next;

    if (entry->next == entry)
    {
        return NULL;
    }

    rfbuf_unlink(entry);

    return entry;
}

void rfbuf_free(volatile rfbuf_t *entry)
{
    rfbuf_append(&rfbuf_freelist, entry);
}


uint8_t rf69_stbycmd[] = {
    RFM69_REG_WRITE(RFM69_REG_OPMODE),
    RFM69_REG_OPMODE_MODE_STDBY,
};
int rf69_mode_stdby(spi_drv_t *spi_drv)
{
    int done = -1;

    while (done == -1)
    {
        done = spi_transfer(spi_drv, sizeof(rf69_stbycmd), NULL, rf69_stbycmd, NULL, NULL);
    }

    return 0;
}

void rf69_spi_rx_cb(spi_drv_t *drv, int len, uint8_t *rxbuf, uint8_t *txbuf, void *arg)
{
    uint32_t irq_state;
    volatile rfbuf_t *entry = rfbuf_txlist.next;

    irq_state = irq_save();

    // Check to see if we got a transmit queued when we were changing modes
    if (entry->next != entry)
    {
        // Yep, go back to transmitting
        rf69_state = RF69_STATE_TX;
        rf69_mode_tx(drv);
    }
    else
    {
        // Nope, ready to get packets
        rf69_state = RF69_STATE_LISTEN;
    }

    irq_restore(irq_state);
}

uint8_t rf69_rxcmd[] = {
    RFM69_REG_WRITE(RFM69_REG_OPMODE),
    RFM69_REG_OPMODE_MODE_RX,
};
int rf69_mode_rx(spi_drv_t *spi_drv)
{
    int done = -1;

    while (done == -1)
    {
        done = spi_transfer(spi_drv, sizeof(rf69_rxcmd), NULL, rf69_rxcmd, rf69_spi_rx_cb, NULL);
    }

    return 0;
}

void rf69_txdone(void)
{
    volatile rfbuf_t *entry = rfbuf_txlist.next;
    rf69_tx_cb_t cb = entry->cb;
    void *arg = entry->arg;

    rfbuf_unlink(entry);
    rfbuf_free(entry);

    if (cb)
    {
        cb(arg);
    }
}

void rf69_txnext(spi_drv_t *drv)
{
    int done = -1;

    while (done == -1)
    {
        volatile rfbuf_t *entry = rfbuf_txlist.next;

        // Done, go back to LISTEN mode
        if (entry->next == entry)
        {
            rf69_mode_rx(drv);
            return;
        }
        
        // Not done, transmit the next one.  If we get a spi failure, just try again as something else
        // was using the spi bus.
        done = spi_transfer(drv, sizeof(entry->pkt) + entry->datalen, NULL, (uint8_t *)entry->pkt, NULL, NULL);
    }
}

// Interrupt handler for the external RF interrupt line, transmit complete, and new rx
void rf69_dio0_int(void *arg)
{
    spi_drv_t *drv = (spi_drv_t *)arg;

    switch (rf69_state)
    {
        case RF69_STATE_TX:
            rf69_txdone();
            rf69_txnext(drv);

            break;

        case RF69_STATE_LISTEN:
            // TODO:  Allocate new packet and receive data
            break;

        default:
            break;
    }
}

void rf69_spi_tx_cb(spi_drv_t *drv, int len, uint8_t *rxbuf, uint8_t *txbuf, void *arg)
{
    rf69_txnext(drv);
}

uint8_t rf69_txcmd[] = {
    RFM69_REG_WRITE(RFM69_REG_OPMODE),
    RFM69_REG_OPMODE_MODE_TX,
};
int rf69_mode_tx(spi_drv_t *spi_drv)
{
    int done = -1;

    while (done == -1)
    {
        done = spi_transfer(spi_drv, sizeof(rf69_txcmd), NULL, rf69_txcmd, rf69_spi_tx_cb, NULL);
    }

    return 0;
}

int rf69_tx(spi_drv_t *spi_drv,
            uint8_t dst, uint8_t src,
            uint8_t dport, uint8_t sport,
            uint8_t *data, uint8_t len,
            rf69_tx_cb_t cb, void *arg)
{
    uint32_t irq_state;
    volatile rfbuf_t *buf;
    rf69_spi_pkt_t *pkt;

    if (len > RF69_PKT_MAX_DATA)
    {
        len = RF69_PKT_MAX_DATA;
    }

    irq_state = irq_save();
    buf = rfbuf_alloc();
    if (!buf)
    {
        irq_restore(irq_state);
        return -1;
    }
    irq_restore(irq_state);

    buf->cb = cb;
    buf->arg = arg;

    pkt = buf->pkt;
    pkt->reg = RFM69_REG_WRITE(RFM69_REG_FIFO);

    if (data)
    {
        memcpy(pkt->data, data, len);
        buf->datalen = len;
    }
    else
    {
        buf->datalen = 0;
    }

    pkt->hdr.dst = dst;
    pkt->hdr.src = src;
    pkt->hdr.dport = dport;
    pkt->hdr.sport = sport;
    pkt->hdr.len = sizeof(pkt->hdr) + buf->datalen - 1;  // Length byte not included

    irq_state = irq_save();
    rfbuf_append(&rfbuf_txlist, buf);

    // TODO:  Check for CRC interrupt with spi active state and don't try to switch state till
    //        complete
    if (rf69_state != RF69_STATE_TX)
    {
        // We have to turn on the TX state before we queue a packet if we want to get the
        // transmit complete interrupt.  Kinda lame.
        rf69_state = RF69_STATE_TX;
        rf69_mode_tx(spi_drv);
    }
    irq_restore(irq_state);

    return 0;
}

int rf69_regs_init(spi_drv_t *spi_drv, rf69_reg_init_t *reglist, int listlen)
{
    uint8_t txbuf[RF69_REG_LIST_DATA_MAX];
    int i;

    for (i = 0; i < listlen; i++)
    {
        rf69_reg_init_t *reg = &reglist[i];

        txbuf[0] = RFM69_REG_WRITE(reg->addr);
        memcpy(&txbuf[1], reg->data, reg->len);

        if (spi_transfer(spi_drv, reg->len + 1, NULL, txbuf, NULL, NULL))
        {
            return -1;
        }

        spi_wait(spi_drv);
    }

    rf69_state = RF69_STATE_LISTEN;
    rf69_mode_stdby(spi_drv);

    return 0;
}

int cmd_rf69(console_t *console, int argc, char *argv[])
{
    int i;

    console_print(console, "Registers:\r\n");
    for (i = 0; i < ARRAY_SIZE(rfregs); i++)
    {
        uint8_t txdata[] = { RFM69_REG_READ(rfregs[i].reg), 0x0 };
        uint8_t rxdata[sizeof(txdata)];
        int done = -1;

        while (done == -1)
        {
            done = spi_transfer(rf69_drv, sizeof(txdata), rxdata, txdata, NULL, NULL);
            spi_wait(rf69_drv);
        }

        console_print(console, "  %14s (%02x): %02x\r\n",
                      rfregs[i].name, rfregs[i].reg,
                      rxdata[1]);
    }

    return 0;
}

int rf69_init(spi_drv_t *spi_drv, rfbuf_t *freelist, rf69_spi_pkt_t *pktlist, int len)
{
    int i;

    rf69_drv = spi_drv;

    // Setup the packet buffer free list based on the passed in array
    for (i = 0; i < len; i++) {
        rfbuf_t *entry = &freelist[i];

        entry->pkt = &pktlist[i];
        rfbuf_append(&rfbuf_freelist, entry);
    }

    return 0;
}

#endif /* __ATSAMD21__ */

