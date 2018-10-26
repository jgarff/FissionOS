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
#include "saml_port.h"
#include "console.h"

#include "rfm69hcw.h"

//
// Defines
//
#define RF69_STATE_IDLE                          0
#define RF69_STATE_STDBY                         1
#define RF69_STATE_MODE_TX                       2
#define RF69_STATE_TX_START                      3
#define RF69_STATE_TX_DONE                       4
#define RF69_STATE_LISTEN                        5
#define RF69_STATE_RX                            6
#define RF69_STATE_MODE_RX                       7


//
// Issues:
//
// TODO:
//

//
// Structures
//
typedef struct
{
    char *name;
    uint8_t reg;
} rfregs_t;


//
// Forward Declarations
//
void rf69_spi_cb(spi_drv_t *drv, int rxlen, int txlen, uint8_t *rxbuf, uint8_t *txbuf, void *arg);


//
// Globals
//
spi_drv_t *rf69_drv = NULL;
uint8_t rf69_intnum = 0;
int rf69_state = RF69_STATE_IDLE;

uint32_t rf69_txcount = 0;
uint32_t rf69_rxcount = 0;
uint32_t rf69_ints = 0;
uint32_t rf69_freecount = 0;

volatile rfbuf_t *rf69_rxbuf = NULL;

volatile rfbuf_t rfbuf_freelist = {
    .next = &rfbuf_freelist,
    .prev = &rfbuf_freelist,
};
volatile rfbuf_t rfbuf_txlist = {
    .next = &rfbuf_txlist,
    .prev = &rfbuf_txlist,
};
volatile rfbuf_t rfbuf_recvd_list = {
    .next = &rfbuf_recvd_list,
    .prev = &rfbuf_recvd_list,
};

// For debug console command
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


//
// Free list helpers
//
void rfbuf_append(volatile rfbuf_t *head, volatile rfbuf_t *entry)
{
    uint32_t irq_state = irq_save();

    entry->next = head;
    entry->prev = head->prev;

    head->prev->next = entry;
    head->prev = entry;

    irq_restore(irq_state);
}

void rfbuf_unlink(volatile rfbuf_t *entry)
{
    uint32_t irq_state = irq_save();

    // Make sure it's on a list already
    if (!entry->prev || !entry->next)
    {
        irq_restore(irq_state);
        return;
    }

    entry->prev->next = entry->next;
    entry->next->prev = entry->prev;

    entry->next = NULL;
    entry->prev = NULL;

    irq_restore(irq_state);
}

volatile rfbuf_t *rfbuf_dequeue(volatile rfbuf_t *head)
{
    uint32_t irq_state = irq_save();
    volatile rfbuf_t *entry = head->next;

    if (entry == head)
    {
        irq_restore(irq_state);
        return NULL;
    }
    irq_restore(irq_state);

    rfbuf_unlink(entry);

    return entry;
}

int rfbuf_empty(void)
{
    uint32_t irq_state = irq_save();
    volatile rfbuf_t *entry = rfbuf_freelist.next;

    if (entry->next == entry)
    {
        irq_restore(irq_state);
        return 1;
    }

    irq_restore(irq_state);

    return 0;
}

// To/From the allocation freelist
volatile rfbuf_t *rfbuf_alloc(void)
{
    volatile rfbuf_t *entry;
    uint32_t irq_state;

    irq_state = irq_save();
    entry = rfbuf_freelist.next;
    if (entry->next == entry)
    {
        irq_restore(irq_state);
        return NULL;
    }
    rf69_freecount--;
    irq_restore(irq_state);

    rfbuf_unlink(entry);

    return entry;
}

void rfbuf_free(volatile rfbuf_t *entry)
{
    uint32_t irq_state = irq_save();
    rf69_freecount++;
    irq_restore(irq_state);

    rfbuf_append(&rfbuf_freelist, entry);

    if (rf69_state == RF69_STATE_STDBY)
    {
        rf69_mode_rx(rf69_drv);
    }
}


//
// Mode change functions
//
uint8_t rf69_stbycmd[] = {
    RFM69_REG_WRITE(RFM69_REG_OPMODE),
    RFM69_REG_OPMODE_MODE_STDBY,
};
int rf69_mode_stdby(spi_drv_t *spi_drv)
{
    int irq_state;
    int done = -1;

    irq_state = irq_save();
    rf69_state = RF69_STATE_STDBY;
    eic_int_disable(rf69_intnum);
    irq_restore(irq_state);

    while (done == -1)
    {
        done = spi_transfer(spi_drv, 0, sizeof(rf69_stbycmd), NULL, rf69_stbycmd, rf69_spi_cb, NULL);
    }

    return 0;
}

uint8_t rf69_sleepcmd[] = {
    RFM69_REG_WRITE(RFM69_REG_OPMODE),
    RFM69_REG_OPMODE_MODE_SLEEP,
};
int rf69_mode_sleep(spi_drv_t *spi_drv)
{
    int done = -1;

    while (done == -1)
    {
        done = spi_transfer(spi_drv, 0, sizeof(rf69_sleepcmd), NULL, rf69_sleepcmd, NULL, NULL);
    }

    return 0;
}

uint8_t rf69_rxcmd[] = {
    RFM69_REG_WRITE(RFM69_REG_OPMODE),
    RFM69_REG_OPMODE_MODE_RX,
};
int rf69_mode_rx(spi_drv_t *spi_drv)
{
    int irq_state;
    int done = -1;

    irq_state = irq_save();
    if (!rf69_rxbuf)
    {
        rf69_rxbuf = rfbuf_alloc();
        if (!rf69_rxbuf)
        {
            irq_restore(irq_state);

            rf69_mode_stdby(spi_drv);

            return 0;
        }
    }

    rf69_state = RF69_STATE_MODE_RX;
    irq_restore(irq_state);

    while (done == -1)
    {
        done = spi_transfer(spi_drv, 0, sizeof(rf69_rxcmd), NULL, rf69_rxcmd, rf69_spi_cb, NULL);
    }

    return 0;
}

uint8_t rf69_txcmd[] = {
    RFM69_REG_WRITE(RFM69_REG_OPMODE),
    RFM69_REG_OPMODE_MODE_TX,
};
int rf69_mode_tx(spi_drv_t *spi_drv)
{
    int irq_state;
    int done;

    irq_state = irq_save();
    rf69_state = RF69_STATE_MODE_TX;
    eic_int_disable(rf69_intnum);
    irq_restore(irq_state);

    done = -1;
    while (done == -1)
    {
        done = spi_transfer(spi_drv, 0, sizeof(rf69_txcmd), NULL, rf69_txcmd, rf69_spi_cb, NULL);
    }

    return 0;
}

//
// Transmit state machine
//
void rf69_txdone(void)
{
    volatile rfbuf_t *entry = rfbuf_txlist.next;
    rf69_tx_cb_t cb = entry->cb;
    void *arg = entry->arg;

    rfbuf_unlink(entry);
    rfbuf_free(entry);

    rf69_txcount++;

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

        rf69_state = RF69_STATE_TX_START;
        
        // Not done, transmit the next one.  If we get a spi failure, just try again as something else
        // was using the spi bus.
        done = spi_transfer(drv, 0, sizeof(entry->pkt->hdr) + entry->datalen + 
                            sizeof(entry->pkt->hdr.len),
                            NULL, (uint8_t *)entry->pkt, rf69_spi_cb, NULL);
    }
}

//
// Receive
//
void rf69_rx_wq_handler(void *arg)
{
    volatile rfbuf_t *rxentry;
    
    rxentry = rfbuf_dequeue(&rfbuf_recvd_list);
    while (rxentry)
    {
        rf_recv(rxentry);

        rxentry = rfbuf_dequeue(&rfbuf_recvd_list);
    }
}

workqueue_t rf69_rx_wq =
{
    .callback = rf69_rx_wq_handler,
    .arg = NULL,
};

void rf69_spi_cb(spi_drv_t *drv, int rxlen, int txlen, uint8_t *rxbuf, uint8_t *txbuf, void *arg)
{
    uint32_t irq_state;
    volatile rfbuf_t *txentry = rfbuf_txlist.next;

    irq_state = irq_save();

    switch (rf69_state)
    {
        case RF69_STATE_MODE_RX:
            rf69_state = RF69_STATE_LISTEN;
            eic_int_enable(rf69_intnum);
            break;

        case RF69_STATE_MODE_TX:
            rf69_txnext(drv);
            break;

        case RF69_STATE_TX_START:
            rf69_state = RF69_STATE_TX_DONE;
            eic_int_enable(rf69_intnum);
            break;

        case RF69_STATE_RX:
            rf69_state = RF69_STATE_LISTEN;
            if (rf69_rxbuf) {
                // Queue for packet parsing
                rfbuf_append(&rfbuf_recvd_list, rf69_rxbuf);
                workqueue_add(&rf69_rx_wq, 0);  // Schedule immediately

                rf69_rxcount++;

                // Allocate the next receive packet
                rf69_rxbuf = rfbuf_alloc();
                // If we couldn't get a receive packet, and there is nothing left to transmit
                // go to standby
                if (!rf69_rxbuf && (txentry->next == txentry))
                {
                    rf69_mode_stdby(drv);
                    break;
                }
            }

            // Check to see if we got a transmit queued when we were changing modes
            if (txentry->next != txentry)
            {
                // Yep, go back to transmitting
                rf69_mode_tx(drv);
            }
            else
            {
                // Nope, ready to get packets
                eic_int_enable(rf69_intnum);
            }
            break;

        default:
            break;
    }

    irq_restore(irq_state);
}

void rf69_rx_handler(spi_drv_t *spi_drv)
{
    uint32_t irq_state;
    rf69_spi_pkt_t *pkt;

    irq_state = irq_save();
    if (!rf69_rxbuf)
    {
        // We should never be in a position to receive packets without a buffer in IDLE mode
        while(1);

        irq_restore(irq_state);
        return;
    }
    rf69_state = RF69_STATE_RX;
    irq_restore(irq_state);

    pkt = rf69_rxbuf->pkt;
    pkt->reg = RFM69_REG_READ(RFM69_REG_FIFO);

    spi_transfer(spi_drv, sizeof(*pkt), 1, (uint8_t *)pkt, (uint8_t *)pkt, rf69_spi_cb, NULL);
}

void rf69_spi_int_cb(spi_drv_t *drv, int rxlen, int txlen, uint8_t *rxbuf, uint8_t *txbuf, void *arg)
{
    uint8_t irqflags2 = rxbuf[1];

    switch (rf69_state)
    {
        case RF69_STATE_TX_DONE:
            if (irqflags2 & RFM69_REG_IRQFLAGS2_PACKETSENT)
            {
                rf69_txdone();
                rf69_txnext(drv);
            } else {
                eic_int_enable(rf69_intnum);
            }
            break;

        case RF69_STATE_LISTEN:
            if (irqflags2 & RFM69_REG_IRQFLAGS2_CRCOK)
            {
                rf69_rx_handler(drv);
            }
            else
            {
                eic_int_enable(rf69_intnum);
            }
            break;

        default:
            break;
    }
}

//
// External Interrupt handler for the external RF interrupt line, transmit complete, and new rx
//
uint8_t rf69_irqflags2_cmd[] = {
    RFM69_REG_READ(RFM69_REG_IRQFLAGS2),
    0,
};
uint8_t rf69_irqflags2_data[sizeof(rf69_irqflags2_cmd)];
int rf69_mode_irqflags2(spi_drv_t *spi_drv)
{
    int done = -1;

    while (done == -1)
    {
        done = spi_transfer(spi_drv, sizeof(rf69_irqflags2_data), sizeof(rf69_irqflags2_cmd),
                            rf69_irqflags2_data, rf69_irqflags2_cmd, rf69_spi_int_cb, NULL);
    }

    return 0;
}

void rf69_dio0_int(void *arg) {
    spi_drv_t *drv = (spi_drv_t *)arg;

    // Disable external interrupt
    eic_int_disable(rf69_intnum);

    rf69_ints++;

    rf69_mode_irqflags2(drv);
}

//
// Transmit API
//
int rf69_tx_buf(spi_drv_t *spi_drv,
                volatile rfbuf_t *buf,
                uint8_t *data, uint8_t len,
                rf69_tx_cb_t cb, void *arg)
{
    rf69_spi_pkt_t *pkt;

    if (len > RF69_PKT_MAX_DATA)
    {
        len = RF69_PKT_MAX_DATA;
    }

    buf = rfbuf_alloc();
    if (!buf)
    {
        return -1;
    }

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

    pkt->hdr.len = sizeof(pkt->hdr) + buf->datalen - 1;  // Length byte not included

    rfbuf_append(&rfbuf_txlist, buf);

    if (rf69_state == RF69_STATE_LISTEN)
    {
        // We have to turn on the TX state before we queue a packet if we want to get the
        // transmit complete interrupt.  Kinda lame.
        rf69_mode_tx(spi_drv);
    }

    return 0;
}

int rf69_tx(spi_drv_t *spi_drv,
            uint8_t dst, uint8_t src,
            uint8_t dport, uint8_t sport,
            void *data, uint8_t len,
            rf69_tx_cb_t cb, void *arg)
{
    volatile rfbuf_t *buf;
    rf69_spi_pkt_t *pkt;

    if (len > RF69_PKT_MAX_DATA)
    {
        len = RF69_PKT_MAX_DATA;
    }

    buf = rfbuf_alloc();
    if (!buf)
    {
        return -1;
    }

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

    rfbuf_append(&rfbuf_txlist, buf);

    switch (rf69_state)
    {
        case RF69_STATE_IDLE:
        case RF69_STATE_LISTEN:
        case RF69_STATE_STDBY:
            // We have to turn on the TX state before we queue a packet if we want to get the
            // transmit complete interrupt.  Kinda lame.
            rf69_mode_tx(spi_drv);
            break;

        default:
            break;
    }

    return 0;
}

//
// Console debug
//
int cmd_rf69_regs(console_t *console, int argc, char *argv[])
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
            done = spi_transfer(rf69_drv, sizeof(rxdata), sizeof(txdata), rxdata, txdata, NULL, NULL);
            spi_wait(rf69_drv);
        }

        console_print(console, "  %14s (%02x): %02x\r\n",
                      rfregs[i].name, rfregs[i].reg,
                      rxdata[1]);
    }

    return 0;
}

int cmd_rf69_stats(console_t *console, int argc, char *argv[])
{
    console_print(console, "Stats:\r\n");
    console_print(console, "  IRQ Count  : %d\r\n", rf69_ints);
    console_print(console, "  TX Count   : %d\r\n", rf69_txcount);
    console_print(console, "  RX Count   : %d\r\n", rf69_rxcount);
    console_print(console, "  Free Count : %d\r\n", rf69_freecount);

    return 0;
}

int cmd_rf69(console_t *console, int argc, char *argv[])
{
    if (argc < 2)
    {
        cmd_help_usage(console, argv[0]);
        return 0;
    }

    if ((argc == 2) && (!strcmp(argv[1], "regs")))
    {
        return cmd_rf69_regs(console, argc, argv);
    }
    else if ((argc == 2) && (!strcmp(argv[1], "stats")))
    {
        return cmd_rf69_stats(console, argc, argv);
    }
    else
    {
        cmd_help_usage(console, argv[0]);
    }

    return 0;
}

//
// Initialization
//
int rf69_regs_init(spi_drv_t *spi_drv, rf69_reg_init_t *reglist, int listlen)
{
    uint8_t txbuf[RF69_REG_LIST_DATA_MAX];
    int i;

    for (i = 0; i < listlen; i++)
    {
        rf69_reg_init_t *reg = &reglist[i];

        txbuf[0] = RFM69_REG_WRITE(reg->addr);
        memcpy(&txbuf[1], reg->data, reg->len);

        if (spi_transfer(spi_drv, 0, reg->len + 1, NULL, txbuf, NULL, NULL))
        {
            return -1;
        }

        spi_wait(spi_drv);
    }

    rf69_mode_rx(spi_drv);
    spi_wait(spi_drv);

    return 0;
}

int rf69_init(spi_drv_t *spi_drv, rfbuf_t *freelist, rf69_spi_pkt_t *pktlist, int len, uint8_t intnum)
{
    int i;

    rf69_drv = spi_drv;
    rf69_intnum = intnum;

    // Setup the packet buffer free list based on the passed in array
    for (i = 0; i < len; i++) {
        rfbuf_t *entry = &freelist[i];

        entry->pkt = &pktlist[i];
        rfbuf_append(&rfbuf_freelist, entry);
    }

    rf69_freecount = i;

    return 0;
}

#endif /* __ATSAMD21__ */

