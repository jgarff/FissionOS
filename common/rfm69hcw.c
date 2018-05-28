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

spi_drv_t *rf69_drv = NULL;

int cmd_rf69(console_t *console, int argc, char *argv[])
{
    int i;

    console_print(console, "Registers:\r\n");
    for (i = 0; i < ARRAY_SIZE(rfregs); i++)
    {
        uint8_t txdata[] = { RFM69_REG_READ(rfregs[i].reg), 0x0 };
        uint8_t rxdata[sizeof(txdata)];

        spi_transfer(rf69_drv, sizeof(txdata), rxdata, txdata, NULL, NULL);
        spi_wait(rf69_drv);

        console_print(console, "  %14s (%02x): %02x\r\n",
                      rfregs[i].name, rfregs[i].reg,
                      rxdata[1]);
    }

    return 0;
}

int rf69_init(spi_drv_t *spi_drv)
{
    rf69_drv = spi_drv;

    return 0;
}

#endif /* __ATSAMD21__ */

