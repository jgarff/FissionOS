/*
 * rf69hcw.h
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

#ifndef __RFM69HCW__
#define __RFM69HCW__

#define RFM69_REG_WRITE(reg)                     (reg | (1 << 7))
#define RFM69_REG_READ(reg)                      (reg)

#define RFM69_REG_FIFO                           0x00
#define RFM69_REG_OPMODE                         0x01
#define RFM69_REG_DATAMODUL                      0x02
#define RFM69_REG_BITRATEMSB                     0x03
#define RFM69_REG_BITRATELSB                     0x04
#define RFM69_REG_FDEVMSB                        0x05
#define RFM69_REG_FDEVLSB                        0x06
#define RFM69_REG_FRFMSB                         0x07
#define RFM69_REG_FRFMID                         0x08
#define RFM69_REG_FRFLSB                         0x09
#define RFM69_REG_OSC1                           0x0a
#define RFM69_REG_AFCCTRL                        0x0b
#define RFM69_REG_LISTEN1                        0x0d
#define RFM69_REG_LISTEN2                        0x0e
#define RFM69_REG_LISTEN3                        0x0f
#define RFM69_REG_VERSION                        0x10
#define RFM69_REG_PALEVEL                        0x11
#define RFM69_REG_PARAMP                         0x12
#define RFM69_REG_OCP                            0x13
#define RFM69_REG_LNA                            0x18
#define RFM69_REG_RXBW                           0x19
#define RFM69_REG_AFCBW                          0x1a
#define RFM69_REG_OOKPEAK                        0x1b
#define RFM69_REG_OOKAVG                         0x1c
#define RFM69_REG_OOKFIX                         0x1d
#define RFM69_REG_AFCFEI                         0x1e
#define RFM69_REG_AFCMSB                         0x1f
#define RFM69_REG_AFCLSB                         0x20
#define RFM69_REG_FEIMSB                         0x21
#define RFM69_REG_FEILSB                         0x22
#define RFM69_REG_RSSICONFIG                     0x23
#define RFM69_REG_RSSIVALUE                      0x24
#define RFM69_REG_DIOMAPPING1                    0x25
#define RFM69_REG_DIOMAPPING2                    0x26
#define RFM69_REG_IRQFLAGS1                      0x27
#define RFM69_REG_IRQFLAGS2                      0x28
#define RFM69_REG_RSSITHRESH                     0x29
#define RFM69_REG_RXTIMEOUT1                     0x2a
#define RFM69_REG_RXTIMEOUT2                     0x2b
#define RFM69_REG_PREAMBLEMSB                    0x2c
#define RFM69_REG_PREAMBLELSB                    0x2d
#define RFM69_REG_SYNCCONFIG                     0x2e
#define RFM69_REG_SYNCVALUE1                     0x2f
#define RFM69_REG_SYNCVALUE2                     0x30
#define RFM69_REG_SYNCVALUE3                     0x31
#define RFM69_REG_SYNCVALUE4                     0x32
#define RFM69_REG_SYNCVALUE5                     0x33
#define RFM69_REG_SYNCVALUE6                     0x34
#define RFM69_REG_SYNCVALUE7                     0x35
#define RFM69_REG_SYNCVALUE8                     0x36
#define RFM69_REG_PACKETCONFIG1                  0x37
#define RFM69_REG_PAYLOADLENGTH                  0x38
#define RFM69_REG_NODEADRS                       0x39
#define RFM69_REG_BROADCASTADRS                  0x3a
#define RFM69_REG_AUTOMODES                      0x3b
#define RFM69_REG_FIFOTHRESH                     0x3c
#define RFM69_REG_PACKETCONFIG2                  0x3d
#define RFM69_REG_AESKEY1                        0x3e
#define RFM69_REG_AESKEY2                        0x3f
#define RFM69_REG_AESKEY3                        0x40
#define RFM69_REG_AESKEY4                        0x41
#define RFM69_REG_AESKEY5                        0x42
#define RFM69_REG_AESKEY6                        0x43
#define RFM69_REG_AESKEY7                        0x44
#define RFM69_REG_AESKEY8                        0x45
#define RFM69_REG_AESKEY9                        0x46
#define RFM69_REG_AESKEY10                       0x47
#define RFM69_REG_AESKEY11                       0x48
#define RFM69_REG_AESKEY12                       0x49
#define RFM69_REG_AESKEY13                       0x4a
#define RFM69_REG_AESKEY14                       0x4b
#define RFM69_REG_AESKEY15                       0x4c
#define RFM69_REG_AESKEY16                       0x4d
#define RFM69_REG_TEMP1                          0x4e
#define RFM69_REG_TEMP2                          0x4f
#define RFM69_REG_TESTLNA                        0x58
#define RFM69_REG_TESTPA1                        0x5a
#define RFM69_REG_TESTPA2                        0x5c
#define RFM69_REG_TESTDAGC                       0x6f
#define RFM69_REG_TESTAFC                        0x71

int cmd_rf69(console_t *console, int argc, char *argv[]);
#define CONSOLE_CMD_RF                            \
    {                                             \
        .cmdstr = "rf",                           \
        .callback = cmd_rf69,                     \
        .usage = "  rf\r\n",                      \
        .help =                                   \
            "  RFM69HCW Registers\r\n"            \
    }


int rf69_init(spi_drv_t *spi_drv);

#endif /* __RFM69HCW__ */
