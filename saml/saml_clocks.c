/*
 * saml_clocks.c
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

#include "reg.h"
#include "saml_clocks.h"


#if defined(__AT91SAML21__) || defined(__ATSAMD53__)

void gclk_setup(uint8_t clknum, uint8_t src, uint16_t div)
{
    volatile gclk_t *gclk = GCLK;
    uint32_t tmp = GCLK_GENCTRL_GENEN |
                   GCLK_GENCTRL_OE |
                   GCLK_GENCTRL_DIV(div) |
                   src;

    write32(&gclk->genctrl[clknum], tmp);
}

void gclk_peripheral_enable(uint8_t clknum, uint8_t peripheral)
{
    volatile gclk_t *gclk = GCLK;

    write32(&gclk->pchctrl[peripheral],
            GCLK_PCHCTRL_GEN(clknum) | GCLK_PCHCTRL_CHEN);
}

void gclk_peripheral_disable(uint8_t clknum, uint8_t peripheral)
{
    volatile gclk_t *gclk = GCLK;

    write32(&gclk->pchctrl[peripheral], 0);
}

#endif /* __AT91SAML21__ || __ATSAMD53__*/


#if defined(__ATSAMD20__) || defined(__ATSAMD21__)

void gclk_setup(uint8_t clknum, uint8_t src, uint16_t div)
{
    volatile gclk_t *gclk = GCLK;
    volatile uint32_t tmp;  // Use temp for unoptimized compiler

    tmp = GCLK_GENDIV_DIV(div) | GCLK_GENDIV_ID(clknum);
    write32(&gclk->gendiv, tmp);
    while (gclk->status)
        ;

    tmp = GCLK_GENCTRL_ID(clknum) |
          GCLK_GENCTRL_SRC(src) |
          GCLK_GENCTRL_OE |
          GCLK_GENCTRL_GENEN;
    write32(&gclk->genctrl, tmp);
    while (gclk->status)
        ;
}

void gclk_peripheral_enable(uint8_t clknum, uint8_t peripheral)
{
    volatile gclk_t *gclk = GCLK;
    volatile uint16_t tmp;  // Use temp for unoptimized compiles

    tmp = GCLK_CLKCTRL_ID(peripheral) | GCLK_CLKCTRL_GEN(clknum) | GCLK_CLKCTRL_CLKEN;
    write16(&gclk->clkctrl, tmp);
    while (gclk->status)
        ;
}

void gclk_peripheral_disable(uint8_t clknum, uint8_t peripheral)
{
    volatile gclk_t *gclk = GCLK;
    volatile uint16_t tmp;

    tmp = GCLK_CLKCTRL_ID(peripheral) | GCLK_CLKCTRL_GEN(clknum);
    write16(&GCLK->clkctrl, tmp);
    while (gclk->status)
        ;
}

#endif /* __ATSAMD20__ */
