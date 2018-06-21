/*
 * saml_port.h
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

#include "saml_vectors.h"
#include "saml_port.h"


static ext_int_t *ext_ints[EIC_INTS_MAX] = { };


static void eic_int_handler(void)
{
    volatile eic_t *eic = EIC;
    uint16_t status = eic->intflag;
    int i;

    // Ack interrupts before processing so if they retrigger during
    // the handler we'll not miss it.
    eic->intflag = status;

    for (i = 0; i < EIC_INTS_MAX; i++)
    {
        if (status & (1 << i))
        {
            if (ext_ints[i])
            {
                ext_ints[i]->callback(ext_ints[i]->arg);
            }
        }
    }
}

void eic_int_setup(uint8_t intnum, ext_int_t *ext_int, uint8_t sense_type)
{
    uint32_t irqstate = irq_save();

    ext_ints[intnum] = ext_int;
    eic_sense(intnum, sense_type);

    irq_restore(irqstate);
}

void eic_enable(void)
{
    volatile eic_t *eic = EIC;
#if defined(__ATSAMD53__)
    int i;
#endif /* __ATSAMD53__ */

    eic->ctrla = EIC_CTRLA_ENABLE;
#if defined(__ATSAMD53__) || defined(__AT91SAML21__)
    while (eic->syncbusy)
        ;
#endif // defined(__ATSAMD53__) || defined(__AT91SAML21__)

#if defined(__ATSAMD20__) || defined(__ATSAMD21__)
    while (eic->status)
        ;
#endif // defined(__ATSAMD53__) || defined(__AT91SAML21__)


#if defined(__ATSAMD53__)
    for (i = 0; i < EIC_INTS_MAX; i++)
    {
        if (ext_ints[i])
        {
            nvic_callback_set(PERIPHERAL_ID_EXTINT0 + i, eic_int_handler);
            nvic_enable(PERIPHERAL_ID_EXTINT0 + i);
        }
    }
#else /* __ATSAMD53__ */
    nvic_callback_set(PERIPHERAL_ID_EIC, eic_int_handler);
    nvic_enable(PERIPHERAL_ID_EIC);
#endif /* __ATSAMD53__ */
}

