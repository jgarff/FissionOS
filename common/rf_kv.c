/*
 * rf_kv.c
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


//
// Issues:
//
// TODO:
//

//
// Structures
//

//
// Forward Declarations
//


//
// Globals
//
rf_kv_t *rf_kvs;
int rf_kv_len = 0;

int rf_kv_register(uint16_t kv, rf_kv_cb_t cb, void *arg)
{
    int i;

    for (i = 0; i < rf_kv_len; i++)
    {
        if (rf_kvs[i].flags & RF_PORT_FLAGS_ALLOC)
        {
            continue;
        }

        rf_kvs[i].flags |= RF_PORT_FLAGS_ALLOC;
        rf_kvs[i].kv = kv;
        rf_kvs[i].cb = cb;
        rf_kvs[i].arg = arg;

        return 0;
    }

    return -1;
}

void rf_kv_init(rf_kv_t *kvs, int len)
{
    int i;

    rf_kvs = kvs;

    for (i = 0; i < len; i++)
    {
        rf_kv_t *kv = &rf_kvs[i];

        kv->kv = 0;
        kv->flags = 0;
        kv->cb = NULL;
        kv->arg = NULL;
    }

    rf_kv_len = len;
}

void rf_kv_recv(volatile rfbuf_t *buf, rf69_pkt_header_t *hdr, void *arg)
{
    rf_kv_pkt_t *kvpkt = (rf_kv_pkt_t *)hdr->data;
    int i;

    if (hdr->len < sizeof(*kvpkt))
    {
        goto done;
    }

    if (hdr->len > RF69_PKT_MAX_KV_DATA)
    {
        goto done;
    }

    for (i = 0; i < rf_kv_len; i++)
    {
        rf_kv_t *kv = &rf_kvs[i];

        if ((kv->flags & RF_KV_FLAGS_ALLOC) &&
            (kv->kv == kvpkt->key))
        {
            if (kv->cb)
            {
                kv->cb(buf, hdr, kvpkt, kv->arg);
                return;
            }
        }
    }

done:
    rfbuf_free(buf);
}

#endif /* __ATSAMD21__ */

