/*
 * rf_recv.c
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
rf_port_t *rf_ports;
int rf_port_len = 0;

int rf_port_register(uint8_t port, rf_port_cb_t cb, void *arg)
{
    int i;

    for (i = 0; i < rf_port_len; i++)
    {
        if (rf_ports[i].flags & RF_PORT_FLAGS_ALLOC)
        {
            continue;
        }

        rf_ports[i].flags |= RF_PORT_FLAGS_ALLOC;
        rf_ports[i].port = port;
        rf_ports[i].cb = cb;
        rf_ports[i].arg = arg;

        return 0;
    }

    return -1;
}

void rf_port_init(rf_port_t *ports, int len)
{
    int i;

    rf_ports = ports;

    for (i = 0; i < len; i++)
    {
        rf_port_t *port = &ports[i];
        
        port->port = 0;
        port->flags = 0;
        port->cb = NULL;
        port->arg = NULL;
    }

    rf_port_len = len;
}

void rf_recv(volatile rfbuf_t *buf)
{
    rf69_pkt_header_t *hdr = &buf->pkt->hdr;
    int i;

    if (hdr->len < sizeof(*hdr))
    {
        goto done;
    }

    if (hdr->len > RF69_PKT_MAX_DATA)
    {
        goto done;
    }

    for (i = 0; i < rf_port_len; i++)
    {
        rf_port_t *port = &rf_ports[i];

        if ((port->flags & RF_PORT_FLAGS_ALLOC) &&
            (port->port == buf->pkt->hdr.dport))
        {
            if (port->cb)
            {
                port->cb(buf, hdr, port->arg);
                return;
            }
        }
    }

done:

    rfbuf_free(buf);
}

#endif /* __ATSAMD21__ */

