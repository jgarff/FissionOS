/*
 * hwheader.h
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


#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "usb.h"
#include "usb_messages.h"
#include "usb_config.h"
#include "product.h"
#include "workqueue.h"
#include "fwheader.h"
#include "systick.h"

#include <saml_nvm.h>
#include <saml_reset.h>

#include "samd21.h"

#include "usb_vendor.h"


#define WORKER_TYPE_NONE                         0
#define WORKER_TYPE_RESET_APP                    1
#define WORKER_TYPE_RESET_BL                     2


device_info_t device_info;


void usb_vendor_worker(void *arg)
{
    uint32_t type = (uint32_t)arg;

    switch (type)
    {
        case WORKER_TYPE_RESET_APP:
            *reset_config = RESET_CONFIG_APPLICATION;
            break;

        case WORKER_TYPE_RESET_BL:
            *reset_config = RESET_CONFIG_BOOTLOADER;
            break;

        default:
            break;
    }

    saml_soft_reset();

    return;
}

workqueue_t switch_bank_wq =
{
    .callback = usb_vendor_worker,
    .arg = NULL,
};

static void rx_vendor_setup(usb_endpoint_entry_t *ep, usb_request_t *req)
{
    switch (req->request)
    {
        case USB_VENDOR_REQUEST_RESET:
            usb_txbuffer_start(ep, NULL, 0);

            if (req->value[0] & 0x1)
            {
                switch_bank_wq.arg = (void *)WORKER_TYPE_RESET_BL;
            }
            else
            {
                switch_bank_wq.arg = (void *)WORKER_TYPE_RESET_APP;
            }

            // Schedule the reset in 1 second to make sure
            // the USB request is completed first
            workqueue_add(&switch_bank_wq, SYSTICK_FREQ);

            break;

        case USB_VENDOR_REQUEST_INFO:
            usb_txbuffer_start(ep, (char *)&device_info, sizeof(device_info));

            break;

        default:
            break;
    }
}

static uint16_t rx_vendor_out(usb_endpoint_entry_t *ep)
{
    char buf[NVM_PAGE_SIZE];
	int len;

    len = usb_endpoint_buffer_read(ep, buf, sizeof(buf));

    return len;
}

void usb_vendor_init(void)
{
    device_info.bank = 0;
    device_info.size = FLASH_SIZE;
    device_info.page_size = NVM_PAGE_SIZE;
    device_info.flags = 0;

    // Register callbacks to handle vendor specific traffic.
    usb_control0_vendor_register(rx_vendor_setup, rx_vendor_out);
}


