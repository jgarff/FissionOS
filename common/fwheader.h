/*
 * hwheader.h
 *
 *
 * Copyright (c) 2017 Western Digital Corporation or its affiliates.
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
#ifndef __FWHEADER_H__
#define __FWHEADER_H__


typedef struct
{
    uint8_t  major;
    uint8_t  minor;
    uint8_t  micro;
    uint8_t  nano;
} __attribute__ ((packed)) version_t;

typedef struct
{
    uint16_t  magic;
#define FWHEADER_MAGIC                           0x8943
    uint16_t  flags;
    uint32_t  crc;
    uint32_t  len;
    version_t version;
} __attribute__ ((packed)) fwheader_t;

typedef struct
{
    uint32_t magic;
#define FWHEADER_V2_MAGIC                        0x0bb08944
    uint32_t flags;
    uint32_t crc;
    uint32_t len;
    version_t version;
    uint8_t  desc[108];                          
    uint32_t stack_ptr;                          // Align field to 128 byte offset
    uint32_t start_addr;
} __attribute__ ((packed)) fwheader_v2_t;



#endif /* __FWHEADER_H__ */
