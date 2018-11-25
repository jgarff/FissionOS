/*
 * usb_config.c
 *
 *
 * Copyright (c) 2017 Jeremy Garff
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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <libusb-1.0/libusb.h>

#include "fwheader.h"

#include "product.h"


#define FLASH_BLOCK_SIZE                          4096
#define FLASH_PAGE_SIZE                           64


typedef struct {
    char *filename;
    uint8_t bus;
    uint8_t device;
    char *serial;
} arg_t;

typedef int (callback_t)(char *arg);

static struct libusb_device_handle *usbdev;

char *short_opts = "ld:bru:is:c:";

void usage(char *argv0)
{
    printf("\n");
    printf("%s: <-h> | <-l> | [-d <bus:device>] <command>\n", argv0);
    printf("\n");
    printf("Arguments:\n");
    printf("    -d <bus:device>    Specify the exact device by USB address\n");
    printf("    -s <serial number> Specify the exact device by serial number\n");
    printf("    -l                 List Available Devices\n");
    printf("    -h                 Help\n");
    printf("\n");
    printf("Commands :\n");
    printf("    -i                 Device Information\n");
    printf("    -b                 Reset Device to Bootloader\n");
    printf("    -r                 Reset Device\n");
    printf("    -u <filename>      Upload firmware file\n");
    printf("    -c <filename>      Upload configuration file\n");
    printf("\n");
}

/*
 * USB Functions
 */

int dev_list(void)
{
    struct libusb_device_handle **dev = &usbdev;
    struct libusb_device **devs;
    struct libusb_device_descriptor info;
    unsigned count, i, found = 0;

    *dev = NULL;

    if (libusb_init(NULL))
    {
        fprintf(stderr, "libusb_init failed\n");

        return -1;
    }

      // get list of devices and counts
    count = libusb_get_device_list(NULL, &devs);
    if (count <= 0)
    {
        fprintf(stderr, "Error enumerating devices\n");
        return -1;
    }

    printf("\n%6s %6s %10s\n", "Bus", "Device", "Serial");
    printf("%6s %6s %10s\n", "---", "------", "------");

    for (i = 0; i < count; i++)
    {
        libusb_get_device_descriptor(devs[i], &info);

        if ((info.idVendor == 0x0011) &&
            (info.idProduct == 0x0101))
        {
            int result;

            if (info.iSerialNumber)
            {
                uint8_t devserial[80];

                result = libusb_open(devs[i], dev);
                if (result)
                {
                    continue;
                }

                result = libusb_get_string_descriptor_ascii(*dev, info.iSerialNumber, devserial,
                                                            sizeof(devserial));

                libusb_close(*dev);

                if (result > 0)
                {
                    printf("%6d %6d %10s\n", 
                            libusb_get_bus_number(devs[i]),
                            libusb_get_port_number(devs[i]),
                            devserial);
                    found = 1;
                }
            }
        }
    }

    printf("\n");

    if (!found) {
        printf("None found\n");
    }
   
    libusb_free_device_list(devs, 1);

    libusb_exit(NULL);

    return 0;
}

int dev_open(struct libusb_device_handle **dev, int bus, int device, char *serial)
{
    struct libusb_device **devs;
    struct libusb_device_descriptor info;
    unsigned count, i;

    *dev = NULL;

    if (libusb_init(NULL))
    {
        fprintf(stderr, "libusb_init failed\n");

        return -1;
    }

      // get list of devices and counts
    count = libusb_get_device_list(NULL, &devs);
    if (count <= 0)
    {
        fprintf(stderr, "Error enumerating devices\n");
        return -1;
    }

    // walk the list, read descriptors, and dump some output from each
    for (i = 0; i < count; i++)
    {
        libusb_get_device_descriptor(devs[i], &info);

        if ((info.idVendor == 0x0011) &&
            (info.idProduct == 0x0101))
        {
            int result;

            if (serial && info.iSerialNumber)
            {
                uint8_t devserial[80];

                result = libusb_open(devs[i], dev);
                if (result)
                {
                    continue;
                }

                result = libusb_get_string_descriptor_ascii(*dev, info.iSerialNumber, devserial,
                                                            sizeof(devserial));

                libusb_close(*dev);

                if (result > 0)
                {
                    if (strcmp((char *)devserial, serial))
                    {
                        continue;
                    }
                }
            }
            else
            {
                if (bus)
                {
                    uint8_t bus_number = libusb_get_bus_number(devs[i]);
                    uint8_t device_addr = libusb_get_device_address(devs[i]);

                    // If the user specified a device, check to see if this is it.
                    // Continue looking if not.
                    if ((bus_number != bus) ||
                        (device_addr != device))
                    {
                        continue;
                    }
                }
            }

            result = libusb_open(devs[i], dev);
            if (result) {
                fprintf(stderr, "Unable to open device\n");

                libusb_free_device_list(devs, 1);
                libusb_exit(NULL);

                return -1;
            }

            break;
        }
    }
   
    libusb_free_device_list(devs, 1);

    // No matching device
    if (i == count)
    {
        libusb_exit(NULL);
        return -1;
    }

    libusb_detach_kernel_driver(*dev, 0);
    if (libusb_claim_interface(*dev, 0))
    {
        libusb_close(*dev);
        libusb_exit(NULL);

        return -1;
    }

    return 0;
}

void dev_close(struct libusb_device_handle **dev)
{
    libusb_release_interface(*dev, 0);
    libusb_attach_kernel_driver(*dev, 0);
    libusb_close(*dev);
    libusb_exit(NULL);
}

int get_device(char *arg)
{
    device_info_t device_info;
    int result;

    result = libusb_control_transfer(usbdev,
                                     LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
                                     LIBUSB_RECIPIENT_DEVICE,
                                     USB_VENDOR_REQUEST_INFO, 0, 0,
                                     (uint8_t *)&device_info, sizeof(device_info), 0);
    if (result != sizeof(device_info))
    {
        fprintf(stderr, "libusb_control_transfer failed %d\n", result);
        return -1;
    }

    printf("\n");
    printf("Bank      : %d\n", device_info.bank);
    printf("Size      : %dKb\n", device_info.size / 1024);
    printf("Page Size : %d\n", device_info.page_size);
    printf("Mode      : %s\n", device_info.flags & DEVICE_INFO_FLAGS_BOOTLOADER ?
                         "Bootloader" : "Normal");
    printf("\n");

    return 0;
}


int reset_device(char *arg)
{
    if (libusb_control_transfer(usbdev,
                                LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                LIBUSB_RECIPIENT_DEVICE,
                                USB_VENDOR_REQUEST_RESET, 0, 0,
                                NULL, 0, 0) != 0)
    {
        fprintf(stderr, "libusb_control_transfer failed\n");
        return -1;
    }

    return 0;
}

int reset_device_bl(char *arg)
{
    if (libusb_control_transfer(usbdev,
                                LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                LIBUSB_RECIPIENT_DEVICE,
                                USB_VENDOR_REQUEST_RESET, 1, 0,
                                NULL, 0, 0) != 0)
    {
        fprintf(stderr, "libusb_control_transfer failed\n");
        return -1;
    }

    return 0;
}

int update_v2(FILE *f, fwheader_v2_t *header)
{
    uint8_t buf[FLASH_PAGE_SIZE];
    uint32_t addr = 0;
    int ret, n;

    fseek(f, 0, SEEK_SET);

    n = fread(buf, 1, sizeof(buf), f);
    while (n > 0)
    {
        ret = libusb_control_transfer(usbdev,
                                      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                      LIBUSB_RECIPIENT_DEVICE,
                                      USB_VENDOR_REQUEST_FLASH,
                                      addr >> 16, addr & 0xffff,
                                      buf, n, 0);
        if (ret != n)
        {
            fprintf(stderr, "Firmware update failed %d %d\n", ret, n);
            fclose(f);

            return -1;
        }

        addr += n;
        n = fread(buf, 1, sizeof(buf), f);
    }

    if (n < 0)
    {
        fprintf(stderr, "File read error\n");
        fclose(f);

        return -1;
    }

    return 0;
}

int update_v1(FILE *f, fwheader_t *header)
{
    uint8_t buf[FLASH_BLOCK_SIZE];
    uint32_t addr = 0;
    int ret, n;

    fseek(f, sizeof(header), SEEK_SET);

    n = fread(buf, 1, sizeof(buf), f);
    while (n > 0)
    {
        ret = libusb_control_transfer(usbdev,
                                      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                      LIBUSB_RECIPIENT_DEVICE,
                                      USB_VENDOR_REQUEST_FLASH,
                                      addr >> 16, addr & 0xffff,
                                      buf, n, 0);
        if (ret != n)
        {
            fprintf(stderr, "Firmware update failed %d %d\n", ret, n);
            fclose(f);

            return -1;
        }

        addr += n;
        n = fread(buf, 1, sizeof(buf), f);
    }

    if (n < 0)
    {
        fprintf(stderr, "File read error\n");
        fclose(f);

        return -1;
    }

    ret = libusb_control_transfer(usbdev,
                                  LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                  LIBUSB_RECIPIENT_DEVICE,
                                  USB_VENDOR_REQUEST_FLASH_DONE,
                                  0, 0,
                                  (uint8_t *)&header, sizeof(header), 0);

    if (ret != sizeof(header))
    {
        fprintf(stderr, "Firmware signature failed %d %d\n", ret, n);
        fclose(f);

        return -1;
    }

    fclose(f);

    return 0;
}

int update_fw(char *arg)
{
    char *filename = arg;
    FILE *f = fopen(filename, "r");
    fwheader_t header;
    fwheader_v2_t v2_header;
    int n;

    if (!f)
    {
        fprintf(stderr, "can't open file\n");
        return -1;
    }

    n = fread(&header, 1, sizeof(header), f);
    if (n != sizeof(header))
    {
        fprintf(stderr, "can't read file\n");
        fclose(f);
        return -1;
    }

    if (header.magic == FWHEADER_MAGIC)
    {
        int result = update_v1(f, &header);
        fclose(f);

        return result;
    }

    fseek(f, 0, SEEK_SET);

    n = fread(&v2_header, 1, sizeof(v2_header), f);
    if (n != sizeof(v2_header))
    {
        fprintf(stderr, "can't read file\n");
        fclose(f);

        return -1;
    }

    if (v2_header.magic == FWHEADER_V2_MAGIC)
    {
        int result = update_v2(f, &v2_header);
        fclose(f);

        return result;
    }

    fprintf(stderr, "Not a firmware file\n");
    fclose(f);

    return -1;
}

int update_config(char *arg)
{
    char *filename = arg;
    FILE *f = fopen(filename, "r");
    uint8_t buf[FLASH_PAGE_SIZE];
    uint32_t addr = 0;
    int ret, n;

    if (!f)
    {
        fprintf(stderr, "can't open file\n");
        return -1;
    }

    n = fread(buf, 1, sizeof(buf), f);
    while (n > 0)
    {
        ret = libusb_control_transfer(usbdev,
                                      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                      LIBUSB_RECIPIENT_DEVICE,
                                      USB_VENDOR_REQUEST_CONFIG,
                                      addr >> 16, addr & 0xffff,
                                      buf, n, 0);
        if (ret != n)
        {
            fprintf(stderr, "Config update failed %d %d\n", ret, n);
            fclose(f);

            return -1;
        }

        addr += n;
        n = fread(buf, 1, sizeof(buf), f);
    }

    if (n < 0)
    {
        fprintf(stderr, "File read error\n");
        fclose(f);

        return -1;
    }

    return 0;
}

int usb_command(callback_t *callback, arg_t *arg)
{
    int ret;

    ret = dev_open(&usbdev, arg->bus, arg->device, arg->serial);
    if (ret)
    {
        printf("Device not found\n");

        return ret;
    }

    ret = callback(arg->filename); 

    dev_close(&usbdev);

    return ret;
}

void strtodev(char *s, arg_t *arg)
{
    char *bus_str = s;
    char *dev_str = NULL;

    arg->bus = 0;
    arg->device = 0;

    while (*s)
    {
        if (*s == ':')
        {
            *s++ = 0;
            dev_str = s;

            break;
        }
        else
        {
            s++;
        }
    }

    if (strlen(bus_str) && strlen(dev_str))
    {
        arg->bus = strtoul(bus_str, NULL, 0);
        arg->device = strtoul(dev_str, NULL, 0);
    }
}

int main(int argc, char *argv[])
{
    callback_t *callback = NULL;
    arg_t arg = {
        .filename = NULL,
        .serial = NULL,
        .bus = 0,
        .device = 0,
    };
    int opt;

    while ((opt = getopt(argc, argv, short_opts)) != -1)
    {
        switch (opt)
        {
            case 'l':
                dev_list();
                return 0;

            case 'd':
                strtodev(optarg, &arg);
                break;

            case 'i':
                callback = get_device;
                break;

            case 'b':
                callback = reset_device_bl;
                break;

            case 'r':
                callback = reset_device;
                break;

            case 'u':
                callback = update_fw;
                arg.filename = optarg;
                break;

            case 'c':
                callback = update_config;
                arg.filename = optarg;
                break;

            case 's':
                arg.serial = optarg;
                break;

            case 'h':
            default:
                usage(argv[0]);
                return -1;
                break;
        }
    }

    if (!callback)
    {
        usage(argv[0]);
        return -1;
    }

    // Local commands
    return usb_command(callback, &arg);
}


