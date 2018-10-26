#!/usr/bin/env python

import argparse, array

expected_serial_length = 8
nvm_page_size = 64
nvm_pages_per_row = 4

# CRC32, same algorithm used in ethernet, png, zip, etc..
def crc32(data):
    crc_poly = 0xedb88320
    crc = 0xffffffff

    for d in data:
        b = (crc & 0xff) ^ d
        for r in range(8):
            if (b & 1):
                b = ( b >> 1) ^ crc_poly;
            else:
                b = (b >> 1)
        crc = (crc >> 8) ^ b

    return crc ^ 0xffffffff

def config_gen(serial, output_file):
    if len(serial) != expected_serial_length:
        print "Invalid serial number length"
        exit(1)

    config = [ # magic
        0x34, 0x24, 0x76, 0x85,
        # version
        0, 0, 1, 0,
        # flags
        0, 0, 0, 0,
    ]
    serialnum = int(serial, 0)
    serial_array = []
    for i in range(8):
        serial_array.append((serialnum >> (8 * i)) & 0xff)
    config.extend(serial_array)

    crc = crc32(config)
    config.extend([
        # crc - little endian
        crc & 0xff, (crc >> 8) & 0xff,
        (crc >> 16) & 0xff, (crc >> 24) & 0xff,
    ])

    config.extend([0] * ((nvm_page_size * nvm_pages_per_row) - len(config)))

    outdata = array.array("B", config)
    outdata.tofile(file(output_file, "wb"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--serial', required=True, help="Serial Number")
    parser.add_argument('-o', '--output-file', required=True, help="Output binary filename")

    namespace = parser.parse_args()

    config_gen(namespace.serial, namespace.output_file)

