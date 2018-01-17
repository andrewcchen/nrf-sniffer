#!/usr/bin/env python3

# Decoder for advmon
# Usage: ./advmon.py [/dev/ttyUSB0]

import sys
import struct

def read_bytes(stream, count):
    buf = bytes()
    while count - len(buf) > 0:
        buf += stream.read(count - len(buf))
    return buf

def read_packet(stream):
    # Packet format:
    # [4 byte timestamp] [1 byte S0] [1 byte length] [n byte data]
    
    hdr = read_bytes(stream, 6)
    timestamp, s0, length = struct.unpack("<LBB", hdr)
    data = read_bytes(stream, length)
    return (timestamp, s0, length, data)


def main():
    serial_port = "/dev/ttyUSB0"
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]

    stream = open(serial_port, "rb", buffering=0)
    
    while True:
        t, s0, l, data = read_packet(stream)
        print("{:d}.{:06d}: {:02x} {:02x}".format(t//1000000, t%1000000, s0, l), end="")
        for x in data:
            print(" {:02x}".format(x), end="")
        print("\n", end="")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
