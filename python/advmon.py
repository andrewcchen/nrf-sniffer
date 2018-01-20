#!/usr/bin/env python3

# Copyright (c) 2018, Andrew Chen <andrew.chuanye.chen@gmail.com>
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
    # [4 byte timestamp] [1 byte flags] [1 byte channel id] [2 bytes RFU]
    # [1 byte S0] [1 byte length] [n byte data]
    
    hdr = read_bytes(stream, 10)
    ts, flags, chn, s0, length = struct.unpack("<LBBxxBB", hdr)
    data = read_bytes(stream, length)
    return (ts, flags, chn, s0, length, data)


def main():
    serial_port = "/dev/ttyUSB0"
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]

    stream = open(serial_port, "rb", buffering=0)
    
    while True:
        t, flags, chn, s0, l, data = read_packet(stream)
        print("{:d}.{:06d}: ".format(t//1000000, t%1000000), end="")
        print("ch{:d} ".format(chn), end="")
        if (flags & 1) == 1:
            crc = "crcok  "
        else:
            crc = "crcFAIL"
        print(crc + ": ", end="")
        print("{:02x} {:02x}".format(s0, l), end="")
        for x in data:
            print(" {:02x}".format(x), end="")
        print("\n", end="")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
