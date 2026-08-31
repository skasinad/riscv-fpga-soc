import serial
import struct

PORT="/dev/cu.usbserial-FT4MG9OV1"
BAUD=115200

HEADER=b"\xA5\x5A"
PAYLOAD_SIZE=27 #version 0x02, +DATA_RAM_COUNT +WORKLOAD_ID


def read_exact(ser,size):
    data=b""
    while len(data)<size:
        chunk=ser.read(size-len(data))

        if chunk:
            data+=chunk

    return data


with serial.Serial(PORT,BAUD,timeout=1) as ser:
    print("Listening on",PORT)
    print()

    while True:
        byte=ser.read(1)

        if byte!=HEADER[:1]:
            continue

        if ser.read(1)!=HEADER[1:]:
            continue

        payload=read_exact(ser,PAYLOAD_SIZE)

        version=payload[0]
        flags=payload[1]

        cycles,instructions,memory,mmio,data_ram=struct.unpack(
            "<QIIII",
            payload[2:26]
        )

        workload=payload[26]

        post_pass=bool(flags&0x01)
        trap=bool(flags&0x02)

        print(
            f"CYC={cycles:>12}  "
            f"INS={instructions:>10}  "
            f"RAM={memory:>10}  "
            f"MMIO={mmio:>8}  "
            f"DRAM={data_ram:>9}  "
            f"WL={workload}  "
            f"POST={'PASS' if post_pass else 'FAIL'}  "
            f"TRAP={'YES' if trap else 'NO'}  "
            f"PROTO={version}"
        )