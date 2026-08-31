import struct
import threading
import time

import serial


HEADER=b"\xA5\x5A"
PAYLOAD_SIZE=22


class TelemetryReader:
    def __init__(self,port,baud=115200):
        self.port=port
        self.baud=baud

        self.lock=threading.Lock()
        self.running=False

        self.data={
            "connected":False,
            "version":0,
            "post_pass":False,
            "trap":False,
            "cycles":0,
            "instructions":0,
            "memory":0,
            "mmio":0,
            "instruction_rate":0,
            "memory_rate":0,
            "mmio_rate":0,
            "cpi":0,
            "packets":0
        }

        self.previous=None
        self.previous_time=None

    def start(self):
        self.running=True

        thread=threading.Thread(
            target=self._run,
            daemon=True
        )

        thread.start()

    def _read_exact(self,ser,size):
        data=b""

        while self.running and len(data)<size:
            chunk=ser.read(size-len(data))

            if chunk:
                data+=chunk

        return data

    def _run(self):
        while self.running:
            try:
                with serial.Serial(
                    self.port,
                    self.baud,
                    timeout=1
                ) as ser:

                    with self.lock:
                        self.data["connected"]=True

                    while self.running:
                        first=ser.read(1)

                        if first!=HEADER[:1]:
                            continue

                        second=ser.read(1)

                        if second!=HEADER[1:]:
                            continue

                        payload=self._read_exact(
                            ser,
                            PAYLOAD_SIZE
                        )

                        if len(payload)!=PAYLOAD_SIZE:
                            continue

                        self._handle_packet(payload)

            except serial.SerialException:
                with self.lock:
                    self.data["connected"]=False

                time.sleep(1)

    def _handle_packet(self,payload):
        version=payload[0]
        flags=payload[1]

        cycles,instructions,memory,mmio=struct.unpack(
            "<QIII",
            payload[2:]
        )

        now=time.time()

        instruction_rate=0
        memory_rate=0
        mmio_rate=0
        cpi=0

        if self.previous is not None:
            elapsed=now-self.previous_time

            if elapsed>0:
                instruction_rate=(
                    instructions-self.previous["instructions"]
                )/elapsed

                memory_rate=(
                    memory-self.previous["memory"]
                )/elapsed

                mmio_rate=(
                    mmio-self.previous["mmio"]
                )/elapsed

                delta_cycles=cycles-self.previous["cycles"]
                delta_instr=instructions-self.previous["instructions"]

                if delta_instr>0:
                    cpi=delta_cycles/delta_instr

        self.previous={
            "cycles":cycles,
            "instructions":instructions,
            "memory":memory,
            "mmio":mmio
        }

        self.previous_time=now

        with self.lock:
            self.data.update({
                "connected":True,
                "version":version,
                "post_pass":bool(flags&0x01),
                "trap":bool(flags&0x02),
                "cycles":cycles,
                "instructions":instructions,
                "memory":memory,
                "mmio":mmio,
                "instruction_rate":instruction_rate,
                "memory_rate":memory_rate,
                "mmio_rate":mmio_rate,
                "cpi":cpi,
                "packets":self.data["packets"]+1
            })

    def snapshot(self):
        with self.lock:
            return dict(self.data)