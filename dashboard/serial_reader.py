import collections
import datetime
import struct
import threading
import time

import serial


MAGIC0=b"\xA5"
TELEMETRY_MAGIC1=b"\x5A"
RESPONSE_MAGIC1=b"\x5B"

#commands happen to share telemetry's magic1 byte (0x5A), direction alone
#(rx vs tx) tells them apart on the wire so we never needed a third value.
#named separately anyway so it doesn't look like we're reusing the
#telemetry constant by accident
COMMAND_MAGIC1=b"\x5A"

#version bumped 0x01->0x02 when payload grew 22->27 bytes for
#DATA_RAM_COUNT and WORKLOAD_ID, old field offsets stayed the same
PAYLOAD_SIZE=27

#response packet: MAGIC0 MAGIC1 VERSION CMD_ECHO STATUS DATA[0..3] CHECKSUM
#magic is read separately below so this is just the rest of it
RESPONSE_PAYLOAD_SIZE=8

CMD_PROTO_VER=0x01

CMD_PING=0x01
CMD_RESET_COUNTERS=0x02
CMD_CAPTURE_SNAPSHOT=0x03
CMD_SET_WORKLOAD=0x04
CMD_INJECT_FAULT=0x05
CMD_SYSTEM_RESET=0x06

RESP_ACK=0x00
RESP_NACK_BAD_CHECKSUM=0x01
RESP_NACK_BAD_VERSION=0x02
RESP_NACK_UNKNOWN_CMD=0x03
RESP_NACK_BAD_ARGUMENT=0x04

RESP_STATUS_NAMES={
    RESP_ACK:"ACK",
    RESP_NACK_BAD_CHECKSUM:"NACK_BAD_CHECKSUM",
    RESP_NACK_BAD_VERSION:"NACK_BAD_VERSION",
    RESP_NACK_UNKNOWN_CMD:"NACK_UNKNOWN_CMD",
    RESP_NACK_BAD_ARGUMENT:"NACK_BAD_ARGUMENT"
}

WORKLOAD_IDS={"IDLE":0,"ALU":1,"MEMORY":2,"BRANCH":3,"MMIO":4,"MIXED":5}
WORKLOAD_NAMES={v:k for k,v in WORKLOAD_IDS.items()}

#generous compared to actual response time (well under a ms at 115200
#baud), this only matters if the fpga has genuinely stopped responding
COMMAND_TIMEOUT=2.0

#recent events is enough for a demo/debug session, not meant to be a
#persistent audit log (see clear_events)
EVENT_HISTORY_SIZE=150


class TelemetryReader:
    def __init__(self,port,baud=115200):
        self.port=port
        self.baud=baud

        self.lock=threading.Lock()
        self.running=False

        #set by the reader thread whenever the port is open, cleared on
        #disconnect. send_command() writes through this same connection
        #instead of opening a second one
        self.ser=None

        #fpga only has one pending-response slot so only one command can
        #be outstanding at a time. holding this lock for the whole write
        #then wait round trip is what actually enforces that
        self.command_lock=threading.Lock()
        self.response_event=threading.Event()
        self.response_packet=None

        self.data={
            "connected":False,
            "version":0,
            "post_pass":False,
            "trap":False,
            "cycles":0,
            "instructions":0,
            "memory":0,
            "mmio":0,
            "data_ram":0,
            "workload":0,
            "instruction_rate":0,
            "memory_rate":0,
            "mmio_rate":0,
            "data_ram_rate":0,
            "cpi":0,
            "packets":0
        }

        self.previous=None
        self.previous_time=None

        #event history gets touched by both the reader thread and flask
        #request threads, so it gets its own lock instead of sharing
        #self.lock - an event append shouldn't have to wait on a
        #telemetry snapshot or vice versa
        self.event_lock=threading.Lock()
        self.event_log=collections.deque(maxlen=EVENT_HISTORY_SIZE)

        #tracks previous state for edge detection, only touched from the
        #reader thread so no lock needed here. post_pass/trap start False
        #instead of None so a healthy board's first packet logs POST PASS
        #without also logging a fake TRAP CLEAR. last_workload starts at
        #None since 0 is a real workload (idle), not "unknown"
        self.last_post_pass=False
        self.last_trap=False
        self.last_workload=None

        #set when trap goes true, consumed once trap clears and post
        #passes again - this is what makes RECOVERY COMPLETE mean
        #something instead of just two unrelated transitions
        self.recovering=False

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

                    self.ser=ser

                    with self.lock:
                        self.data["connected"]=True

                    self.add_event("connection","CONNECTED")

                    #a reconnect can't see what happened while the port was
                    #closed, so treat it like a fresh start instead of
                    #assuming anything carried over
                    self.last_post_pass=False
                    self.last_trap=False
                    self.last_workload=None
                    self.recovering=False

                    while self.running:
                        first=ser.read(1)

                        if first!=MAGIC0:
                            continue

                        second=ser.read(1)

                        #only place bytes come off the wire so it's also
                        #the only place that classifies them - telemetry
                        #and responses share the same tx line
                        if second==TELEMETRY_MAGIC1:
                            payload=self._read_exact(ser,PAYLOAD_SIZE)

                            if len(payload)==PAYLOAD_SIZE:
                                self._handle_telemetry(payload)

                        elif second==RESPONSE_MAGIC1:
                            payload=self._read_exact(ser,RESPONSE_PAYLOAD_SIZE)

                            if len(payload)==RESPONSE_PAYLOAD_SIZE:
                                self._handle_response(payload)

            except serial.SerialException:
                with self.lock:
                    was_connected=self.data["connected"]
                    self.data["connected"]=False

                #only log if we were actually connected before - if the
                #port never opened in the first place there's nothing to
                #call "disconnected"
                if was_connected:
                    self.add_event("connection","DISCONNECTED")

                time.sleep(1)

            finally:
                self.ser=None

    def _handle_response(self,payload):
        version,cmd_echo,status,d0,d1,d2,d3,checksum=struct.unpack(
            "<BBBBBBBB",
            payload
        )

        expected_checksum=version^cmd_echo^status^d0^d1^d2^d3

        #only one command can be outstanding at a time, so whoever's
        #waiting in send_command() is the one this response is for. if
        #nobody's waiting it's a stray packet and gets dropped
        self.response_packet={
            "cmd_echo":cmd_echo,
            "status":status,
            "data":d0|(d1<<8)|(d2<<16)|(d3<<24),
            "checksum_ok":checksum==expected_checksum
        }

        self.response_event.set()

    def _handle_telemetry(self,payload):
        version=payload[0]
        flags=payload[1]

        cycles,instructions,memory,mmio,data_ram=struct.unpack(
            "<QIIII",
            payload[2:26]
        )

        workload=payload[26]

        now=time.time()

        instruction_rate=0
        memory_rate=0
        mmio_rate=0
        data_ram_rate=0
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

                data_ram_rate=(
                    data_ram-self.previous["data_ram"]
                )/elapsed

                delta_cycles=cycles-self.previous["cycles"]
                delta_instr=instructions-self.previous["instructions"]

                if delta_instr>0:
                    cpi=delta_cycles/delta_instr

        self.previous={
            "cycles":cycles,
            "instructions":instructions,
            "memory":memory,
            "mmio":mmio,
            "data_ram":data_ram
        }

        self.previous_time=now

        post_pass=bool(flags&0x01)
        trap=bool(flags&0x02)

        with self.lock:
            self.data.update({
                "connected":True,
                "version":version,
                "post_pass":post_pass,
                "trap":trap,
                "cycles":cycles,
                "instructions":instructions,
                "memory":memory,
                "mmio":mmio,
                "data_ram":data_ram,
                "workload":workload,
                "instruction_rate":instruction_rate,
                "memory_rate":memory_rate,
                "data_ram_rate":data_ram_rate,
                "mmio_rate":mmio_rate,
                "cpi":cpi,
                "packets":self.data["packets"]+1
            })

        self._log_telemetry_transitions(post_pass,trap,workload)

    def _log_telemetry_transitions(self,post_pass,trap,workload):
        #only runs from the reader thread so last_post_pass/last_trap/
        #last_workload/recovering need no lock, just event_log does
        #(handled inside add_event)
        if trap!=self.last_trap:
            if trap:
                self.add_event("fault","TRAP ASSERTED")
                self.recovering=True
            else:
                self.add_event("fault","TRAP CLEAR")

            self.last_trap=trap

        if post_pass!=self.last_post_pass:
            self.add_event("post","POST PASS" if post_pass else "POST FAIL")
            self.last_post_pass=post_pass

        #checked fresh every packet, not assuming trap clear and post pass
        #happen in any particular order - they can land on the same packet
        #or different ones depending on timing
        if self.recovering and not trap and post_pass:
            self.add_event("recovery","RECOVERY COMPLETE")
            self.recovering=False

        if workload!=self.last_workload:
            workload_name=WORKLOAD_NAMES.get(workload,str(workload))
            self.add_event("workload",f"WORKLOAD ACTIVE {workload_name}")
            self.last_workload=workload

    def snapshot(self):
        with self.lock:
            return dict(self.data)

    def add_event(self,kind,message):
        now=datetime.datetime.now()

        entry={
            "time":now.strftime("%H:%M:%S.")+f"{now.microsecond//1000:03d}",
            "kind":kind,
            "message":message
        }

        #only held for the append itself, never around a serial read/write
        #or an http response, so neither side can stall the other
        with self.event_lock:
            self.event_log.append(entry)

    def get_events(self):
        with self.event_lock:
            return list(self.event_log)

    def clear_events(self):
        #host-side display history only, never touches the fpga, not to
        #be confused with RESET_COUNTERS/SYSTEM_RESET. leaves the last_*
        #trackers alone since those track real hardware state, not what's
        #currently on screen
        with self.event_lock:
            self.event_log.clear()

    def send_command(self,cmd,arg=0):
        #timeout instead of a blocking acquire so a rapid double click (or
        #two browser tabs) gets a clean "busy" back instead of queueing up
        if not self.command_lock.acquire(timeout=0.1):
            return {"ok":False,"error":"busy"}

        try:
            ser=self.ser

            if ser is None or not ser.is_open:
                return {"ok":False,"error":"disconnected"}

            arg_bytes=struct.pack("<I",arg&0xFFFFFFFF)
            checksum=CMD_PROTO_VER^cmd

            for b in arg_bytes:
                checksum^=b

            frame=MAGIC0+COMMAND_MAGIC1+bytes([CMD_PROTO_VER,cmd])+arg_bytes+bytes([checksum])

            #cleared right before writing, not right after grabbing the
            #lock, so a stale packet from a previous call can't be
            #mistaken for this one's response
            self.response_event.clear()
            self.response_packet=None

            try:
                ser.write(frame)
            except serial.SerialException:
                return {"ok":False,"error":"write_failed"}

            if not self.response_event.wait(timeout=COMMAND_TIMEOUT):
                return {"ok":False,"error":"timeout"}

            response=self.response_packet

            if response is None or not response["checksum_ok"]:
                return {"ok":False,"error":"bad_checksum"}

            if response["cmd_echo"]!=cmd:
                return {"ok":False,"error":"cmd_mismatch"}

            return {
                "ok":True,
                "status":response["status"],
                "data":response["data"]
            }
        finally:
            self.command_lock.release()