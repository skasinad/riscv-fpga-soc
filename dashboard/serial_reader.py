import collections
import datetime
import struct
import threading
import time

import serial


MAGIC0=b"\xA5"
TELEMETRY_MAGIC1=b"\x5A"
RESPONSE_MAGIC1=b"\x5B"

# Host->FPGA command frames happen to share telemetry's magic1 byte (0x5A)
# -- direction alone (RX vs TX) already disambiguates them on the wire, so
# the protocol didn't need a third value. Named separately here anyway so
# a command frame isn't misread as "reusing the telemetry constant".
COMMAND_MAGIC1=b"\x5A"

# M4: telemetry version bumped 0x01->0x02, payload grew 22->27 bytes to add
# DATA_RAM_COUNT and WORKLOAD_ID. Old field order/offsets are unchanged --
# the new fields are appended at the end.
PAYLOAD_SIZE=27

# Response packet (M3): MAGIC0 MAGIC1 VERSION CMD_ECHO STATUS DATA[0..3]
# CHECKSUM. The 2-byte magic is read separately below, so this is just the
# remaining VERSION+CMD_ECHO+STATUS+DATA(4)+CHECKSUM tail.
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

# Generous relative to actual response latency (well under a millisecond of
# UART time at 115200 baud) -- this only matters if the FPGA has genuinely
# stopped responding, in which case the caller should hear back well
# before a user gives up on the button.
COMMAND_TIMEOUT=2.0

# M7: recent events is enough for a demo/debug session -- this isn't meant
# to be a persistent audit log (see TelemetryReader.clear_events).
EVENT_HISTORY_SIZE=150


class TelemetryReader:
    def __init__(self,port,baud=115200):
        self.port=port
        self.baud=baud

        self.lock=threading.Lock()
        self.running=False

        # Set by the reader thread whenever the port is open, cleared on
        # disconnect. send_command() reads this to write commands through
        # the same connection the reader thread owns, instead of opening a
        # second one.
        self.ser=None

        # The FPGA's response arbiter only has one pending-response slot,
        # so only one host command may be outstanding at a time. Holding
        # this lock for the full write-then-wait round trip is what
        # enforces that, not just a convenience against races.
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

        # M7: event history is shared between the reader thread (telemetry
        # state transitions) and Flask request threads (command outcomes),
        # so it gets its own lock rather than reusing self.lock -- an event
        # append should never have to wait behind a telemetry snapshot, or
        # vice versa.
        self.event_lock=threading.Lock()
        self.event_log=collections.deque(maxlen=EVENT_HISTORY_SIZE)

        # Previous-state trackers for edge detection, read/written only
        # from the reader thread (_handle_telemetry runs there exclusively)
        # so they need no lock of their own. post_pass/trap start at False
        # rather than None so a healthy board's first telemetry packet
        # logs POST PASS without also logging a spurious TRAP CLEAR (there
        # was nothing to clear). last_workload starts at None since 0 is a
        # real workload (IDLE), not "unknown".
        self.last_post_pass=False
        self.last_trap=False
        self.last_workload=None

        # Set when trap transitions to True, consumed once trap has
        # cleared AND POST has passed again -- this is what lets
        # RECOVERY COMPLETE mean "actually recovered", not just "two
        # unrelated transitions happened at some point".
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

                    # A reconnect has no visibility into what happened
                    # while the port was closed, so treat it like a fresh
                    # start for logging purposes rather than assuming
                    # continuity with whatever was true before the gap.
                    self.last_post_pass=False
                    self.last_trap=False
                    self.last_workload=None
                    self.recovering=False

                    while self.running:
                        first=ser.read(1)

                        if first!=MAGIC0:
                            continue

                        second=ser.read(1)

                        # This is the one place bytes come off the wire, so
                        # it's also the one place that classifies them --
                        # telemetry and responses share the physical TX
                        # line, and nothing downstream can afford to guess.
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

                # Only log a transition -- if the port was never open in
                # the first place (e.g. FPGA not yet plugged in when the
                # app started), there's nothing to report as "disconnected".
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

        # Only one command can be outstanding at a time (command_lock
        # enforces that), so whoever is currently waiting in send_command()
        # is the intended recipient of the next response packet seen here.
        # If nobody's waiting this is a stray/duplicate and gets dropped.
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
        # Only runs from the reader thread (the sole caller of
        # _handle_telemetry), so last_post_pass/last_trap/last_workload/
        # recovering need no lock -- only event_log itself (touched from
        # both this thread and Flask threads) does, inside add_event.
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

        # Checked fresh on every packet where either field changed, not
        # assumed to happen in a specific order -- trap clearing and POST
        # re-passing can land on the same telemetry packet or different
        # ones depending on timing, and this only cares about the combined
        # state, not which one changed most recently.
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

        # Held only for the append itself, never around a serial read/
        # write or an HTTP response -- a slow Flask request can't stall
        # the reader thread's own logging, or vice versa.
        with self.event_lock:
            self.event_log.append(entry)

    def get_events(self):
        with self.event_lock:
            return list(self.event_log)

    def clear_events(self):
        # Host-side display history only. This never touches the FPGA and
        # must not be confused with RESET_COUNTERS/SYSTEM_RESET. Also
        # deliberately doesn't reset last_post_pass/last_trap/
        # last_workload/recovering -- those track real hardware state, not
        # what's currently displayed, so clearing the log can't cause the
        # next telemetry packet to spuriously re-log a state that hasn't
        # actually changed.
        with self.event_lock:
            self.event_log.clear()

    def send_command(self,cmd,arg=0):
        # acquire(timeout=...) rather than a blocking acquire so a rapid
        # double-click (or two browser tabs) gets a clean "busy" result
        # instead of silently queueing behind whatever's already in flight
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

            # cleared right before writing, not right after opening the
            # lock, so a stale event/packet from a previous call can never
            # be mistaken for this one's response
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