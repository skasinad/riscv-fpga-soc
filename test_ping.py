import serial
import struct
import sys
import time

PORT = "/dev/cu.usbserial-FT4MG9OV1"
BAUD = 115200

#host to FPGA command frame
#checksum = XOR of VERSION, CMD, ARG[0..3]
CMD_MAGIC = b"\xA5\x5A"
PROTO_VERSION = 0x01
CMD_PING = 0x01
CMD_RESET_COUNTERS = 0x02
CMD_CAPTURE_SNAPSHOT = 0x03

#existing telemetry protocol (unchanged)... flags byte gains bit2 in M1, bit3 in M2
TELEMETRY_MAGIC = b"\xA5\x5A"
TELEMETRY_PAYLOAD_SIZE = 22
FLAG_PING_SEEN = 0x04
FLAG_SNAPSHOT_SEEN = 0x08

#M3 response packet: MAGIC0 MAGIC1 VERSION CMD_ECHO STATUS DATA[0..3] CHECKSUM
#same checksum convention as the command frame, magic1 0x5B distinguishes it
#from a telemetry packet on the same wire
RESPONSE_MAGIC = b"\xA5\x5B"
RESPONSE_PAYLOAD_SIZE = 8  # VERSION+CMD_ECHO+STATUS+DATA(4)+CHECKSUM

RESP_ACK = 0x00
RESP_NACK_BAD_CHECKSUM = 0x01
RESP_NACK_BAD_VERSION = 0x02
RESP_NACK_UNKNOWN_CMD = 0x03
RESP_STATUS_NAMES = {
    RESP_ACK: "ACK",
    RESP_NACK_BAD_CHECKSUM: "NACK_BAD_CHECKSUM",
    RESP_NACK_BAD_VERSION: "NACK_BAD_VERSION",
    RESP_NACK_UNKNOWN_CMD: "NACK_UNKNOWN_CMD",
}


def build_frame(cmd, arg=0, checksum=None):
    arg_bytes = struct.pack("<I", arg)
    if checksum is None:
        checksum = PROTO_VERSION ^ cmd
        for b in arg_bytes:
            checksum ^= b
    return CMD_MAGIC + bytes([PROTO_VERSION, cmd]) + arg_bytes + bytes([checksum])


def read_packet(ser):
    """Reads one telemetry or response packet, distinguished by magic1.
    Returns None on a timeout/malformed header -- caller decides whether
    that's worth retrying."""
    b0 = ser.read(1)
    if b0 != b"\xA5":
        return None

    b1 = ser.read(1)

    if b1 == b"\x5A":
        payload = ser.read(TELEMETRY_PAYLOAD_SIZE)
        if len(payload) != TELEMETRY_PAYLOAD_SIZE:
            return None

        flags = payload[1]
        cycles, instructions, memory, mmio = struct.unpack("<QIII", payload[2:])
        return {"type": "telemetry", "flags": flags, "cycles": cycles,
                "instructions": instructions, "memory": memory, "mmio": mmio}

    if b1 == b"\x5B":
        payload = ser.read(RESPONSE_PAYLOAD_SIZE)
        if len(payload) != RESPONSE_PAYLOAD_SIZE:
            return None

        version, cmd_echo, status = payload[0], payload[1], payload[2]
        data = struct.unpack("<I", payload[3:7])[0]
        checksum = payload[7]

        calc = version ^ cmd_echo ^ status
        for b in payload[3:7]:
            calc ^= b

        return {"type": "response", "version": version, "cmd_echo": cmd_echo,
                "status": status, "data": data, "checksum_ok": checksum == calc}

    return None


def send_command_wait_response(ser, cmd, arg=0, checksum=None, max_packets=10):
    """Sends a command frame and waits for its response, logging (and
    skipping) any telemetry packets that show up first -- this is what
    actually proves the host can tell the two packet types apart while
    telemetry keeps flowing."""
    ser.write(build_frame(cmd, arg, checksum))

    for _ in range(max_packets):
        pkt = read_packet(ser)

        if pkt is None:
            continue

        if pkt["type"] == "telemetry":
            print(f"  (telemetry packet while waiting: flags=0x{pkt['flags']:02x})")
            continue

        return pkt

    return None


def test_ping(ser):
    print("--- Test A: PING ACK ---")
    resp = send_command_wait_response(ser, CMD_PING)

    if resp is None:
        print("FAIL: no response received")
        return False

    print(f"  response: cmd_echo=0x{resp['cmd_echo']:02x} status={RESP_STATUS_NAMES.get(resp['status'], resp['status'])} "
          f"checksum_ok={resp['checksum_ok']}")

    if resp["cmd_echo"] != CMD_PING or resp["status"] != RESP_ACK or not resp["checksum_ok"]:
        print("FAIL: unexpected response fields")
        return False

    print("PASS")
    return True


def test_reset_counters(ser):
    print("--- Test B: RESET_COUNTERS ACK ---")

    before = read_packet(ser)
    while before is not None and before["type"] != "telemetry":
        before = read_packet(ser)

    if before is None:
        print("FAIL: no telemetry before reset")
        return False
    print(f"  before: instructions={before['instructions']}")

    resp = send_command_wait_response(ser, CMD_RESET_COUNTERS)
    if resp is None:
        print("FAIL: no response received")
        return False

    print(f"  response: cmd_echo=0x{resp['cmd_echo']:02x} status={RESP_STATUS_NAMES.get(resp['status'], resp['status'])} "
          f"checksum_ok={resp['checksum_ok']}")

    if resp["cmd_echo"] != CMD_RESET_COUNTERS or resp["status"] != RESP_ACK or not resp["checksum_ok"]:
        print("FAIL: unexpected response fields")
        return False

    # the CPU keeps running between the command and the next telemetry
    # sample, so this won't read exactly 0 -- it just has to be far below
    # "before" rather than a continuation of the same trend
    after = read_packet(ser)
    while after is not None and after["type"] != "telemetry":
        after = read_packet(ser)

    if after is None:
        print("FAIL: no telemetry after reset")
        return False
    print(f"  after:  instructions={after['instructions']}")

    if after["instructions"] >= before["instructions"]:
        print("FAIL: instruction count did not drop")
        return False

    later = read_packet(ser)
    while later is not None and later["type"] != "telemetry":
        later = read_packet(ser)

    if later is None:
        print("FAIL: no telemetry to confirm counting resumed")
        return False
    print(f"  later:  instructions={later['instructions']}")

    if later["instructions"] <= after["instructions"]:
        print("FAIL: instruction count did not resume increasing")
        return False

    print("PASS")
    return True


def test_capture_snapshot(ser):
    print("--- Test C: CAPTURE_SNAPSHOT ACK ---")
    resp = send_command_wait_response(ser, CMD_CAPTURE_SNAPSHOT)

    if resp is None:
        print("FAIL: no response received")
        return False

    print(f"  response: cmd_echo=0x{resp['cmd_echo']:02x} status={RESP_STATUS_NAMES.get(resp['status'], resp['status'])} "
          f"data={resp['data']} checksum_ok={resp['checksum_ok']}")

    if resp["cmd_echo"] != CMD_CAPTURE_SNAPSHOT or resp["status"] != RESP_ACK or not resp["checksum_ok"]:
        print("FAIL: unexpected response fields")
        return False

    if resp["data"] == 0:
        print("FAIL: response DATA (captured instruction count) is 0")
        return False

    print(f"  captured instruction count: {resp['data']}")
    print("PASS")
    return True


def test_bad_checksum(ser):
    print("--- Test D: BAD CHECKSUM ---")
    resp = send_command_wait_response(ser, CMD_PING, checksum=0xFF)  # deliberately wrong

    if resp is None:
        print("FAIL: no response received")
        return False

    print(f"  response: cmd_echo=0x{resp['cmd_echo']:02x} status={RESP_STATUS_NAMES.get(resp['status'], resp['status'])} "
          f"checksum_ok={resp['checksum_ok']}")

    if resp["status"] != RESP_NACK_BAD_CHECKSUM:
        print("FAIL: expected NACK_BAD_CHECKSUM")
        return False

    print("PASS")
    return True


def test_unknown_command(ser):
    print("--- Test E: UNKNOWN COMMAND ---")
    resp = send_command_wait_response(ser, 0xFF)  # valid checksum, undefined command

    if resp is None:
        print("FAIL: no response received")
        return False

    print(f"  response: cmd_echo=0x{resp['cmd_echo']:02x} status={RESP_STATUS_NAMES.get(resp['status'], resp['status'])} "
          f"checksum_ok={resp['checksum_ok']}")

    if resp["status"] != RESP_NACK_UNKNOWN_CMD or resp["cmd_echo"] != 0xFF:
        print("FAIL: expected NACK_UNKNOWN_CMD echoing cmd 0xff")
        return False

    print("PASS")
    return True


def test_telemetry_coexistence(ser, seconds=3):
    print(f"--- Test F: TELEMETRY COEXISTENCE ({seconds}s, sending PINGs throughout) ---")

    end_time = time.time() + seconds
    telemetry_seen = 0
    response_seen = 0

    while time.time() < end_time:
        ser.write(build_frame(CMD_PING))
        pkt = read_packet(ser)

        while pkt is not None and time.time() < end_time:
            if pkt["type"] == "telemetry":
                telemetry_seen += 1
                print(f"  TELEMETRY  flags=0x{pkt['flags']:02x} instr={pkt['instructions']}")
            else:
                response_seen += 1
                print(f"  RESPONSE   cmd_echo=0x{pkt['cmd_echo']:02x} "
                      f"status={RESP_STATUS_NAMES.get(pkt['status'], pkt['status'])} checksum_ok={pkt['checksum_ok']}")
                break  # got the PING's response, send the next one

            pkt = read_packet(ser)

    print(f"  {telemetry_seen} telemetry packet(s), {response_seen} response packet(s), no decode errors")

    if response_seen == 0:
        print("FAIL: never got a response back during the coexistence window")
        return False

    print("PASS")
    return True


if __name__ == "__main__":
    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        ser.reset_input_buffer()

        results = [
            test_ping(ser),
            test_reset_counters(ser),
            test_capture_snapshot(ser),
            test_bad_checksum(ser),
            test_unknown_command(ser),
            test_telemetry_coexistence(ser),
        ]

    if all(results):
        print("\nALL TESTS PASS")
        sys.exit(0)

    print("\nSOME TESTS FAILED")
    sys.exit(1)
