import serial
import struct
import sys
import time

PORT = "/dev/cu.usbserial-FT4MG9OV1"
BAUD = 115200

CMD_MAGIC = b"\xA5\x5A"
PROTO_VERSION = 0x01
CMD_PING = 0x01
CMD_SET_WORKLOAD = 0x04
CMD_INJECT_FAULT = 0x05
CMD_SYSTEM_RESET = 0x06

TELEMETRY_PAYLOAD_SIZE = 27
RESPONSE_PAYLOAD_SIZE = 8

WORKLOADS = {"IDLE": 0, "ALU": 1, "MEMORY": 2, "BRANCH": 3, "MMIO": 4, "MIXED": 5}

TRAP_OBSERVE_TIMEOUT = 5.0  #ack is near instant, trap follows within microseconds
                            #of the fault being armed, telemetry just needs to catch up
RECOVERY_TIMEOUT = 5.0  #post's own delay is ~100ms so 5s is plenty of margin
LED_OBSERVE_PAUSE = 5.0  #time to physically look at led7 before recovering


def build_frame(cmd, arg=0):
    arg_bytes = struct.pack("<I", arg)
    checksum = PROTO_VERSION ^ cmd
    for b in arg_bytes:
        checksum ^= b
    return CMD_MAGIC + bytes([PROTO_VERSION, cmd]) + arg_bytes + bytes([checksum])


def read_packet(ser):
    b0 = ser.read(1)
    if b0 != b"\xA5":
        return None

    b1 = ser.read(1)

    if b1 == b"\x5A":
        payload = ser.read(TELEMETRY_PAYLOAD_SIZE)
        if len(payload) != TELEMETRY_PAYLOAD_SIZE:
            return None

        flags = payload[1]
        cycles, instructions, memory, mmio, data_ram = struct.unpack("<QIIII", payload[2:26])

        return {
            "type": "telemetry",
            "post_pass": bool(flags & 0x01),
            "trap": bool(flags & 0x02),
            "cycles": cycles,
            "instructions": instructions,
            "memory": memory,
            "mmio": mmio,
            "data_ram": data_ram,
            "workload": payload[26],
        }

    if b1 == b"\x5B":
        payload = ser.read(RESPONSE_PAYLOAD_SIZE)
        if len(payload) != RESPONSE_PAYLOAD_SIZE:
            return None

        return {"type": "response", "cmd_echo": payload[1], "status": payload[2]}

    return None


def wait_for_response(ser, cmd, arg=0, max_packets=20):
    ser.write(build_frame(cmd, arg))

    for _ in range(max_packets):
        pkt = read_packet(ser)
        if pkt is not None and pkt["type"] == "response":
            return pkt

    return None


def wait_for_telemetry(ser, predicate, timeout):
    end_time = time.time() + timeout

    while time.time() < end_time:
        pkt = read_packet(ser)
        if pkt is not None and pkt["type"] == "telemetry" and predicate(pkt):
            return pkt

    return None


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}")
        sys.exit(1)


if __name__ == "__main__":
    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        ser.reset_input_buffer()

        print("--- Test A: baseline ---")
        resp = wait_for_response(ser, CMD_PING)
        require(resp is not None and resp["status"] == 0x00, "PING did not ACK")

        telem = wait_for_telemetry(ser, lambda p: p["post_pass"], 3.0)
        require(telem is not None, "telemetry never reported POST PASS at baseline")
        require(not telem["trap"], "trap already asserted at baseline")
        print("PING ACK OK")
        print("POST PASS")
        print("TRAP CLEAR")

        print()
        print("--- Test B: select workload (ALU) ---")
        resp = wait_for_response(ser, CMD_SET_WORKLOAD, WORKLOADS["ALU"])
        require(resp is not None and resp["status"] == 0x00, "SET_WORKLOAD ALU did not ACK")

        telem = wait_for_telemetry(ser, lambda p: p["workload"] == WORKLOADS["ALU"], 3.0)
        require(telem is not None, "telemetry never reported ALU active")
        print("SET_WORKLOAD ALU ACK")
        print("telemetry confirms ALU active")

        print()
        print("--- Test C: inject fault ---")
        resp = wait_for_response(ser, CMD_INJECT_FAULT)
        require(resp is not None and resp["status"] == 0x00, "INJECT_FAULT did not ACK")
        print("FAULT ACK")

        #ack just means armed, not trapped - trap state only ever comes
        #from telemetry, checked separately here
        telem = wait_for_telemetry(ser, lambda p: p["trap"], TRAP_OBSERVE_TIMEOUT)
        require(telem is not None, f"telemetry never reported trap=1 within {TRAP_OBSERVE_TIMEOUT}s")
        print("TRAP ASSERTED")

        print()
        print(">>> Physically verify LED7 is ON now.")
        print(f">>> Pausing {LED_OBSERVE_PAUSE:.0f}s for manual observation before recovery...")
        time.sleep(LED_OBSERVE_PAUSE)

        print()
        print("--- Test D: system reset ---")
        resp = wait_for_response(ser, CMD_SYSTEM_RESET)
        require(resp is not None and resp["status"] == 0x00, "SYSTEM_RESET did not ACK")
        print("SYSTEM RESET ACK")

        telem = wait_for_telemetry(ser, lambda p: p["post_pass"] and not p["trap"], RECOVERY_TIMEOUT)
        require(telem is not None, f"POST/trap recovery not observed within {RECOVERY_TIMEOUT}s")
        print("POST PASS")
        print("TRAP CLEAR")
        print("RECOVERY PASS")

        print()
        print(">>> Physically verify LED7 is OFF, LED0-5 ON, LED6 OFF now.")

        print()
        print("--- Test E: post-recovery functionality ---")
        resp = wait_for_response(ser, CMD_PING)
        require(resp is not None and resp["status"] == 0x00, "post-recovery PING did not ACK")

        resp = wait_for_response(ser, CMD_SET_WORKLOAD, WORKLOADS["MEMORY"])
        require(resp is not None and resp["status"] == 0x00, "post-recovery SET_WORKLOAD MEMORY did not ACK")

        telem = wait_for_telemetry(ser, lambda p: p["workload"] == WORKLOADS["MEMORY"], 3.0)
        require(telem is not None, "telemetry never reported MEMORY active after recovery")

        print("PING ACK OK")
        print("SET_WORKLOAD MEMORY ACK, telemetry confirms MEMORY active")

        print()
        print(">>> FAULT/RECOVERY TEST PASS")
