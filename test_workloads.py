import serial
import struct
import sys
import time

PORT = "/dev/cu.usbserial-FT4MG9OV1"
BAUD = 115200

CMD_MAGIC = b"\xA5\x5A"
PROTO_VERSION = 0x01
CMD_PING = 0x01
CMD_RESET_COUNTERS = 0x02
CMD_SET_WORKLOAD = 0x04

TELEMETRY_PAYLOAD_SIZE = 27  # M4: version 0x02, +DATA_RAM_COUNT +WORKLOAD_ID
RESPONSE_PAYLOAD_SIZE = 8

WORKLOADS = [(0, "IDLE"), (1, "ALU"), (2, "MEMORY"), (3, "BRANCH"), (4, "MMIO"), (5, "MIXED")]

COLLECT_SECONDS = 2.5  # telemetry is ~1/s, this comfortably spans 2 samples


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

        cycles, instructions, memory, mmio, data_ram = struct.unpack("<QIIII", payload[2:26])
        return {"type": "telemetry", "cycles": cycles, "instructions": instructions,
                "memory": memory, "mmio": mmio, "data_ram": data_ram, "workload": payload[26]}

    if b1 == b"\x5B":
        payload = ser.read(RESPONSE_PAYLOAD_SIZE)
        if len(payload) != RESPONSE_PAYLOAD_SIZE:
            return None

        return {"type": "response", "cmd_echo": payload[1], "status": payload[2]}

    return None


def wait_for_response(ser, cmd, arg=0, max_packets=10):
    ser.write(build_frame(cmd, arg))

    for _ in range(max_packets):
        pkt = read_packet(ser)
        if pkt is not None and pkt["type"] == "response":
            return pkt

    return None


def collect_telemetry(ser, seconds):
    first = None
    last = None
    end_time = time.time() + seconds

    while time.time() < end_time:
        pkt = read_packet(ser)
        if pkt is None or pkt["type"] != "telemetry":
            continue

        if first is None:
            first = pkt
        last = pkt

    return first, last


def rate(first, last, field, elapsed_s):
    if first is None or last is None or elapsed_s <= 0:
        return 0.0
    return (last[field] - first[field]) / elapsed_s


if __name__ == "__main__":
    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        ser.reset_input_buffer()

        print("--- confirm PING ---")
        resp = wait_for_response(ser, CMD_PING)
        if resp is None or resp["status"] != 0x00:
            print("FAIL: PING did not ACK, aborting")
            sys.exit(1)
        print("PING ACK OK")

        print("--- reset counters ---")
        resp = wait_for_response(ser, CMD_RESET_COUNTERS)
        if resp is None or resp["status"] != 0x00:
            print("FAIL: RESET_COUNTERS did not ACK, aborting")
            sys.exit(1)
        print("RESET_COUNTERS ACK OK")

        results = {}

        for wl_id, wl_name in WORKLOADS:
            print(f"--- SET_WORKLOAD {wl_name} ---")
            resp = wait_for_response(ser, CMD_SET_WORKLOAD, wl_id)

            if resp is None or resp["status"] != 0x00:
                print(f"FAIL: SET_WORKLOAD {wl_name} did not ACK")
                sys.exit(1)

            time.sleep(0.3)  # let a stale in-flight previous-workload call finish

            first, last = collect_telemetry(ser, COLLECT_SECONDS)
            if first is None or last is None:
                print(f"FAIL: no telemetry collected for {wl_name}")
                sys.exit(1)

            # cycle-counter delta, not wall-clock time, so the rate reflects
            # actual hardware execution rather than host/OS scheduling jitter
            elapsed_cycles = last["cycles"] - first["cycles"]
            elapsed_s = elapsed_cycles / 50_000_000.0 if elapsed_cycles > 0 else 0

            delta_instr = last["instructions"] - first["instructions"]

            results[wl_name] = {
                "instr_rate": rate(first, last, "instructions", elapsed_s),
                "dram_rate": rate(first, last, "data_ram", elapsed_s),
                "mmio_rate": rate(first, last, "mmio", elapsed_s),
                "cpi": elapsed_cycles / delta_instr if delta_instr > 0 else 0,
            }

            r = results[wl_name]
            print(f"  INSTR/s={r['instr_rate']:.0f}  DATA_RAM/s={r['dram_rate']:.0f}  "
                  f"MMIO/s={r['mmio_rate']:.0f}  CPI={r['cpi']:.2f}")

        print()
        print(f"{'WORKLOAD':<10}{'INSTR/s':>12}{'DATA_RAM/s':>14}{'MMIO/s':>12}{'CPI':>8}")
        for wl_id, wl_name in WORKLOADS:
            r = results[wl_name]
            print(f"{wl_name:<10}{r['instr_rate']:>12.0f}{r['dram_rate']:>14.0f}"
                  f"{r['mmio_rate']:>12.0f}{r['cpi']:>8.2f}")

        # sanity checks on workload intent, not hardcoded thresholds --
        # these numbers haven't been measured on real hardware yet
        print()
        warnings = []

        if results["MEMORY"]["dram_rate"] <= results["ALU"]["dram_rate"]:
            warnings.append("MEMORY DATA_RAM/s <= ALU DATA_RAM/s")

        if results["MMIO"]["mmio_rate"] <= results["ALU"]["mmio_rate"]:
            warnings.append("MMIO MMIO/s <= ALU MMIO/s")

        if results["MMIO"]["mmio_rate"] <= results["MEMORY"]["mmio_rate"]:
            warnings.append("MMIO MMIO/s <= MEMORY MMIO/s")

        if warnings:
            print("WARNINGS (measurements inconsistent with workload intent):")
            for w in warnings:
                print(f"  - {w}")
            sys.exit(1)

        print("Workload signatures are consistent with intent.")
        sys.exit(0)
