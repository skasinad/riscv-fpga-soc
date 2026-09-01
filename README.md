# RISC V FPGA Experimentation Platform
[Link for video demonstration](https://drive.google.com/file/d/1yuzfY7mUa41g7OHrjCE1uGmEuxYOjFrQ/view)
This project is a small RISC V computer built on an FPGA, but the goal was not just to get a processor running.

I wanted to build something where I could actually see what the processor was doing, control how it was running, and even intentionally make it fail in a controlled way.

The system uses a PicoRV32 processor running the RV32I instruction set on an Alchitry Cu FPGA board. I added my own MMIO system, hardware counters, snapshot logic, UART communication, workload control, fault injection, and a live browser dashboard.

From the dashboard, I can watch the processor running in real time. I can switch between different workloads, compare memory and MMIO activity, reset counters, capture snapshots, intentionally inject an illegal instruction into the CPU, watch the real processor enter a trap, and then reset the system and watch it boot back up and recover.

What started as getting a RISC V processor running on an FPGA eventually turned into a small hands on computer architecture and debugging lab.

# What the project can do

The dashboard can show

- processor cycle count
- instruction count
- total RAM activity
- actual data RAM activity
- MMIO activity
- instructions per second
- RAM accesses per second
- data RAM accesses per second
- MMIO accesses per second
- CPI
- POST status
- trap status
- active workload
- live activity graphs
- command results
- system events

The user can also control the FPGA from the browser.

The dashboard can

- switch between six CPU workloads
- reset hardware performance counters
- capture a hardware snapshot
- inject an illegal instruction
- make the real PicoRV32 processor trap
- reset the full SoC
- watch POST run again
- recover back into normal operation

LED7 on the physical FPGA board is directly connected to the PicoRV32 trap signal.

When the processor traps, LED7 turns on.

When the system is reset and the processor recovers, LED7 turns back off.

# Hardware

The current system uses

- Alchitry Cu FPGA board
- Lattice iCE40 HX8K FPGA
- CB132 package
- 100 MHz board clock
- 50 MHz SoC clock
- PicoRV32 processor
- RV32I instruction set
- 4 KB FPGA RAM
- onboard FTDI USB connection

The FPGA talks directly to the Mac through USB.

No Raspberry Pi or external UART adapter is needed.

# Tools

The project was developed using

- riscv64 elf gcc
- riscv64 elf objcopy
- Yosys
- nextpnr ice40
- icepack
- iceprog
- Icarus Verilog
- Python
- pyserial
- Flask
- HTML
- CSS
- JavaScript
- Chart.js

# System overview

There are three main parts of the project.

## The RISC V SoC

The FPGA contains the PicoRV32 processor, 4 KB of RAM, MMIO registers, performance counters, snapshot logic, UART communication, workload control, fault injection, and reset logic.

The CPU runs bare metal firmware directly from FPGA memory.

## Hardware monitoring

The FPGA has separate RTL logic that watches what the processor is doing.

It tracks things such as

```text
cycles
instructions
RAM accesses
data RAM accesses
MMIO accesses
trap state
active workload
```

The important part is that these measurements are not being made by the firmware itself.

The FPGA is watching the processor activity directly.

## Host dashboard

The telemetry path is

```text
PicoRV32
    |
FPGA counters
    |
UART
    |
FTDI USB
    |
Python
    |
Flask
    |
browser dashboard
```

Commands go in the opposite direction.

```text
browser
    |
Flask
    |
Python
    |
USB UART
    |
FPGA command parser
    |
RISC V SoC
```

This gives the system two way communication.

# Power On Self Test

Whenever the system boots, the firmware runs a POST.

The POST checks several important parts of the SoC.

It checks

- GPIO and MMIO
- system status
- cycle counter
- debug register reads and writes
- execution counters
- snapshot registers

A successful POST ends with

```text
GPIO_OUT = 0x3F
```

On the physical board this gives

```text
LED0 to LED5 ON
LED6 OFF
LED7 OFF
```

LED7 is separate from the normal firmware GPIO output.

It is directly connected to the PicoRV32 trap signal.

# MMIO map

The current MMIO register map is

```text
0x10000000 GPIO_OUT
0x10000004 SYS_STATUS
0x10000008 CYCLE_COUNT_LO
0x1000000C CYCLE_COUNT_HI
0x10000010 DEBUG_CTRL
0x10000014 INSTR_COUNT
0x10000018 MEM_COUNT
0x1000001C MMIO_COUNT
0x10000020 SNAP_CTRL
0x10000024 SNAP_CYCLE_LO
0x10000028 SNAP_CYCLE_HI
0x1000002C SNAP_INSTR
0x10000030 SNAP_MEM
0x10000034 SNAP_MMIO
0x10000038 WORKLOAD_SELECT
0x1000003C DATA_RAM_COUNT
0x10000040 SNAP_DATA_RAM
0x10000044 WORKLOAD_SCRATCH
```

As the project grew, I kept all of the older addresses in the same place and added new registers after them.

# RAM activity

There are two different RAM counters.

```text
MEM_COUNT
```

counts all RAM accesses.

That includes instruction fetches and normal data accesses.

```text
DATA_RAM_COUNT
```

only counts real data RAM accesses.

Conceptually the hardware checks something like

```verilog
mem_valid && mem_ready && ram_ready && !mem_instr
```

This makes it much easier to compare different workloads.

For example, an ALU workload may execute millions of instructions while barely touching data RAM.

A memory workload causes the data RAM counter to increase very quickly.

# Workload engine

The firmware has six different workloads.

```text
IDLE
ALU
MEMORY
BRANCH
MMIO
MIXED
```

The host can change the active workload while the processor is running.

The system does not need to reboot.

## IDLE

IDLE gives a basic low activity baseline.

The CPU is still running instructions, but it does very little data RAM or MMIO work.

## ALU

ALU runs mostly integer operations using CPU registers.

It creates high instruction activity with very little data RAM or MMIO activity.

## MEMORY

MEMORY repeatedly performs loads and stores on a small buffer inside FPGA RAM.

This causes DATA_RAM_COUNT to increase very quickly.

## BRANCH

BRANCH runs deterministic branch heavy control flow.

It keeps data RAM and MMIO traffic low while making the CPU execute a large amount of conditional control flow.

## MMIO

MMIO repeatedly writes to a safe scratch MMIO register.

This causes MMIO_COUNT to increase very quickly.

## MIXED

MIXED combines arithmetic, RAM accesses, branches, and MMIO.

This gives a workload that has activity across several different parts of the SoC.

# Real FPGA results

The workloads were tested on the physical Alchitry Cu board.

One test produced these results.

```text
WORKLOAD       INSTR/s      DATA_RAM/s      MMIO/s     CPI

IDLE          13332934             167          167    3.75
ALU           12193071            1219          609    4.10
MEMORY        10263027         2550663           99    4.87
BRANCH        13279218             781          391    3.77
MMIO          11108645            1387      2775775    4.50
MIXED         11607491          979321       124854    4.31
```

The differences are very clear on real hardware.

For example

```text
ALU DATA_RAM/s
about 1,200

MEMORY DATA_RAM/s
about 2,550,000
```

The MMIO workload shows a similar difference.

```text
ALU MMIO/s
about 600

MMIO workload MMIO/s
about 2,775,000
```

These numbers come from the actual hardware counters inside the FPGA.

The dashboard is not generating fake activity.

# UART communication

The FPGA communicates with the host using

```text
115200 baud
8N1
```

The UART RX signal comes from outside the FPGA clock domain.

Because of that, the input first passes through a two flop synchronizer before the receiver uses it.

The host and FPGA use binary packets instead of text commands.

# Host command packet

A host command is 9 bytes.

```text
MAGIC0
MAGIC1
VERSION
CMD
ARG0
ARG1
ARG2
ARG3
CHECKSUM
```

The current commands are

```text
0x01 PING
0x02 RESET_COUNTERS
0x03 CAPTURE_SNAPSHOT
0x04 SET_WORKLOAD
0x05 INJECT_FAULT
0x06 SYSTEM_RESET
```

The 32 bit argument is sent little endian.

# Response packets

The FPGA sends an ACK or NACK response for commands.

A response packet contains

```text
MAGIC0
MAGIC1
VERSION
CMD_ECHO
STATUS
DATA0
DATA1
DATA2
DATA3
CHECKSUM
```

The response status can show things such as

```text
ACK
NACK_BAD_CHECKSUM
NACK_BAD_VERSION
NACK_UNKNOWN_CMD
NACK_BAD_ARGUMENT
```

Responses and telemetry use the same UART transmitter.

The FPGA makes sure one packet finishes before another one starts.

This prevents response bytes and telemetry bytes from mixing together.

# Telemetry

The FPGA sends telemetry about once every second.

The current telemetry packet uses version 0x02.

It includes

- flags
- 64 bit cycle count
- instruction count
- total RAM count
- MMIO count
- data RAM count
- active workload

The Python host uses these values to calculate

- instructions per second
- total RAM accesses per second
- data RAM accesses per second
- MMIO accesses per second
- CPI

# Hardware snapshots

The SoC can capture a coherent snapshot of its hardware counters.

A snapshot includes

```text
cycle count
instruction count
RAM count
data RAM count
MMIO count
```

The snapshot registers all capture at the same logical moment.

After that, the live counters continue running while the snapshot values stay frozen.

Snapshots can be triggered from firmware through MMIO or from the host through UART.

The current command response returns the captured instruction count to the host.

# Fault injection

One of the main parts of the project is deterministic fault injection.

The user can press

```text
INJECT ILLEGAL INSTRUCTION
```

from the dashboard.

The FPGA does not fake a trap signal.

Instead, the hardware arms a one shot fault.

On the next valid instruction fetch, the instruction returned to PicoRV32 is replaced with

```text
0x00000000
```

for one fetch only.

The actual RAM contents are never changed.

For the current RV32I PicoRV32 configuration, this instruction encoding is illegal.

The flow looks like this.

```text
host sends fault command
        |
FPGA sends ACK
        |
fault becomes armed
        |
next instruction fetch is replaced
        |
PicoRV32 sees illegal instruction
        |
real CPU trap goes high
        |
LED7 turns ON
        |
dashboard shows TRAP ASSERTED
```

The important part is that this is the real PicoRV32 trap signal.

The dashboard is only displaying what happened in hardware.

# System reset and recovery

Once PicoRV32 enters trap, the system uses a full reset to recover.

The host sends

```text
SYSTEM RESET
```

The FPGA first sends the ACK back to the host.

Only after the final ACK byte has completely finished transmitting does the reset begin.

The internal reset lasts 128 SoC clock cycles.

After reset

```text
CPU starts again
        |
firmware restarts
        |
POST runs again
        |
POST passes
        |
workload returns to IDLE
        |
telemetry resumes
```

LED7 goes back off because the real processor is no longer trapped.

A normal recovered board ends with

```text
LED0 to LED5 ON
LED6 OFF
LED7 OFF
```

This allows the full experiment

```text
RUN
 |
FAULT
 |
TRAP
 |
RESET
 |
POST
 |
RECOVER
```

# Dashboard

The dashboard is built using

- Python
- Flask
- pyserial
- HTML
- CSS
- JavaScript
- Chart.js

The interface was intentionally made to look more like engineering instrumentation software than a normal web application.

The dashboard shows

- connection state
- CPU information
- POST state
- trap state
- counters
- live rates
- CPI
- activity graph
- active workload
- workload controls
- reset counters control
- snapshot control
- fault injection control
- system reset control
- event log

The active workload shown in the browser comes from FPGA telemetry.

The browser does not assume that the hardware changed just because a button was clicked.

# Event log

The dashboard keeps a chronological event log.

A real hardware test looked like this.

```text
16:13:53.893  CONNECTED
16:13:54.306  POST PASS
16:13:54.306  WORKLOAD ACTIVE IDLE
16:14:09.138  SET_WORKLOAD MEMORY ACK
16:14:09.346  WORKLOAD ACTIVE MEMORY
16:14:10.690  CAPTURE_SNAPSHOT ACK  instr=2414882747
16:14:20.674  INJECT_FAULT ACK
16:14:21.378  TRAP ASSERTED
16:14:27.746  SYSTEM_RESET ACK
16:14:28.754  TRAP CLEAR
16:14:28.754  RECOVERY COMPLETE
16:14:28.754  WORKLOAD ACTIVE IDLE
```

The log only records important events.

It does not log every telemetry packet.

Hardware events such as TRAP ASSERTED come from actual FPGA telemetry.

Command ACK events are only recorded after the FPGA actually responds.

The event log currently keeps the latest 150 events in memory.

# Building the firmware

Compile the firmware with

```bash
riscv64-elf-gcc \
-march=rv32i \
-mabi=ilp32 \
-ffreestanding \
-nostdlib \
-Os \
-T firmware/link.ld \
firmware/start.S firmware/main.c \
-o firmware/firmware.elf
```

Convert the ELF into a binary file.

```bash
riscv64-elf-objcopy -O binary \
firmware/firmware.elf \
firmware/firmware.bin
```

Convert the binary into the HEX file used by FPGA memory.

```bash
python3 firmware/bin2hex.py \
firmware/firmware.bin \
> firmware/firmware.hex
```

# Building the FPGA design

Run Yosys.

```bash
yosys -p "synth_ice40 -top top -json build/soc.json" \
rtl/core/picorv32.v \
rtl/top.v
```

Run nextpnr.

```bash
nextpnr-ice40 \
--hx8k \
--package cb132 \
--json build/soc.json \
--pcf constraints/alchitry_cu.pcf \
--asc build/soc.asc \
--freq 50
```

The 50 MHz timing result must pass before programming the FPGA.

A generated `.asc` file by itself does not mean timing passed.

Pack the bitstream.

```bash
icepack build/soc.asc build/soc.bin
```

Program the board.

```bash
iceprog build/soc.bin
```

The repository also has a Makefile for the normal build flow.

A typical build is

```bash
make clean
make
make program
```

# Starting the dashboard

Create or activate the Python environment and run

```bash
source .venv/bin/activate
python3 dashboard/app.py
```

Then open

```text
http://127.0.0.1:5000
```

The FPGA normally appears on macOS as

```text
/dev/cu.usbserial-FT4MG9OV1
```

This may be different on another system.

Make sure another serial program is not already using the port.

You can check on macOS with

```bash
lsof /dev/cu.usbserial-FT4MG9OV1
```

# Running the workload test

The physical workload test can be run with

```bash
python3 test_workloads.py
```

It switches through the six workloads and prints measured results for

```text
INSTR/s
DATA_RAM/s
MMIO/s
CPI
```

# Running the fault recovery test

The fault and recovery test can be run with

```bash
python3 test_fault_recovery.py
```

The expected physical sequence is

```text
healthy system
        |
fault command
        |
illegal instruction injected
        |
PicoRV32 trap
        |
LED7 ON
        |
system reset
        |
POST runs again
        |
trap clears
        |
LED7 OFF
```

# Simulation

Icarus Verilog is used for RTL simulation.

The project includes tests for things such as

- UART RX
- malformed packets
- parser timeout and recovery
- command ACK and NACK responses
- workload selection
- performance counters
- data RAM counting
- snapshot behavior
- telemetry and response arbitration
- illegal instruction injection
- real PicoRV32 trap behavior
- full system reset
- POST after reset
- post recovery commands

The main integration tests are in the `sim` directory.

# One of the most important bugs in the project

One of the biggest debugging problems happened before most of the monitoring system was built.

The design worked correctly in RTL simulation but failed on the physical FPGA.

The original SoC was being run directly from the 100 MHz board clock.

nextpnr showed that the design could only close timing at around 74 MHz.

The important detail was that nextpnr still created the output `.asc` file even though timing had failed.

That meant it was possible to pack and program a design that had not actually met timing.

On real hardware the CPU began corrupting instruction execution and entering trap.

The RTL itself was not the problem.

The design was simply running too fast for the physical FPGA implementation.

The SoC was first stabilized at 25 MHz and later moved to the current 50 MHz clock.

This was one of the biggest lessons from the project.

```text
simulation correctness
does not automatically mean
physical timing correctness
```

# Another hardware bug

A later bug happened while adding the execution counters.

An old signal named `ram_select` had been removed earlier in the project but was accidentally referenced again.

Because Verilog allows implicit nets by default, the code still compiled.

The undeclared signal became an undriven wire.

The result was that `MEM_COUNT` stayed at zero on the real hardware and POST returned fault code `0x49`.

The counter was fixed by using the real completed RAM handshake.

```verilog
if(mem_valid && mem_ready && ram_ready)
    mem_access_count <= mem_access_count + 1;
```

This bug is one reason the project now treats implicit nets as a serious failure.

# Project goal

The goal of this project is not to build the fastest RISC V processor.

It is also not meant to replace a commercial FPGA development or debugging platform.

The goal is to build a small system where another student can actually experiment with a real processor.

Someone can use it to see things such as

- how different workloads change processor behavior
- how instruction fetches affect total memory traffic
- how data memory activity differs from total RAM activity
- how MMIO traffic appears
- how CPI changes
- how hardware counters can observe a CPU
- how UART can be used as a control and telemetry channel
- how an illegal instruction causes a real processor trap
- how a system can detect a fault and recover through reset
- why FPGA timing closure matters even when simulation passes

The final result is a small hands on RISC V architecture and fault observability lab running on real FPGA hardware.
