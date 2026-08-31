BUILD=build
FIRMWARE=firmware
RTL=rtl
CONSTRAINTS=constraints

RISCV_GCC=riscv64-elf-gcc
RISCV_OBJCOPY=riscv64-elf-objcopy

.PHONY: all firmware synth pnr bitstream program clean

all: bitstream

firmware:
	$(RISCV_GCC) \
		-march=rv32i \
		-mabi=ilp32 \
		-ffreestanding \
		-nostdlib \
		-Os \
		-T $(FIRMWARE)/link.ld \
		$(FIRMWARE)/start.S \
		$(FIRMWARE)/main.c \
		-o $(FIRMWARE)/firmware.elf
	$(RISCV_OBJCOPY) -O binary \
		$(FIRMWARE)/firmware.elf \
		$(FIRMWARE)/firmware.bin
	python3 $(FIRMWARE)/bin2hex.py \
		$(FIRMWARE)/firmware.bin \
		> $(FIRMWARE)/firmware.hex

synth: firmware
	mkdir -p $(BUILD)
	yosys -p "synth_ice40 -top top -json $(BUILD)/soc.json" \
    	$(RTL)/core/picorv32.v \
    	$(RTL)/uart_tx.v \
    	$(RTL)/top.v

pnr: synth
	nextpnr-ice40 \
		--hx8k \
		--package cb132 \
		--json $(BUILD)/soc.json \
		--pcf $(CONSTRAINTS)/alchitry_cu.pcf \
		--asc $(BUILD)/soc.asc \
		--freq 100

bitstream: pnr
	icepack $(BUILD)/soc.asc $(BUILD)/soc.bin

program: bitstream
	iceprog $(BUILD)/soc.bin

clean:
	rm -rf $(BUILD)
	rm -f $(FIRMWARE)/firmware.elf
	rm -f $(FIRMWARE)/firmware.bin
	rm -f $(FIRMWARE)/firmware.hex
