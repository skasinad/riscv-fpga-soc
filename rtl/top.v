module top(
    input wire clk,
    output wire [7:0] led,
    output wire usb_tx,
    input wire usb_rx
);

// Clock divider
//
// The Alchitry Cu provides a 100 MHz input clock. The current PicoRV32 SoC
// does not meet timing at 100 MHz -- place-and-route shows an achievable
// frequency of roughly 74 MHz. The system runs from a divided 50 MHz clock,
// which gives comfortable margin while still being faster than the initial
// 25 MHz bring-up frequency.

reg [1:0] clkdiv=0;

always @(posedge clk)
    clkdiv<=clkdiv+1;

wire sys_clk=clkdiv[0];


// Reset generation
//
// resetn combines two sources, each with exactly one owner, so it stays a
// plain derived wire rather than something written from more than one
// place. poweron_counter is the original M1 cold-boot reset, unchanged.
// reset_pulse_active is a host-requested pulse armed by reset_pulse_trigger
// (see the TX arbiter block below -- awaiting_reset_flush/reset_pulse_trigger),
// which only fires once a CMD_SYSTEM_RESET ACK has fully left the UART
// transmitter. Combining them as poweron_done && !reset_pulse_active means
// either source alone is enough to hold the SoC in reset.

reg [7:0] poweron_counter=0;
wire poweron_done=poweron_counter[7];

always @(posedge sys_clk)
begin
    if(!poweron_done)
        poweron_counter<=poweron_counter+1;
end

// 128 cycles (2.56us @ 50 MHz) matches the power-on reset's own
// already-verified duration. Every register in this design uses a plain
// synchronous "if(!resetn) reg<=0" pattern (no multi-cycle reset chains),
// so even 1 cycle would technically clear them all, but reusing the
// cold-boot duration avoids inventing a second, separately-justified
// constant for what is architecturally the same kind of reset.
localparam RESET_PULSE_CYCLES = 128;

reg [7:0] reset_pulse_counter=0;
reg reset_pulse_active=0;

always @(posedge sys_clk)
begin
    if(reset_pulse_trigger)
    begin
        reset_pulse_active<=1;
        reset_pulse_counter<=0;
    end
    else if(reset_pulse_active)
    begin
        if(reset_pulse_counter==RESET_PULSE_CYCLES-1)
            reset_pulse_active<=0;
        else
            reset_pulse_counter<=reset_pulse_counter+1;
    end
end

wire resetn=poweron_done && !reset_pulse_active;



// PicoRV32 memory interface
wire trap;

wire mem_valid;
wire mem_instr;
wire mem_ready;

wire [31:0] mem_addr;
wire [31:0] mem_wdata;
wire [3:0] mem_wstrb;
wire [31:0] mem_rdata;



// 4 KB RAM

localparam RAM_BYTES = 32'h00001000;

reg [31:0] memory [0:1023];

reg ram_ready=0;
reg [31:0] ram_rdata=0;

// word index into memory[]; mem_addr is byte-addressed, memory[] is 32-bit wide
wire [9:0] ram_word_addr = mem_addr[11:2];

// M6: one-shot illegal-instruction fault injection.
//
// 0x00000000 is guaranteed illegal for this core's configuration: opcode
// bits[6:0]=0000000 doesn't match any RV32I major opcode (LUI=0110111,
// AUIPC=0010111, JAL=1101111, JALR=1100111, LOAD=0000011, OP-IMM=0010011,
// OP=0110011, BRANCH=1100011, STORE=0100011, FENCE=0001111, SYSTEM=1110011
// -- all nonzero), so every instr_* classification flag in picorv32.v's
// decoder stays 0 and instr_trap's all-zero check fires unconditionally
// (confirmed by reading rtl/core/picorv32.v directly, not assumed).
// COMPRESSED_ISA=0 on this core, so the 16-bit C-extension decode path
// never even applies. This is also the RISC-V spec's own reserved
// "guaranteed illegal" encoding, not a value specific to this decoder.
localparam ILLEGAL_INSTRUCTION = 32'h00000000;

// fault_pending is armed by CMD_INJECT_FAULT and lives in the RX parser
// block below (same split-ownership shape as resp_pending/resp_consumed:
// one block arms it, the block that actually consumes it clears it).
// inject_this_fetch is latched from the exact same request-qualifying
// condition ram_ready/ram_rdata already use below, so it lands in
// lockstep one cycle later when the fetch actually completes -- only a
// real instruction fetch (mem_instr) hitting this synchronous RAM path
// can ever set it. Data loads/stores never do (mem_instr=0 at request
// time), and MMIO reads never reach this block at all.
reg fault_pending=0;
reg inject_this_fetch=0;

initial begin
    $readmemh("firmware/firmware.hex",memory);
end



// MMIO register bank
localparam GPIO_OUT_ADDR     = 32'h10000000;
localparam SYS_STATUS_ADDR   = 32'h10000004;
localparam CYCLE_COUNT_LO    = 32'h10000008;
localparam CYCLE_COUNT_HI    = 32'h1000000C;
localparam DEBUG_CTRL_ADDR   = 32'h10000010;
localparam INSTR_COUNT_ADDR   = 32'h10000014;
localparam MEM_COUNT_ADDR     = 32'h10000018;
localparam MMIO_COUNT_ADDR    = 32'h1000001C;

localparam SNAP_CTRL_ADDR     = 32'h10000020;
localparam SNAP_CYCLE_LO_ADDR = 32'h10000024;
localparam SNAP_CYCLE_HI_ADDR = 32'h10000028;
localparam SNAP_INSTR_ADDR    = 32'h1000002C;
localparam SNAP_MEM_ADDR      = 32'h10000030;
localparam SNAP_MMIO_ADDR     = 32'h10000034;

localparam WORKLOAD_SELECT_ADDR  = 32'h10000038;
localparam DATA_RAM_COUNT_ADDR   = 32'h1000003C;
localparam SNAP_DATA_RAM_ADDR    = 32'h10000040;
localparam WORKLOAD_SCRATCH_ADDR = 32'h10000044;

localparam MMIO_BASE = GPIO_OUT_ADDR;
localparam MMIO_TOP  = DEBUG_CTRL_ADDR;

reg [7:0] gpio_out=0;
reg [31:0] debug_ctrl=0;
reg [63:0] cycle_counter=0;

reg [31:0] instr_count=0;
reg [31:0] mem_access_count=0;
reg [31:0] mmio_access_count=0;

// MEM_COUNT already has physically-verified historical meaning (total RAM
// accesses, including fetches) and isn't being touched. This is the subset
// that's actually data traffic -- the thing a MEMORY-vs-ALU workload
// comparison actually needs and MEM_COUNT alone can't show.
reg [31:0] data_ram_count=0;

reg [63:0] snap_cycle_count=0;
reg [31:0] snap_instr_count=0;
reg [31:0] snap_mem_count=0;
reg [31:0] snap_mmio_count=0;
reg [31:0] snap_data_ram_count=0;

// Set only by the RX command parser (CMD_SET_WORKLOAD) -- firmware reads
// it through MMIO but never writes it, so there's exactly one owner. 3
// bits covers workload IDs 0-5 with room to spare.
reg [2:0] workload_select=0;

// Plain R/W scratch word, same shape as debug_ctrl, so the MMIO workload
// has somewhere to hit repeatedly without touching GPIO/SNAP_CTRL/DEBUG_CTRL.
reg [31:0] workload_scratch=0;

reg [7:0] uart_data=0;
reg uart_start=0;
wire uart_busy;
wire uart_tx_line;

reg [5:0] uart_state=0; // widened from [4:0] in M3: response states run 26-35, M4 shifts that to 31-40
reg [25:0] uart_interval=0;

// M6: set the cycle the SYSTEM_RESET response's checksum byte (state 40)
// is handed to uart_tx, cleared the cycle that byte's transmission
// actually finishes. uart_state returns to 0 immediately when state 40
// fires -- before the checksum byte has even started shifting out -- so
// checking uart_state alone would reset the SoC before the ACK reaches
// the host. Watching for uart_busy to go 1-then-0 while this is set is
// what makes the reset genuinely wait for the byte to leave the wire.
reg awaiting_reset_flush=0;

// One-cycle pulse: safe to begin the internal reset pulse now. Read by
// the reset generator above; written only here.
reg reset_pulse_trigger=0;

reg [63:0] uart_cycle_snapshot=0;
reg [31:0] uart_instr_snapshot=0;
reg [31:0] uart_mem_snapshot=0;
reg [31:0] uart_mmio_snapshot=0;
reg [7:0] uart_flags_snapshot=0;
reg [31:0] uart_data_ram_snapshot=0;
reg [7:0] uart_workload_snapshot=0;

// One cycle behind resp_pending on purpose: cmd_capture_snapshot and
// snap_instr_count updating are themselves a cycle apart (pulse fires,
// then the MMIO-ack block's sibling if writes the register the cycle
// after). Without this delay an idle arbiter can commit and latch
// resp_data_value in that same window, one cycle before snap_instr_count
// has actually taken on the fresh value -- reporting stale data.
reg resp_pending_d1=0;

// Response fields latched at the moment the arbiter commits to sending,
// so the byte-send sequence below has a stable snapshot even if the RX
// side classifies a new command mid-transmission.
reg resp_consumed=0;
reg [7:0] resp_tx_cmd_echo=0;
reg [7:0] resp_tx_status=0;
reg [31:0] resp_tx_data=0;
reg [7:0] resp_tx_checksum=0;

// CAPTURE_SNAPSHOT and SET_WORKLOAD are the only commands with a meaningful
// result value; everything else reports 0 in DATA. workload_select is
// read here (not cmd_arg) because by the time the arbiter reaches this
// commit point it's already settled -- same one-cycle margin resp_pending_d1
// already provides for snap_instr_count below, and workload_select updates
// even sooner since it's a direct assignment, not a pulse-triggered one.
wire [31:0] resp_data_value =
    (resp_cmd_echo==CMD_CAPTURE_SNAPSHOT && resp_status==RESP_ACK) ? snap_instr_count :
    (resp_cmd_echo==CMD_SET_WORKLOAD && resp_status==RESP_ACK) ? {29'b0,workload_select} :
    32'b0;

wire [7:0] resp_checksum_value =
    CMD_PROTO_VER ^ resp_cmd_echo ^ resp_status ^
    resp_data_value[7:0] ^ resp_data_value[15:8] ^ resp_data_value[23:16] ^ resp_data_value[31:24];

// Host command receiver. MAGIC(2) VERSION(1) CMD(1) ARG(4, little-endian)
// CHECKSUM(1). Checksum is XOR of VERSION,CMD,ARG[0..3] -- magic bytes
// aren't included, they're the framing signal, not payload.
localparam CMD_MAGIC0    = 8'hA5;
localparam CMD_MAGIC1    = 8'h5A;
localparam CMD_PROTO_VER = 8'h01;
localparam CMD_PING             = 8'h01;
localparam CMD_RESET_COUNTERS   = 8'h02;
localparam CMD_CAPTURE_SNAPSHOT = 8'h03;
localparam CMD_SET_WORKLOAD     = 8'h04;
localparam CMD_INJECT_FAULT     = 8'h05;
localparam CMD_SYSTEM_RESET     = 8'h06;

localparam WORKLOAD_MAX = 32'd5; // IDLE..MIXED, see firmware/main.c

// Response packet: MAGIC0 MAGIC1 VERSION CMD_ECHO STATUS DATA[0..3] CHECKSUM.
// Same checksum convention as the command frame. MAGIC1 is 0x5B, not the
// telemetry stream's 0x5A -- both ride usb_tx, so the host needs to tell
// them apart from the first two bytes without waiting for more.
localparam RESP_MAGIC0 = 8'hA5;
localparam RESP_MAGIC1 = 8'h5B;

localparam RESP_ACK               = 8'h00;
localparam RESP_NACK_BAD_CHECKSUM = 8'h01;
localparam RESP_NACK_BAD_VERSION  = 8'h02;
localparam RESP_NACK_UNKNOWN_CMD  = 8'h03;
localparam RESP_NACK_BAD_ARGUMENT = 8'h04;

localparam RX_IDLE     = 0;
localparam RX_MAGIC1   = 1;
localparam RX_VERSION  = 2;
localparam RX_CMDID    = 3;
localparam RX_ARG0     = 4;
localparam RX_ARG1     = 5;
localparam RX_ARG2     = 6;
localparam RX_ARG3     = 7;
localparam RX_CHECKSUM = 8;

// 20000 cycles = 400us @ 50 MHz, ~4.6 byte times -- generous margin over
// host-side scheduling jitter between bytes of the same frame without
// taking long to resync if a transfer genuinely stalls mid-frame
localparam CMD_TIMEOUT_CYCLES = 20000;

wire [7:0] rx_data;
wire rx_valid;

reg [3:0] cmd_state=0;
reg [7:0] cmd_id=0;
reg [31:0] cmd_arg=0;
reg [7:0] cmd_checksum_calc=0;
reg [14:0] cmd_timeout=0;

reg cmd_ping_seen=0;
reg cmd_reset_counters=0;
reg cmd_capture_snapshot=0;

// SNAP_* only exists on the CPU's MMIO bus, and cmd_snapshot_seen predates
// M3's response packets -- kept as-is since it's already dashboard/M1/M2
// compatible and cheap to leave in place.
reg cmd_snapshot_seen=0;

// One outstanding response at a time: the RX parser latches what to send
// here, the TX arbiter drains it whenever the line is free. If a second
// valid frame classifies before the first response has started sending,
// it overwrites resp_cmd_echo/resp_status and the first response is lost
// (its command side effect already happened and is NOT re-run or lost --
// only the acknowledgment of it is). Host commands are expected to be
// serialized (wait for a response before sending the next one); a deeper
// queue isn't justified for that usage pattern.
reg resp_pending=0;
reg [7:0] resp_cmd_echo=0;
reg [7:0] resp_status=0;

wire mmio_select;

assign mmio_select =
    mem_valid &&
    (mem_addr>=32'h10000000) &&
    (mem_addr<=32'h10000044);


// Synchronous RAM
// This follows the style used by PicoRV32's reference PicoSoC memory implementation.
always @(posedge sys_clk)
begin
    ram_ready<=mem_valid &&
               !ram_ready &&
               (mem_addr<RAM_BYTES);

    inject_this_fetch<=mem_valid &&
                        !ram_ready &&
                        (mem_addr<RAM_BYTES) &&
                        mem_instr &&
                        fault_pending;

    if(mem_valid &&
       !ram_ready &&
       (mem_addr<RAM_BYTES))
    begin
        ram_rdata<=memory[ram_word_addr];

        if(mem_wstrb[0])
            memory[ram_word_addr][7:0]<=mem_wdata[7:0];

        if(mem_wstrb[1])
            memory[ram_word_addr][15:8]<=mem_wdata[15:8];

        if(mem_wstrb[2])
            memory[ram_word_addr][23:16]<=mem_wdata[23:16];

        if(mem_wstrb[3])
            memory[ram_word_addr][31:24]<=mem_wdata[31:24];
    end
end


// Hardware cycle counter
always @(posedge sys_clk)
begin
    if(!resetn)
        cycle_counter<=0;
    else
        cycle_counter<=cycle_counter+1;
end

//Execution monitor
always @(posedge sys_clk)
begin
    if(!resetn)
    begin
        instr_count<=0;
        mem_access_count<=0;
        mmio_access_count<=0;
        data_ram_count<=0;
    end
    else if(cmd_reset_counters)
    begin
        // host reset wins over a same-cycle bus event so each RESET_COUNTERS
        // establishes a clean measurement boundary instead of racing whatever
        // the CPU happened to be doing that cycle. cycle_counter is left
        // alone -- it's system uptime, not an experiment counter, and POST's
        // own delay_cycles() already depends on it being monotonic.
        instr_count<=0;
        mem_access_count<=0;
        mmio_access_count<=0;
        data_ram_count<=0;
    end
    else
    begin
        if(mem_valid&&mem_ready&&mem_instr)
            instr_count<=instr_count+1;

        if(mem_valid&&mem_ready&&ram_ready)
            mem_access_count<=mem_access_count+1;

        // same qualifying condition as mem_access_count, minus fetches --
        // ram_ready alone can't tell a data load/store from an instruction
        // fetch, that's what mem_instr is for
        if(mem_valid&&mem_ready&&ram_ready&&!mem_instr)
            data_ram_count<=data_ram_count+1;

        if(mem_valid&&mem_ready&&mmio_select)
            mmio_access_count<=mmio_access_count+1;
    end
end


// Debug indicators
reg saw_mem_request=0;
reg saw_instruction_fetch=0;
reg saw_mmio_write=0;

// MMIO acknowledgement and writes
reg mmio_ready=0;
reg [31:0] mmio_rdata=0;

always @(posedge sys_clk)
begin
    mmio_ready<=0;

    // M6: this block previously had no resetn handling at all -- harmless
    // for cold boot (FPGA flip-flops take their declared initial value
    // straight from the bitstream at configuration), but a host-triggered
    // SYSTEM_RESET is a runtime pulse, not a reconfiguration, so without
    // this these registers would silently keep their pre-reset values.
    // gpio_out in particular drives LED0-6 directly and must not still
    // show a stale fault code after recovery, and the snapshot registers
    // are explicitly required to reset. mmio_ready/mmio_rdata don't need
    // it: mmio_ready already self-clears every cycle above, and stale
    // mmio_rdata is never observed since mem_rdata only selects it when
    // mmio_ready is genuinely 1 (same reasoning ram_rdata already relies
    // on without its own resetn gating).
    if(!resetn)
    begin
        gpio_out<=0;
        debug_ctrl<=0;
        workload_scratch<=0;
        snap_cycle_count<=0;
        snap_instr_count<=0;
        snap_mem_count<=0;
        snap_mmio_count<=0;
        snap_data_ram_count<=0;
    end
    else
    begin
    if(mmio_select&&!mmio_ready)
    begin
        mmio_ready<=1;

        case(mem_addr)

            GPIO_OUT_ADDR:
            begin
                if(mem_wstrb!=0)
                    gpio_out<=mem_wdata[7:0];

                mmio_rdata<={24'b0,gpio_out};
            end

            SYS_STATUS_ADDR:
            begin
                mmio_rdata<={
                    26'b0,
                    trap,
                    saw_mmio_write,
                    saw_instruction_fetch,
                    saw_mem_request,
                    resetn,
                    1'b1
                };
            end

            CYCLE_COUNT_LO:
            begin
                mmio_rdata<=cycle_counter[31:0];
            end

            CYCLE_COUNT_HI:
            begin
                mmio_rdata<=cycle_counter[63:32];
            end

            DEBUG_CTRL_ADDR:
            begin
                if(mem_wstrb!=0)
                    debug_ctrl<=mem_wdata;

                mmio_rdata<=debug_ctrl;
            end

            INSTR_COUNT_ADDR:
            begin
                mmio_rdata<=instr_count;
            end

            MEM_COUNT_ADDR:
            begin
                mmio_rdata<=mem_access_count;
            end

            MMIO_COUNT_ADDR:
            begin
                mmio_rdata<=mmio_access_count;
            end
            SNAP_CTRL_ADDR:
            begin
                if(mem_wstrb!=0 && mem_wdata[0])
                begin
                    snap_cycle_count<=cycle_counter;
                    snap_instr_count<=instr_count;
                    snap_mem_count<=mem_access_count;
                    snap_mmio_count<=mmio_access_count;
                    snap_data_ram_count<=data_ram_count;
                end

                mmio_rdata<=32'b0;
            end

            SNAP_CYCLE_LO_ADDR:
            begin
                mmio_rdata<=snap_cycle_count[31:0];
            end

            SNAP_CYCLE_HI_ADDR:
            begin
                mmio_rdata<=snap_cycle_count[63:32];
            end

            SNAP_INSTR_ADDR:
            begin
                mmio_rdata<=snap_instr_count;
            end

            SNAP_MEM_ADDR:
            begin
                mmio_rdata<=snap_mem_count;
            end

            SNAP_MMIO_ADDR:
            begin
                mmio_rdata<=snap_mmio_count;
            end

            WORKLOAD_SELECT_ADDR:
            begin
                mmio_rdata<={29'b0,workload_select};
            end

            DATA_RAM_COUNT_ADDR:
            begin
                mmio_rdata<=data_ram_count;
            end

            SNAP_DATA_RAM_ADDR:
            begin
                mmio_rdata<=snap_data_ram_count;
            end

            WORKLOAD_SCRATCH_ADDR:
            begin
                if(mem_wstrb!=0)
                    workload_scratch<=mem_wdata;

                mmio_rdata<=workload_scratch;
            end

            default:
                mmio_rdata<=32'b0;

        endcase
    end

    // Reuses the exact SNAP_* registers the firmware-triggered SNAP_CTRL
    // write above uses, so a host-captured snapshot and a firmware-captured
    // one have identical coherence semantics -- not a second snapshot path.
    // If both triggers land on the same cycle they'd write identical values
    // anyway (same source counters), so no arbitration is needed here.
    if(cmd_capture_snapshot)
    begin
        snap_cycle_count<=cycle_counter;
        snap_instr_count<=instr_count;
        snap_mem_count<=mem_access_count;
        snap_mmio_count<=mmio_access_count;
        snap_data_ram_count<=data_ram_count;
    end
    end
end


// Return data / ready to processor
assign mem_ready =
    ram_ready ||
    mmio_ready;

assign mem_rdata =
    ram_ready ? (inject_this_fetch ? ILLEGAL_INSTRUCTION : ram_rdata) :
    mmio_ready ? mmio_rdata :
    32'b0;


always @(posedge sys_clk)
begin
    if(mem_valid)
        saw_mem_request<=1;

    if(mem_valid&&mem_instr)
        saw_instruction_fetch<=1;

    if(mem_valid &&
       mem_addr==GPIO_OUT_ADDR &&
       mem_wstrb!=0)
        saw_mmio_write<=1;
end



// LEDs
assign led[0]=gpio_out[0];
assign led[1]=gpio_out[1];
assign led[2]=gpio_out[2];
assign led[3]=gpio_out[3];
assign led[4]=gpio_out[4];
assign led[5]=gpio_out[5];
assign led[6]=gpio_out[6];
assign led[7]=trap;

// RISC-V CPU
picorv32 #(
    .PROGADDR_RESET(32'h00000000),

    .STACKADDR(32'h00001000),

    .ENABLE_COUNTERS(0),

    .ENABLE_COUNTERS64(0),

    .COMPRESSED_ISA(0),

    .CATCH_MISALIGN(1),

    .CATCH_ILLINSN(1)
) cpu (
    .clk(sys_clk),

    .resetn(resetn),

    .trap(trap),

    .mem_valid(mem_valid),

    .mem_instr(mem_instr),

    .mem_ready(mem_ready),

    .mem_addr(mem_addr),

    .mem_wdata(mem_wdata),

    .mem_wstrb(mem_wstrb),

    .mem_rdata(mem_rdata),

    .irq(32'b0)
);

uart_tx #(
    .CLK_FREQ(50000000),
    .BAUD(115200)
) uart0 (
    .clk(sys_clk),
    .resetn(resetn),
    .start(uart_start),
    .data(uart_data),
    .tx(uart_tx_line),
    .busy(uart_busy)
);

assign usb_tx=uart_tx_line;

always @(posedge sys_clk)
begin
    uart_start<=0;
    resp_consumed<=0;
    resp_pending_d1<=resp_pending;
    reset_pulse_trigger<=0;

    if(!resetn)
    begin
        uart_state<=0;
        uart_interval<=0;
        awaiting_reset_flush<=0;
    end
    else if(uart_state==0)
    begin
        // Responses jump ahead of a new telemetry packet -- the host is
        // actively waiting on one, telemetry is a free-running background
        // stream that can absorb the delay. uart_interval keeps counting
        // through the whole thing since it only advances here at uart_state==0,
        // same as it already pauses during an ordinary telemetry send.
        if(resp_pending_d1)
        begin
            resp_consumed<=1;
            resp_tx_cmd_echo<=resp_cmd_echo;
            resp_tx_status<=resp_status;
            resp_tx_data<=resp_data_value;
            resp_tx_checksum<=resp_checksum_value;

            uart_state<=31; // response header, see states 31-40 below
        end
        else if(uart_interval==49999999)
        begin
            uart_interval<=0;

            uart_cycle_snapshot<=cycle_counter;
            uart_instr_snapshot<=instr_count;
            uart_mem_snapshot<=mem_access_count;
            uart_mmio_snapshot<=mmio_access_count;
            uart_data_ram_snapshot<=data_ram_count;
            uart_workload_snapshot<={5'b0,workload_select};

            uart_flags_snapshot<={
                4'b0,
                cmd_snapshot_seen,
                cmd_ping_seen,
                trap,
                (gpio_out[5:0]==6'h3F)
            };

            uart_state<=1;
        end
        else
        begin
            uart_interval<=uart_interval+1;
        end
    end
    else if(!uart_busy&&!uart_start)
    begin
        case(uart_state)

            //Packet header. Version bumped 0x01->0x02 in M4: DATA_RAM_COUNT
            //and WORKLOAD_ID extend the payload length, so an old decoder
            //expecting exactly 22 bytes needs a way to notice before it
            //misframes the next packet against the leftover bytes.
            1:  begin uart_data<=8'hA5; uart_start<=1; uart_state<=2;  end
            2:  begin uart_data<=8'h5A; uart_start<=1; uart_state<=3;  end
            3:  begin uart_data<=8'h02; uart_start<=1; uart_state<=4;  end
            4:  begin uart_data<=uart_flags_snapshot; uart_start<=1; uart_state<=5; end

            //64-bit cycle counter, little endian
            5:  begin uart_data<=uart_cycle_snapshot[7:0];   uart_start<=1; uart_state<=6;  end
            6:  begin uart_data<=uart_cycle_snapshot[15:8];  uart_start<=1; uart_state<=7;  end
            7:  begin uart_data<=uart_cycle_snapshot[23:16]; uart_start<=1; uart_state<=8;  end
            8:  begin uart_data<=uart_cycle_snapshot[31:24]; uart_start<=1; uart_state<=9;  end
            9:  begin uart_data<=uart_cycle_snapshot[39:32]; uart_start<=1; uart_state<=10; end
            10: begin uart_data<=uart_cycle_snapshot[47:40]; uart_start<=1; uart_state<=11; end
            11: begin uart_data<=uart_cycle_snapshot[55:48]; uart_start<=1; uart_state<=12; end
            12: begin uart_data<=uart_cycle_snapshot[63:56]; uart_start<=1; uart_state<=13; end

            //Instruction counter
            13: begin uart_data<=uart_instr_snapshot[7:0];   uart_start<=1; uart_state<=14; end
            14: begin uart_data<=uart_instr_snapshot[15:8];  uart_start<=1; uart_state<=15; end
            15: begin uart_data<=uart_instr_snapshot[23:16]; uart_start<=1; uart_state<=16; end
            16: begin uart_data<=uart_instr_snapshot[31:24]; uart_start<=1; uart_state<=17; end

            //RAM access counter (includes instruction fetches, historical meaning)
            17: begin uart_data<=uart_mem_snapshot[7:0];   uart_start<=1; uart_state<=18; end
            18: begin uart_data<=uart_mem_snapshot[15:8];  uart_start<=1; uart_state<=19; end
            19: begin uart_data<=uart_mem_snapshot[23:16]; uart_start<=1; uart_state<=20; end
            20: begin uart_data<=uart_mem_snapshot[31:24]; uart_start<=1; uart_state<=21; end

            //MMIO access counter
            21: begin uart_data<=uart_mmio_snapshot[7:0];   uart_start<=1; uart_state<=22; end
            22: begin uart_data<=uart_mmio_snapshot[15:8];  uart_start<=1; uart_state<=23; end
            23: begin uart_data<=uart_mmio_snapshot[23:16]; uart_start<=1; uart_state<=24; end
            24: begin uart_data<=uart_mmio_snapshot[31:24]; uart_start<=1; uart_state<=25; end

            //Data RAM access counter (excludes instruction fetches -- new in M4)
            25: begin uart_data<=uart_data_ram_snapshot[7:0];   uart_start<=1; uart_state<=26; end
            26: begin uart_data<=uart_data_ram_snapshot[15:8];  uart_start<=1; uart_state<=27; end
            27: begin uart_data<=uart_data_ram_snapshot[23:16]; uart_start<=1; uart_state<=28; end
            28: begin uart_data<=uart_data_ram_snapshot[31:24]; uart_start<=1; uart_state<=29; end

            //Active workload ID
            29: begin uart_data<=uart_workload_snapshot; uart_start<=1; uart_state<=30; end

            //Wait for the next reporting interval
            30: uart_state<=0;

            //Response header
            31: begin uart_data<=RESP_MAGIC0;      uart_start<=1; uart_state<=32; end
            32: begin uart_data<=RESP_MAGIC1;      uart_start<=1; uart_state<=33; end
            33: begin uart_data<=CMD_PROTO_VER;    uart_start<=1; uart_state<=34; end
            34: begin uart_data<=resp_tx_cmd_echo; uart_start<=1; uart_state<=35; end
            35: begin uart_data<=resp_tx_status;   uart_start<=1; uart_state<=36; end

            //32-bit result, little endian (0 unless CAPTURE_SNAPSHOT/SET_WORKLOAD ACK)
            36: begin uart_data<=resp_tx_data[7:0];   uart_start<=1; uart_state<=37; end
            37: begin uart_data<=resp_tx_data[15:8];  uart_start<=1; uart_state<=38; end
            38: begin uart_data<=resp_tx_data[23:16]; uart_start<=1; uart_state<=39; end
            39: begin uart_data<=resp_tx_data[31:24]; uart_start<=1; uart_state<=40; end

            // M6: arm awaiting_reset_flush right here, not after uart_state
            // returns to 0 -- resp_tx_cmd_echo/resp_tx_status are still the
            // values latched when this response committed, and checking
            // resp_tx_status==RESP_ACK means a NACKed SYSTEM_RESET (e.g.
            // bad checksum) can never arm a reset.
            40: begin
                uart_data<=resp_tx_checksum;
                uart_start<=1;
                uart_state<=0;

                if(resp_tx_cmd_echo==CMD_SYSTEM_RESET && resp_tx_status==RESP_ACK)
                    awaiting_reset_flush<=1;
            end

            default: uart_state<=0;

        endcase
    end

    // M6: fires exactly once, on the cycle the checksum byte armed above
    // has actually finished shifting out (busy went 1 then back to 0) --
    // not on the cycle uart_state first returns to 0, which happens
    // before that byte has even started transmitting. This is what
    // guarantees the SYSTEM_RESET ACK is off the wire before resetn drops.
    if(awaiting_reset_flush && !uart_busy && !uart_start)
    begin
        awaiting_reset_flush<=0;
        reset_pulse_trigger<=1;
    end
end

uart_rx #(
    .CLK_FREQ(50000000),
    .BAUD(115200)
) uart_rx0 (
    .clk(sys_clk),
    .resetn(resetn),
    .rx(usb_rx),
    .data(rx_data),
    .valid(rx_valid)
);

// A malformed or partial frame must never produce a command side effect --
// any magic/version/checksum mismatch just drops back to RX_IDLE and waits
// for the next 0xA5. Only a fully verified frame reaches RX_CHECKSUM's
// dispatch. Unrecognized-but-valid command IDs are accepted and silently
// ignored, not treated as an error.
always @(posedge sys_clk)
begin
    // command pulses default low every cycle so one received frame can't
    // retrigger its side effect on a later cycle where nothing arrived
    cmd_reset_counters<=0;
    cmd_capture_snapshot<=0;

    if(!resetn)
    begin
        cmd_state<=RX_IDLE;
        cmd_timeout<=0;
        cmd_ping_seen<=0;
        cmd_snapshot_seen<=0;
        resp_pending<=0;
        fault_pending<=0;
        workload_select<=0; // M6: reset default is IDLE, a clean reboot behaves like a clean boot
    end
    else begin

    // Cleared here by default once the arbiter has drained it; overridden
    // below (later in program order, so it wins) if a new frame classifies
    // on the exact same cycle -- a fresh command's response is never
    // dropped just because the previous one's bookkeeping finished first.
    if(resp_consumed)
        resp_pending<=0;

    // Same split-ownership shape as resp_pending/resp_consumed above:
    // inject_this_fetch is computed one cycle later in the synchronous RAM
    // block, the exact cycle the armed fetch actually completes, so that's
    // where the arm gets consumed.
    if(inject_this_fetch)
        fault_pending<=0;

    if(rx_valid)
    begin
        cmd_timeout<=0;

        case(cmd_state)

            RX_IDLE:
                if(rx_data==CMD_MAGIC0)
                    cmd_state<=RX_MAGIC1;

            RX_MAGIC1:
                cmd_state<=(rx_data==CMD_MAGIC1) ? RX_VERSION : RX_IDLE;

            RX_VERSION:
            begin
                if(rx_data==CMD_PROTO_VER)
                begin
                    cmd_checksum_calc<=rx_data;
                    cmd_state<=RX_CMDID;
                end
                else
                begin
                    // no CMD byte has been read yet at this point in the
                    // frame, so there's nothing real to echo
                    resp_pending<=1;
                    resp_cmd_echo<=8'h00;
                    resp_status<=RESP_NACK_BAD_VERSION;
                    cmd_state<=RX_IDLE;
                end
            end

            RX_CMDID:
            begin
                cmd_id<=rx_data;
                cmd_checksum_calc<=cmd_checksum_calc^rx_data;
                cmd_state<=RX_ARG0;
            end

            RX_ARG0:
            begin
                cmd_arg[7:0]<=rx_data;
                cmd_checksum_calc<=cmd_checksum_calc^rx_data;
                cmd_state<=RX_ARG1;
            end

            RX_ARG1:
            begin
                cmd_arg[15:8]<=rx_data;
                cmd_checksum_calc<=cmd_checksum_calc^rx_data;
                cmd_state<=RX_ARG2;
            end

            RX_ARG2:
            begin
                cmd_arg[23:16]<=rx_data;
                cmd_checksum_calc<=cmd_checksum_calc^rx_data;
                cmd_state<=RX_ARG3;
            end

            RX_ARG3:
            begin
                cmd_arg[31:24]<=rx_data;
                cmd_checksum_calc<=cmd_checksum_calc^rx_data;
                cmd_state<=RX_CHECKSUM;
            end

            RX_CHECKSUM:
            begin
                // the CMD byte was received either way -- echo it even on
                // a checksum failure, it's useful diagnostic information
                // even though it isn't guaranteed to be the byte the host
                // actually intended if the corruption was elsewhere
                resp_pending<=1;
                resp_cmd_echo<=cmd_id;

                if(rx_data==cmd_checksum_calc)
                begin
                    case(cmd_id)
                        CMD_PING:
                        begin
                            cmd_ping_seen<=1;
                            resp_status<=RESP_ACK;
                        end

                        CMD_RESET_COUNTERS:
                        begin
                            cmd_reset_counters<=1;
                            resp_status<=RESP_ACK;
                        end

                        CMD_CAPTURE_SNAPSHOT:
                        begin
                            cmd_capture_snapshot<=1;
                            cmd_snapshot_seen<=1;
                            resp_status<=RESP_ACK;
                        end

                        CMD_SET_WORKLOAD:
                        begin
                            if(cmd_arg<=WORKLOAD_MAX)
                            begin
                                workload_select<=cmd_arg[2:0];
                                resp_status<=RESP_ACK;
                            end
                            else
                                resp_status<=RESP_NACK_BAD_ARGUMENT;
                        end

                        CMD_INJECT_FAULT:
                        begin
                            // Arg is reserved/unused -- one deterministic
                            // fault type in M6, nothing to select. Arming
                            // again while already trapped is harmless: a
                            // trapped PicoRV32 never issues another
                            // instruction fetch (see cpu_state_trap in
                            // picorv32.v), so fault_pending would just sit
                            // armed, unconsumed, until SYSTEM_RESET clears
                            // it -- not undefined behavior, just inert.
                            fault_pending<=1;
                            resp_status<=RESP_ACK;
                        end

                        CMD_SYSTEM_RESET:
                        begin
                            // No RTL side effect here -- the actual reset
                            // is armed downstream in the TX arbiter (state
                            // 40) once this ACK has fully left the
                            // transmitter, not at command-accept time. See
                            // awaiting_reset_flush/reset_pulse_trigger.
                            resp_status<=RESP_ACK;
                        end

                        default:
                            resp_status<=RESP_NACK_UNKNOWN_CMD;
                    endcase
                end
                else
                    resp_status<=RESP_NACK_BAD_CHECKSUM;

                cmd_state<=RX_IDLE;
            end

            default:
                cmd_state<=RX_IDLE;

        endcase
    end
    else if(cmd_state!=RX_IDLE)
    begin
        if(cmd_timeout==CMD_TIMEOUT_CYCLES-1)
            cmd_state<=RX_IDLE;
        else
            cmd_timeout<=cmd_timeout+1;
    end

    end
end

endmodule