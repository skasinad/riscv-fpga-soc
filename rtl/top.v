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

reg [7:0] reset_counter=0;
wire resetn;

always @(posedge sys_clk)
begin
    if(!reset_counter[7])
        reset_counter<=reset_counter+1;
end

assign resetn=reset_counter[7];



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

localparam MMIO_BASE = GPIO_OUT_ADDR;
localparam MMIO_TOP  = DEBUG_CTRL_ADDR;

reg [7:0] gpio_out=0;
reg [31:0] debug_ctrl=0;
reg [63:0] cycle_counter=0;

reg [31:0] instr_count=0;
reg [31:0] mem_access_count=0;
reg [31:0] mmio_access_count=0;

reg [63:0] snap_cycle_count=0;
reg [31:0] snap_instr_count=0;
reg [31:0] snap_mem_count=0;
reg [31:0] snap_mmio_count=0;

reg [7:0] uart_data=0;
reg uart_start=0;
wire uart_busy;
wire uart_tx_line;

reg [5:0] uart_state=0; // widened from [4:0] in M3: response states run 26-35
reg [25:0] uart_interval=0;

reg [63:0] uart_cycle_snapshot=0;
reg [31:0] uart_instr_snapshot=0;
reg [31:0] uart_mem_snapshot=0;
reg [31:0] uart_mmio_snapshot=0;
reg [7:0] uart_flags_snapshot=0;

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

// CAPTURE_SNAPSHOT is the only command with a meaningful result value;
// everything else reports 0 in DATA
wire [31:0] resp_data_value =
    (resp_cmd_echo==CMD_CAPTURE_SNAPSHOT && resp_status==RESP_ACK) ? snap_instr_count : 32'b0;

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
    (mem_addr<=32'h10000034);


// Synchronous RAM
// This follows the style used by PicoRV32's reference PicoSoC memory implementation.
always @(posedge sys_clk)
begin
    ram_ready<=mem_valid &&
               !ram_ready &&
               (mem_addr<RAM_BYTES);

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
    end
    else
    begin
        if(mem_valid&&mem_ready&&mem_instr)
            instr_count<=instr_count+1;

        if(mem_valid&&mem_ready&&ram_ready)
            mem_access_count<=mem_access_count+1;

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
    end
end


// Return data / ready to processor
assign mem_ready =
    ram_ready ||
    mmio_ready;

assign mem_rdata =
    ram_ready ? ram_rdata :
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

    if(!resetn)
    begin
        uart_state<=0;
        uart_interval<=0;
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

            uart_state<=26;
        end
        else if(uart_interval==49999999)
        begin
            uart_interval<=0;

            uart_cycle_snapshot<=cycle_counter;
            uart_instr_snapshot<=instr_count;
            uart_mem_snapshot<=mem_access_count;
            uart_mmio_snapshot<=mmio_access_count;

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

            //Packet header
            1:  begin uart_data<=8'hA5; uart_start<=1; uart_state<=2;  end
            2:  begin uart_data<=8'h5A; uart_start<=1; uart_state<=3;  end
            3:  begin uart_data<=8'h01; uart_start<=1; uart_state<=4;  end
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

            //RAM access counter
            17: begin uart_data<=uart_mem_snapshot[7:0];   uart_start<=1; uart_state<=18; end
            18: begin uart_data<=uart_mem_snapshot[15:8];  uart_start<=1; uart_state<=19; end
            19: begin uart_data<=uart_mem_snapshot[23:16]; uart_start<=1; uart_state<=20; end
            20: begin uart_data<=uart_mem_snapshot[31:24]; uart_start<=1; uart_state<=21; end

            //MMIO access counter
            21: begin uart_data<=uart_mmio_snapshot[7:0];   uart_start<=1; uart_state<=22; end
            22: begin uart_data<=uart_mmio_snapshot[15:8];  uart_start<=1; uart_state<=23; end
            23: begin uart_data<=uart_mmio_snapshot[23:16]; uart_start<=1; uart_state<=24; end
            24: begin uart_data<=uart_mmio_snapshot[31:24]; uart_start<=1; uart_state<=25; end

            //Wait for the next reporting interval
            25: uart_state<=0;

            //Response header
            26: begin uart_data<=RESP_MAGIC0;      uart_start<=1; uart_state<=27; end
            27: begin uart_data<=RESP_MAGIC1;      uart_start<=1; uart_state<=28; end
            28: begin uart_data<=CMD_PROTO_VER;    uart_start<=1; uart_state<=29; end
            29: begin uart_data<=resp_tx_cmd_echo; uart_start<=1; uart_state<=30; end
            30: begin uart_data<=resp_tx_status;   uart_start<=1; uart_state<=31; end

            //32-bit result, little endian (0 unless CAPTURE_SNAPSHOT ACK)
            31: begin uart_data<=resp_tx_data[7:0];   uart_start<=1; uart_state<=32; end
            32: begin uart_data<=resp_tx_data[15:8];  uart_start<=1; uart_state<=33; end
            33: begin uart_data<=resp_tx_data[23:16]; uart_start<=1; uart_state<=34; end
            34: begin uart_data<=resp_tx_data[31:24]; uart_start<=1; uart_state<=35; end

            35: begin uart_data<=resp_tx_checksum; uart_start<=1; uart_state<=0; end

            default: uart_state<=0;

        endcase
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
    end
    else begin

    // Cleared here by default once the arbiter has drained it; overridden
    // below (later in program order, so it wins) if a new frame classifies
    // on the exact same cycle -- a fresh command's response is never
    // dropped just because the previous one's bookkeeping finished first.
    if(resp_consumed)
        resp_pending<=0;

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