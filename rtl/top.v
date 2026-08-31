module top(
    input wire clk,
    output wire [7:0] led,
    output wire usb_tx,
    input wire usb_rx
);

//clock divider
//100mhz input doesn't meet timing for the picorv32 setup, pnr only gets
//about 74mhz out of it, so we run everything off a divided 50mhz clock

reg [1:0] clkdiv=0;

always @(posedge clk)
    clkdiv<=clkdiv+1;

wire sys_clk=clkdiv[0];

//reset generation
//resetn comes from two sources so it's a plain wire, never written from
//two places. poweron_counter is the original cold boot reset, and
//reset_pulse_active is the host triggered SYSTEM_RESET pulse (armed once
//the ack for it is fully sent, see the tx arbiter below)

reg [7:0] poweron_counter=0;
wire poweron_done=poweron_counter[7];

always @(posedge sys_clk)
begin
    if(!poweron_done)
        poweron_counter<=poweron_counter+1;
end

//128 cycles matches the power-on reset duration so both resets behave
//the same way
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


//picorv32 memory interface
wire trap;

wire mem_valid;
wire mem_instr;
wire mem_ready;

wire [31:0] mem_addr;
wire [31:0] mem_wdata;
wire [3:0] mem_wstrb;
wire [31:0] mem_rdata;


//4kb ram

localparam RAM_BYTES = 32'h00001000;

reg [31:0] memory [0:1023];

reg ram_ready=0;
reg [31:0] ram_rdata=0;

//word index into memory[], mem_addr is byte addressed but memory[] is 32 bit
wire [9:0] ram_word_addr = mem_addr[11:2];

//one shot illegal instruction fault injection
//0x00000000 is guaranteed illegal here - opcode bits[6:0]=0000000 doesn't
//match any RV32I opcode (checked this against picorv32's decoder directly),
//and it's also the spec's own "definitely illegal" encoding so it's not
//just something that happens to work on this core
localparam ILLEGAL_INSTRUCTION = 32'h00000000;

//fault_pending gets armed by CMD_INJECT_FAULT down in the rx parser, same
//pattern as resp_pending/resp_consumed - one block sets it, one clears it.
//inject_this_fetch mirrors ram_ready's timing so it lines up with the
//actual fetch completing, only real instruction fetches trip this
reg fault_pending=0;
reg inject_this_fetch=0;

initial begin
    $readmemh("firmware/firmware.hex",memory);
end



//mmio register bank
localparam GPIO_OUT_ADDR = 32'h10000000;
localparam SYS_STATUS_ADDR = 32'h10000004;
localparam CYCLE_COUNT_LO = 32'h10000008;
localparam CYCLE_COUNT_HI = 32'h1000000C;
localparam DEBUG_CTRL_ADDR = 32'h10000010;
localparam INSTR_COUNT_ADDR = 32'h10000014;
localparam MEM_COUNT_ADDR = 32'h10000018;
localparam MMIO_COUNT_ADDR = 32'h1000001C;

localparam SNAP_CTRL_ADDR = 32'h10000020;
localparam SNAP_CYCLE_LO_ADDR = 32'h10000024;
localparam SNAP_CYCLE_HI_ADDR = 32'h10000028;
localparam SNAP_INSTR_ADDR = 32'h1000002C;
localparam SNAP_MEM_ADDR = 32'h10000030;
localparam SNAP_MMIO_ADDR = 32'h10000034;

localparam WORKLOAD_SELECT_ADDR = 32'h10000038;
localparam DATA_RAM_COUNT_ADDR = 32'h1000003C;
localparam SNAP_DATA_RAM_ADDR = 32'h10000040;
localparam WORKLOAD_SCRATCH_ADDR = 32'h10000044;

localparam MMIO_BASE = GPIO_OUT_ADDR;
localparam MMIO_TOP = DEBUG_CTRL_ADDR;

reg [7:0] gpio_out=0;
reg [31:0] debug_ctrl=0;
reg [63:0] cycle_counter=0;

reg [31:0] instr_count=0;
reg [31:0] mem_access_count=0;
reg [31:0] mmio_access_count=0;

//MEM_COUNT already means "total ram access incl fetches" and that's a
//physically verified meaning we don't want to break, so this tracks just
//the data traffic separately (what a memory vs alu comparison needs)
reg [31:0] data_ram_count=0;

reg [63:0] snap_cycle_count=0;
reg [31:0] snap_instr_count=0;
reg [31:0] snap_mem_count=0;
reg [31:0] snap_mmio_count=0;
reg [31:0] snap_data_ram_count=0;

//only the rx parser writes this, firmware just reads it over mmio
//3 bits covers workload ids 0-5 with room to spare
reg [2:0] workload_select=0;

//plain r/w scratch word like debug_ctrl, gives the mmio workload
//something to hit without touching gpio/snap_ctrl/debug_ctrl
reg [31:0] workload_scratch=0;

reg [7:0] uart_data=0;
reg uart_start=0;
wire uart_busy;
wire uart_tx_line;

reg [5:0] uart_state=0; //widened from [4:0], response states got shuffled around a few times
reg [25:0] uart_interval=0;

//set when the system_reset ack's last byte gets handed to uart_tx, cleared
//once that byte is actually done sending. uart_state goes back to 0 before
//the byte even starts shifting out, so checking uart_state alone isn't
//enough - need to watch uart_busy go high then low again
reg awaiting_reset_flush=0;

//one cycle pulse, tells the reset generator up top it's safe to reset now
reg reset_pulse_trigger=0;

reg [63:0] uart_cycle_snapshot=0;
reg [31:0] uart_instr_snapshot=0;
reg [31:0] uart_mem_snapshot=0;
reg [31:0] uart_mmio_snapshot=0;
reg [7:0] uart_flags_snapshot=0;
reg [31:0] uart_data_ram_snapshot=0;
reg [7:0] uart_workload_snapshot=0;

//one cycle behind resp_pending on purpose - snap_instr_count updates a
//cycle after cmd_capture_snapshot pulses, so without this delay we'd
//latch the response data before the snapshot register actually updates
reg resp_pending_d1=0;

//latched when the arbiter commits to sending so the byte sequence below
//stays stable even if a new command comes in mid transmission
reg resp_consumed=0;
reg [7:0] resp_tx_cmd_echo=0;
reg [7:0] resp_tx_status=0;
reg [31:0] resp_tx_data=0;
reg [7:0] resp_tx_checksum=0;

//only capture_snapshot and set_workload have a real result value,
//everything else just reports 0. reading workload_select here instead of
//cmd_arg since it's already settled by the time the arbiter commits
wire [31:0] resp_data_value =
    (resp_cmd_echo==CMD_CAPTURE_SNAPSHOT && resp_status==RESP_ACK) ? snap_instr_count :
    (resp_cmd_echo==CMD_SET_WORKLOAD && resp_status==RESP_ACK) ? {29'b0,workload_select} :
    32'b0;

wire [7:0] resp_checksum_value =
    CMD_PROTO_VER ^ resp_cmd_echo ^ resp_status ^
    resp_data_value[7:0] ^ resp_data_value[15:8] ^ resp_data_value[23:16] ^ resp_data_value[31:24];

//host command frame: MAGIC(2) VERSION(1) CMD(1) ARG(4 little endian) CHECKSUM(1)
//checksum is xor of version/cmd/arg bytes, magic isn't included since it's
//just the framing signal not payload
localparam CMD_MAGIC0 = 8'hA5;
localparam CMD_MAGIC1 = 8'h5A;
localparam CMD_PROTO_VER = 8'h01;
localparam CMD_PING = 8'h01;
localparam CMD_RESET_COUNTERS = 8'h02;
localparam CMD_CAPTURE_SNAPSHOT = 8'h03;
localparam CMD_SET_WORKLOAD = 8'h04;
localparam CMD_INJECT_FAULT = 8'h05;
localparam CMD_SYSTEM_RESET = 8'h06;

localparam WORKLOAD_MAX = 32'd5; //idle..mixed, see firmware/main.c

//response packet: MAGIC0 MAGIC1 VERSION CMD_ECHO STATUS DATA[0..3] CHECKSUM
//same checksum convention as the command frame. magic1 is 0x5B here, not
//0x5A, so the host can tell response vs telemetry apart right away
localparam RESP_MAGIC0 = 8'hA5;
localparam RESP_MAGIC1 = 8'h5B;

localparam RESP_ACK = 8'h00;
localparam RESP_NACK_BAD_CHECKSUM = 8'h01;
localparam RESP_NACK_BAD_VERSION = 8'h02;
localparam RESP_NACK_UNKNOWN_CMD = 8'h03;
localparam RESP_NACK_BAD_ARGUMENT = 8'h04;

localparam RX_IDLE = 0;
localparam RX_MAGIC1 = 1;
localparam RX_VERSION = 2;
localparam RX_CMDID = 3;
localparam RX_ARG0 = 4;
localparam RX_ARG1 = 5;
localparam RX_ARG2 = 6;
localparam RX_ARG3 = 7;
localparam RX_CHECKSUM = 8;

//20000 cycles = 400us @ 50mhz, ~4.6 byte times, plenty of margin for host
//scheduling jitter without taking forever to resync if a frame stalls
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

//predates the response packets, keeping it since it's cheap and the
//older dashboard path still uses it
reg cmd_snapshot_seen=0;

//only one outstanding response at a time - rx parser latches what to
//send, tx arbiter drains it when the line is free. if a second command
//comes in before the first response goes out, it overwrites
//resp_cmd_echo/status and the first ack gets lost (the command's actual
//side effect already happened though, just the ack for it is gone).
//host is expected to wait for a response before sending the next command
reg resp_pending=0;
reg [7:0] resp_cmd_echo=0;
reg [7:0] resp_status=0;

wire mmio_select;

assign mmio_select =
    mem_valid &&
    (mem_addr>=32'h10000000) &&
    (mem_addr<=32'h10000044);


//synchronous ram, same style as picorv32's reference picosoc memory
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


//hardware cycle counter
always @(posedge sys_clk)
begin
    if(!resetn)
        cycle_counter<=0;
    else
        cycle_counter<=cycle_counter+1;
end

//execution monitor
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
        //host reset wins over a same cycle bus event so RESET_COUNTERS gives
        //a clean boundary instead of racing whatever the cpu is doing.
        //cycle_counter stays untouched, it's uptime not an experiment
        //counter and POST needs it monotonic
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

        //same condition as mem_access_count but skip fetches - mem_instr
        //tells us if this is a real data access vs an instruction fetch
        if(mem_valid&&mem_ready&&ram_ready&&!mem_instr)
            data_ram_count<=data_ram_count+1;

        if(mem_valid&&mem_ready&&mmio_select)
            mmio_access_count<=mmio_access_count+1;
    end
end


//debug indicators
reg saw_mem_request=0;
reg saw_instruction_fetch=0;
reg saw_mmio_write=0;

//mmio ack and writes
reg mmio_ready=0;
reg [31:0] mmio_rdata=0;

always @(posedge sys_clk)
begin
    mmio_ready<=0;

    //this block used to have no resetn handling, which was fine for cold
    //boot since the fpga just loads initial values from the bitstream. but
    //a SYSTEM_RESET is a runtime pulse not a reconfig, so these would keep
    //their old values without this. gpio_out especially needs to clear or
    //it'd still show a stale fault code after recovery. mmio_ready/rdata
    //don't need resetting, same reasoning as ram_rdata not needing it
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

    //reuses the same SNAP_* registers the firmware SNAP_CTRL write above
    //uses, so host and firmware snapshots behave identically, not a
    //separate path. if both happen to fire the same cycle they'd write
    //the same values anyway so no arbitration needed
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


//return data/ready to the cpu
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


//leds
assign led[0]=gpio_out[0];
assign led[1]=gpio_out[1];
assign led[2]=gpio_out[2];
assign led[3]=gpio_out[3];
assign led[4]=gpio_out[4];
assign led[5]=gpio_out[5];
assign led[6]=gpio_out[6];
assign led[7]=trap;

//risc-v cpu
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
        //responses jump ahead of telemetry, host is actively waiting on a
        //response but telemetry is just a free running background stream
        //that can wait. uart_interval keeps counting through this since it
        //only advances at uart_state==0 anyway, same as during a normal send
        if(resp_pending_d1)
        begin
            resp_consumed<=1;
            resp_tx_cmd_echo<=resp_cmd_echo;
            resp_tx_status<=resp_status;
            resp_tx_data<=resp_data_value;
            resp_tx_checksum<=resp_checksum_value;

            uart_state<=31; //response header, see states 31-40 below
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

            //packet header. version bumped 0x01->0x02 when we added
            //DATA_RAM_COUNT and WORKLOAD_ID so an old decoder can tell
            //before it misframes the next packet
            1:  begin uart_data<=8'hA5; uart_start<=1; uart_state<=2;  end
            2:  begin uart_data<=8'h5A; uart_start<=1; uart_state<=3;  end
            3:  begin uart_data<=8'h02; uart_start<=1; uart_state<=4;  end
            4:  begin uart_data<=uart_flags_snapshot; uart_start<=1; uart_state<=5; end

            //64 bit cycle counter, little endian
            5:  begin uart_data<=uart_cycle_snapshot[7:0];   uart_start<=1; uart_state<=6;  end
            6:  begin uart_data<=uart_cycle_snapshot[15:8];  uart_start<=1; uart_state<=7;  end
            7:  begin uart_data<=uart_cycle_snapshot[23:16]; uart_start<=1; uart_state<=8;  end
            8:  begin uart_data<=uart_cycle_snapshot[31:24]; uart_start<=1; uart_state<=9;  end
            9:  begin uart_data<=uart_cycle_snapshot[39:32]; uart_start<=1; uart_state<=10; end
            10: begin uart_data<=uart_cycle_snapshot[47:40]; uart_start<=1; uart_state<=11; end
            11: begin uart_data<=uart_cycle_snapshot[55:48]; uart_start<=1; uart_state<=12; end
            12: begin uart_data<=uart_cycle_snapshot[63:56]; uart_start<=1; uart_state<=13; end

            //instruction counter
            13: begin uart_data<=uart_instr_snapshot[7:0];   uart_start<=1; uart_state<=14; end
            14: begin uart_data<=uart_instr_snapshot[15:8];  uart_start<=1; uart_state<=15; end
            15: begin uart_data<=uart_instr_snapshot[23:16]; uart_start<=1; uart_state<=16; end
            16: begin uart_data<=uart_instr_snapshot[31:24]; uart_start<=1; uart_state<=17; end

            //ram access counter, includes instruction fetches (historical meaning)
            17: begin uart_data<=uart_mem_snapshot[7:0];   uart_start<=1; uart_state<=18; end
            18: begin uart_data<=uart_mem_snapshot[15:8];  uart_start<=1; uart_state<=19; end
            19: begin uart_data<=uart_mem_snapshot[23:16]; uart_start<=1; uart_state<=20; end
            20: begin uart_data<=uart_mem_snapshot[31:24]; uart_start<=1; uart_state<=21; end

            //mmio access counter
            21: begin uart_data<=uart_mmio_snapshot[7:0];   uart_start<=1; uart_state<=22; end
            22: begin uart_data<=uart_mmio_snapshot[15:8];  uart_start<=1; uart_state<=23; end
            23: begin uart_data<=uart_mmio_snapshot[23:16]; uart_start<=1; uart_state<=24; end
            24: begin uart_data<=uart_mmio_snapshot[31:24]; uart_start<=1; uart_state<=25; end

            //data ram access counter, excludes instruction fetches
            25: begin uart_data<=uart_data_ram_snapshot[7:0];   uart_start<=1; uart_state<=26; end
            26: begin uart_data<=uart_data_ram_snapshot[15:8];  uart_start<=1; uart_state<=27; end
            27: begin uart_data<=uart_data_ram_snapshot[23:16]; uart_start<=1; uart_state<=28; end
            28: begin uart_data<=uart_data_ram_snapshot[31:24]; uart_start<=1; uart_state<=29; end

            //active workload id
            29: begin uart_data<=uart_workload_snapshot; uart_start<=1; uart_state<=30; end

            //wait for the next reporting interval
            30: uart_state<=0;

            //response header
            31: begin uart_data<=RESP_MAGIC0;      uart_start<=1; uart_state<=32; end
            32: begin uart_data<=RESP_MAGIC1;      uart_start<=1; uart_state<=33; end
            33: begin uart_data<=CMD_PROTO_VER;    uart_start<=1; uart_state<=34; end
            34: begin uart_data<=resp_tx_cmd_echo; uart_start<=1; uart_state<=35; end
            35: begin uart_data<=resp_tx_status;   uart_start<=1; uart_state<=36; end

            //32 bit result, little endian (0 unless capture_snapshot/set_workload ack)
            36: begin uart_data<=resp_tx_data[7:0];   uart_start<=1; uart_state<=37; end
            37: begin uart_data<=resp_tx_data[15:8];  uart_start<=1; uart_state<=38; end
            38: begin uart_data<=resp_tx_data[23:16]; uart_start<=1; uart_state<=39; end
            39: begin uart_data<=resp_tx_data[31:24]; uart_start<=1; uart_state<=40; end

            //arm awaiting_reset_flush right here, not after uart_state goes
            //back to 0 - resp_tx_cmd_echo/status still hold what was
            //latched when this response committed. checking for ack means
            //a nacked system_reset can't trigger an actual reset
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

    //fires once the checksum byte above is actually done shifting out
    //(busy went high then low), not when uart_state first hits 0 which
    //happens before the byte even starts. this is what keeps the ack
    //ahead of the reset
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

//a bad frame should never cause a side effect - any magic/version/checksum
//mismatch just drops back to idle and waits for the next 0xA5. unknown but
//valid command ids get ack'd and ignored, not treated as an error
always @(posedge sys_clk)
begin
    //pulses default low every cycle so one frame can't retrigger later
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
        workload_select<=0; //reset default is idle, clean reboot should look like a clean boot
    end
    else begin

    //cleared here once the arbiter drains it, overridden below if a new
    //frame classifies the same cycle so we never drop a fresh response
    if(resp_consumed)
        resp_pending<=0;

    //same pattern as resp_pending/resp_consumed, inject_this_fetch gets
    //computed a cycle later in the ram block right when the fetch completes
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
                    //no cmd byte read yet at this point so nothing real to echo
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
                //cmd byte was received either way, echo it even on a bad
                //checksum since it's still useful for debugging even if it
                //might not be what was actually intended
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
                            //arg unused, only one fault type right now.
                            //arming again while already trapped is harmless
                            //since a trapped cpu never fetches again anyway,
                            //so fault_pending just sits armed until a reset
                            //clears it
                            fault_pending<=1;
                            resp_status<=RESP_ACK;
                        end

                        CMD_SYSTEM_RESET:
                        begin
                            //no side effect here, the actual reset gets
                            //armed downstream in the tx arbiter once this
                            //ack is fully sent, see awaiting_reset_flush above
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