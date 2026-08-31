module top(
    input wire clk,
    output wire [7:0] led
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

endmodule