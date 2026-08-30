module top(
    input wire clk,
    output wire [7:0] led
);

//Clock divider
//The Alchitry Cu provides a 100 MHz input clock. The current PicoRV32 SoC does not meet timing at 100 MHz, with place-and-route showing an
//achievable frequency of roughly 74 MHz. The system therefore runs from a divided 50 MHz clock, providing timing margin while retaining 
//substantially more performance than the initial 25 MHz bring-up.

reg [1:0] clkdiv=0;

always @(posedge clk)
    clkdiv<=clkdiv+1;

wire sys_clk=clkdiv[0];


//
// Reset generation
//

reg [7:0] reset_counter=0;
wire resetn;

always @(posedge sys_clk)
begin
    if(!reset_counter[7])
        reset_counter<=reset_counter+1;
end

assign resetn=reset_counter[7];



//PicoRV32 memory interface
wire trap;

wire mem_valid;
wire mem_instr;
wire mem_ready;

wire [31:0] mem_addr;
wire [31:0] mem_wdata;
wire [3:0] mem_wstrb;
wire [31:0] mem_rdata;



//4 KB RAM

reg [31:0] memory [0:1023];

reg ram_ready=0;
reg [31:0] ram_rdata=0;

initial begin
    $readmemh("firmware/firmware.hex",memory);
end



//MMIO register bank
localparam GPIO_OUT_ADDR     = 32'h10000000;
localparam SYS_STATUS_ADDR   = 32'h10000004;
localparam CYCLE_COUNT_LO    = 32'h10000008;
localparam CYCLE_COUNT_HI    = 32'h1000000C;
localparam DEBUG_CTRL_ADDR   = 32'h10000010;

reg [7:0] gpio_out=0;
reg [31:0] debug_ctrl=0;
reg [63:0] cycle_counter=0;

wire ram_select;
wire mmio_select;

assign ram_select =
    mem_valid &&
    (mem_addr<32'h00001000);

assign mmio_select =
    mem_valid &&
    (mem_addr>=32'h10000000) &&
    (mem_addr<=32'h10000010);


//Synchronous RAM
//This follows the style used by PicoRV32's reference PicoSoC memory implementation.
always @(posedge sys_clk)
begin
    ram_ready<=mem_valid &&
               !ram_ready &&
               (mem_addr<32'h00001000);

    if(mem_valid &&
       !ram_ready &&
       (mem_addr<32'h00001000))
    begin
        ram_rdata<=memory[mem_addr[11:2]];

        if(mem_wstrb[0])
            memory[mem_addr[11:2]][7:0]<=mem_wdata[7:0];

        if(mem_wstrb[1])
            memory[mem_addr[11:2]][15:8]<=mem_wdata[15:8];

        if(mem_wstrb[2])
            memory[mem_addr[11:2]][23:16]<=mem_wdata[23:16];

        if(mem_wstrb[3])
            memory[mem_addr[11:2]][31:24]<=mem_wdata[31:24];
    end
end


//
// LED MMIO acknowledgement
//

//Hardware cycle counter
always @(posedge sys_clk)
begin
    if(!resetn)
        cycle_counter<=0;
    else
        cycle_counter<=cycle_counter+1;
end


//Debug indicators
reg saw_mem_request=0;
reg saw_instruction_fetch=0;
reg saw_mmio_write=0;

//MMIO acknowledgement and writes
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

            default:
                mmio_rdata<=32'b0;

        endcase
    end
end


//Return data / ready to processor
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
       mem_addr==32'h10000000 &&
       mem_wstrb!=0)
        saw_mmio_write<=1;
end



// LEDs
assign led[0]=gpio_out[0];
assign led[1]=gpio_out[1];
assign led[2]=saw_mem_request;
assign led[3]=saw_instruction_fetch;
assign led[4]=saw_mmio_write;
assign led[5]=trap;
assign led[6]=resetn;
assign led[7]=0;

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