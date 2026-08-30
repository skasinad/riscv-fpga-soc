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



//Memory mapped LED register
reg led_reg=0;

wire ram_select;
wire led_select;

assign ram_select =
    mem_valid &&
    (mem_addr<32'h00001000);

assign led_select =
    mem_valid &&
    (mem_addr==32'h10000000);



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

reg led_ready=0;

always @(posedge sys_clk)
begin
    led_ready<=0;

    if(led_select&&!led_ready)
    begin
        led_ready<=1;

        if(mem_wstrb!=0)
            led_reg<=mem_wdata[0];
    end
end



// Return data / ready to processor
assign mem_ready =
    ram_ready ||
    led_ready;

assign mem_rdata =
    ram_ready ? ram_rdata :
    led_select ? {31'b0,led_reg} :
    32'b0;

// Debug indicators
reg saw_mem_request=0;
reg saw_instruction_fetch=0;
reg saw_mmio_write=0;

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
assign led[0]=led_reg;
assign led[1]=resetn;
assign led[2]=saw_mem_request;
assign led[3]=saw_instruction_fetch;
assign led[4]=saw_mmio_write;
assign led[5]=trap;
assign led[6]=0;
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