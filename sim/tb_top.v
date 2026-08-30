`timescale 1ns/1ps

module tb_top;

reg clk = 0;
wire [7:0] led;

always #5 clk = ~clk;

top dut (
    .clk(clk),
    .led(led)
);

integer cycle;

initial begin
    $dumpfile("sim/wave.vcd");
    $dumpvars(0, tb_top);

    for (cycle = 0; cycle < 1600; cycle = cycle + 1) begin
        @(posedge clk);
        #1;
        $display("cyc=%0d resetn=%b state=%s mem_valid=%b mem_instr=%b mem_ready=%b addr=%h wdata=%h wstrb=%b rdata=%h trap=%b",
            cycle,
            dut.resetn,
            dut.cpu.dbg_ascii_state,
            dut.mem_valid,
            dut.mem_instr,
            dut.mem_ready,
            dut.mem_addr,
            dut.mem_wdata,
            dut.mem_wstrb,
            dut.mem_rdata,
            dut.trap
        );

        if (dut.trap) begin
            $display(">>> TRAP asserted at cycle %0d, mem_addr=%h", cycle, dut.mem_addr);
        end

        if (dut.led_reg) begin
            $display(">>> LED_REG went high at cycle %0d", cycle);
            $finish;
        end
    end

    $display(">>> Simulation ended without LED going on. led_reg=%b trap=%b", dut.led_reg, dut.trap);
    $finish;
end

endmodule
