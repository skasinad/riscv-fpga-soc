`timescale 1ns/1ps

module tb_top;

reg clk = 0;
wire [7:0] led;

always #5 clk = ~clk;

top dut (
    .clk(clk),
    .led(led)
);

//firmware runs the full post sequence, ends with a ~5,000,000 sys_clk
//delay before pass, so give it enough margin to not time out early
localparam RAW_CLK_LIMIT = 12_000_000;

reg [7:0] last_gpio;
integer i;

initial begin
    $dumpfile("sim/wave.vcd");
    $dumpvars(0, tb_top);

    last_gpio = 8'hFF; //force a print on the first sample

    for (i = 0; i < RAW_CLK_LIMIT; i = i + 1) begin
        @(posedge clk);

        if (dut.gpio_out !== last_gpio) begin
            $display("t=%0t gpio_out=%02h trap=%b", $time, dut.gpio_out, dut.trap);
            last_gpio = dut.gpio_out;
        end

        if (dut.trap) begin
            $display(">>> TRAP asserted at t=%0t, mem_addr=%h", $time, dut.mem_addr);
            $finish;
        end

        if (dut.gpio_out == 8'h3F) begin
            $display(">>> POST_PASS (gpio_out=0x3F) at t=%0t", $time);
            $display("    instr_count=%0d mem_access_count=%0d mmio_access_count=%0d",
                dut.instr_count, dut.mem_access_count, dut.mmio_access_count);
            if (dut.instr_count == 0 || dut.mem_access_count == 0 || dut.mmio_access_count == 0)
                $display(">>> WARNING: POST passed but an execution-monitor counter read 0 in sim");
            $finish;
        end

        if (dut.gpio_out[6]) begin
            $display(">>> POST fault, gpio_out=%02h at t=%0t", dut.gpio_out, $time);
            $display("    instr_count=%0d mem_access_count=%0d mmio_access_count=%0d",
                dut.instr_count, dut.mem_access_count, dut.mmio_access_count);
            $finish;
        end
    end

    $display(">>> TIMEOUT after %0d raw clk edges. gpio_out=%02h trap=%b", RAW_CLK_LIMIT, dut.gpio_out, dut.trap);
    $finish;
end

endmodule
