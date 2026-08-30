`timescale 1ns/1ps

module tb_pnr;

reg clk = 0;
always #5 clk = ~clk;

// pin mapping (from constraints/alchitry_cu.pcf via icestorm chipdb-8k.txt):
//   clk    P7  -> io_16_0_1
//   led[0] J11 -> io_33_6_0
//   led[1] K11 -> io_33_4_1
//   led[2] K12 -> io_33_4_0
//   led[3] K14 -> io_33_5_0
//   led[4] L12 -> io_33_2_0
//   led[5] L14 -> io_33_3_1
//   led[6] M12 -> io_33_1_0
//   led[7] N14 -> io_33_2_1

wire led0, led1, led2, led3, led4, led5, led6, led7;

chip dut (
    .io_16_0_1(clk),
    .io_33_4_1(led1),
    .io_33_6_0(led0),
    .io_33_3_1(led5),
    .io_33_2_0(led4),
    .io_33_1_0(led6),
    .io_33_2_1(led7),
    .io_33_4_0(led2),
    .io_33_5_0(led3)
);

integer i;

initial begin
    $dumpfile("sim/wave_pnr.vcd");
    $dumpvars(0, tb_pnr);

    for (i = 0; i < 4000; i = i + 1) begin
        @(posedge clk);
        if (led0) begin
            $display(">>> LED0 (physical J11, PNR netlist) went HIGH at raw-clk edge %0d (t=%0t)", i, $time);
            $display("    led1(resetn)=%b led2(saw_mem_req)=%b led3(saw_instr_fetch)=%b led4(saw_mmio_write)=%b led5(trap)=%b",
                led1, led2, led3, led4, led5);
            $finish;
        end
    end

    $display(">>> TIMEOUT: LED0 never went high.");
    $display("    led1(resetn)=%b led2(saw_mem_req)=%b led3(saw_instr_fetch)=%b led4(saw_mmio_write)=%b led5(trap)=%b",
        led1, led2, led3, led4, led5);
    $finish;
end

endmodule
