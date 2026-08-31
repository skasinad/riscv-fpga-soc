`timescale 1ns/1ps

//gate level check against the actual routed bitstream (via icebox_vlog),
//not just rtl sim. rtl sim is zero delay and can look fine even when pnr
//misses timing, this is how we caught the 100mhz vs ~74mhz fmax problem

module tb_pnr;

reg clk = 0;
always #5 clk = ~clk;

//pin mapping (from constraints/alchitry_cu.pcf via icestorm chipdb-8k.txt):
//   clk    P7  -> io_16_0_1
//   led[0] J11 -> io_33_6_0  (gpio_out[0])
//   led[1] K11 -> io_33_4_1  (gpio_out[1])
//   led[2] K12 -> io_33_4_0  (gpio_out[2])
//   led[3] K14 -> io_33_5_0  (gpio_out[3])
//   led[4] L12 -> io_33_2_0  (gpio_out[4])
//   led[5] L14 -> io_33_3_1  (gpio_out[5])
//   led[6] M12 -> io_33_1_0  (gpio_out[6])
//   led[7] N14 -> io_33_2_1  (trap)

wire led0, led1, led2, led3, led4, led5, led6, led7;

chip dut (
    .io_16_0_1(clk),
    .io_33_6_0(led0),
    .io_33_4_1(led1),
    .io_33_4_0(led2),
    .io_33_5_0(led3),
    .io_33_2_0(led4),
    .io_33_3_1(led5),
    .io_33_1_0(led6),
    .io_33_2_1(led7)
);

localparam RAW_CLK_LIMIT = 12_000_000;

reg [7:0] gpio_led;
reg [7:0] last_gpio;
integer i;

always @* gpio_led = {led6, led5, led4, led3, led2, led1, led0};

initial begin
    $dumpfile("sim/wave_pnr.vcd");
    $dumpvars(0, tb_pnr);

    last_gpio = 8'hFF;

    for (i = 0; i < RAW_CLK_LIMIT; i = i + 1) begin
        @(posedge clk);

        if (gpio_led !== last_gpio) begin
            $display("t=%0t gpio_out=%02h trap(led7)=%b", $time, gpio_led, led7);
            last_gpio = gpio_led;
        end

        if (led7) begin
            $display(">>> TRAP (led7, PNR netlist) asserted at t=%0t", $time);
            $finish;
        end

        if (gpio_led == 8'h3F) begin
            $display(">>> POST_PASS (gpio_out=0x3F, PNR netlist) at t=%0t", $time);
            $finish;
        end

        if (gpio_led[6]) begin
            $display(">>> POST fault (PNR netlist), gpio_out=%02h at t=%0t", gpio_led, $time);
            $finish;
        end
    end

    $display(">>> TIMEOUT after %0d raw clk edges. gpio_out=%02h trap=%b", RAW_CLK_LIMIT, gpio_led, led7);
    $finish;
end

endmodule
