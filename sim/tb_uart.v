`timescale 1ns/1ps

// Receives usb_tx in real sim time rather than counting sys_clk edges --
// bit period is an exact multiple of sys_clk (434 cycles @ 50 MHz, no
// fractional drift), so fixed-delay sampling stays aligned across all 6 bytes.
module tb_uart;

localparam integer BIT_NS = 8680; // 1 / 115200 baud @ CLKS_PER_BIT=434, sys_clk=50MHz

reg clk = 0;
wire [7:0] led;
wire usb_tx;

always #5 clk = ~clk;

top dut (
    .clk(clk),
    .led(led),
    .usb_tx(usb_tx)
);

reg [7:0] expected [0:5];
reg [7:0] got;
integer i, bitn, errors;

initial begin
    expected[0]=8'h52; expected[1]=8'h56; expected[2]=8'h33; // R V 3
    expected[3]=8'h32; expected[4]=8'h0D; expected[5]=8'h0A; // 2 \r \n

    errors = 0;

    for (i = 0; i < 6; i = i + 1) begin
        @(negedge usb_tx);              // start bit edge
        #(BIT_NS + BIT_NS/2);           // center of data bit 0

        got = 0;
        for (bitn = 0; bitn < 8; bitn = bitn + 1) begin
            got[bitn] = usb_tx;         // LSB first
            #(BIT_NS);
        end

        if (usb_tx !== 1'b1) begin
            $display(">>> FRAMING ERROR on byte %0d: stop bit = %b", i, usb_tx);
            errors = errors + 1;
        end

        $display("byte %0d: got=%02h expected=%02h %s", i, got, expected[i],
            (got === expected[i]) ? "OK" : "MISMATCH");

        if (got !== expected[i])
            errors = errors + 1;
    end

    if (errors == 0)
        $display(">>> UART TX PASS");
    else
        $display(">>> UART TX FAIL: %0d error(s)", errors);

    $finish;
end

initial begin
    #2_000_000;
    $display(">>> TIMEOUT waiting for UART bytes");
    $finish;
end

endmodule
