module uart_rx #(
    parameter CLK_FREQ=50000000,
    parameter BAUD=115200
)(
    input wire clk,
    input wire resetn,
    input wire rx,
    output reg [7:0] data=0,
    output reg valid=0
);

localparam CLKS_PER_BIT=CLK_FREQ/BAUD;
localparam HALF_BIT=CLKS_PER_BIT/2;

//rx comes in async off the ftdi pin so sync it into our clock domain first
reg rx_sync0=1;
reg rx_sync1=1;

always @(posedge clk)
begin
    rx_sync0<=rx;
    rx_sync1<=rx_sync0;
end

reg busy=0;
reg [15:0] clk_count=0;
reg [3:0] bit_index=0; //0=start bit, 1-8=data lsb first, 9=stop bit
reg [7:0] shift_reg=0;

wire [15:0] bit_target=(bit_index==0) ? (HALF_BIT-1) : (CLKS_PER_BIT-1);

always @(posedge clk)
begin
    valid<=0;

    if(!resetn)
    begin
        busy<=0;
        clk_count<=0;
        bit_index<=0;
    end
    else if(!busy)
    begin
        if(!rx_sync1)
        begin
            busy<=1;
            clk_count<=0;
            bit_index<=0;
        end
    end
    else if(clk_count==bit_target)
    begin
        clk_count<=0;

        if(bit_index==0)
        begin
            //still centered on the start bit, if the line already went
            //back high this was just noise not a real frame
            if(rx_sync1)
                busy<=0;
            else
                bit_index<=1;
        end
        else if(bit_index<=8)
        begin
            shift_reg<={rx_sync1,shift_reg[7:1]};
            bit_index<=bit_index+1;
        end
        else
        begin
            busy<=0;

            if(rx_sync1) //stop bit must be high or the frame is junk
            begin
                data<=shift_reg;
                valid<=1;
            end
        end
    end
    else
    begin
        clk_count<=clk_count+1;
    end
end

endmodule
