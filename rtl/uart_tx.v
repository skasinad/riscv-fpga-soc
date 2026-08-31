module uart_tx #(
    parameter CLK_FREQ=50000000,
    parameter BAUD=115200
)(
    input wire clk,
    input wire resetn,
    input wire start,
    input wire [7:0] data,
    output reg tx=1,
    output reg busy=0
);

localparam CLKS_PER_BIT=CLK_FREQ/BAUD;

reg [15:0] clk_count=0;
reg [3:0] bit_index=0;
reg[9:0 ] shift_reg=10'h3FF;

always @(posedge clk)
begin
    if(!resetn)
    begin
        tx<=1;
        busy<=0;
        clk_count<=0;
        bit_index<=0;
        shift_reg<=10'h3FF;
    end
    else
    begin
        if(start&&!busy)
        begin
            shift_reg<={1'b1,data,1'b0};
            busy<=1;
            clk_count<=0;
            bit_index<=0;
            tx<=0;
        end
        else if(busy)
        begin
            if(clk_count==CLKS_PER_BIT-1)
            begin
                clk_count<=0;

                if(bit_index==9)
                begin
                    busy<=0;
                    tx<=1;
                end
                else
                begin
                    bit_index<=bit_index+1;
                    shift_reg<=shift_reg>>1;
                    tx<=shift_reg[1];
                end
            end
            else
            begin
                clk_count<=clk_count+1;
            end
        end
    end
end

endmodule