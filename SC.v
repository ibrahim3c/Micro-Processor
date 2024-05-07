module Sequence_Counter3Bit2(
    input clk, reset,
    output reg [2:0] count
);

initial begin
    count=3'h0;
end
    always @(posedge clk) begin
        if (reset)
            count <= 3'h0;
        else 
            count <= count + 1;
    end
endmodule