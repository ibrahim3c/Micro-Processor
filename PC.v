module PC_Reg(
    input INR,clk,LD
    ,input[3:0] in
    ,output reg [3:0] out
) ;


initial begin
    out=4'h0;
end

    always @(posedge clk ) begin
        // if(CLR) out<=4'b0;
        if(LD)  out <=in;
         if(INR)  out<=out+1;
    end


endmodule


