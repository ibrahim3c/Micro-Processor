module AdderAndLogic4 (
    input AND, ADD, LDA, CMA, CIR, CIL,    // Instructions signals 
    input [7:0] AC, DR,
    input CIN,
    output reg E,
    output reg [7:0] ACData
);
initial
begin
    E=1;
end;
    
always @* begin
    // AND OPERATION
    if (AND) begin
        ACData = AC & DR;
    end

    // ADD OPERATION
    else if (ADD) begin
        {E,ACData}=AC+DR+CIN;
    end

    // LDA OPERATION (DR to AC)
    else if (LDA) begin
        ACData = DR;
    end

    // CMA OPERATION (Complement AC)
    else if (CMA) begin
        ACData = ~AC;
    end

    // CIR (Circulate Right) OPERATION
    else if (CIR) begin
       ACData= AC >> 1 | (E ? 8'b10000000 : 8'b0);
       E=AC[0];
    end

    // CIL (Circulate Left) OPERATION
    else if (CIL) begin
        ACData =(AC << 1 | {7'b0, E});
        E = ACData[7];
    end

    // Default assignment if no operation is selected
    else begin
        ACData = AC;
    end
end
endmodule
