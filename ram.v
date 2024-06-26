module RAM_8x4bit (
    input wire clk,         
    input wire read,
    input wire write,    
    input wire [3:0] addr,  
    input wire [7:0] data_in
,  output reg [7:0] data_out  
);

// B[3]=>CLA  B[2]=>CMA B[1]=>CIR B[0]=>CIL

// 8x4-bit memory matrix==> arr[16][8]
reg [7:0] ram [0:15];  // 16 memory locations and one location 8 bit
    // Initialize memory array with desired values
    initial begin
    // Initialize memory array with desired values                  
    ram[0]  = 8'h0C;  // AND => 0 000 1100 (12)
    ram[1]  = 8'h1A;  // ADD => 0 001 1010 (10) 
    ram[2]  = 8'h26;  // LDA => 0 010 0110 (6)
    ram[3]  = 8'h78;  //CLA  => 0 111 1000  // register reference   
    ram[4]  = 8'h74;  //CMA  => 0 111 0100
    ram[5]  = 8'h72;  //CIR  => 0 111 0010
    ram[6]  = 8'h71;  //CIR  => 0 111 0001
    ram[7]  = 8'h07;
    ram[8]  = 8'h08;
    ram[9]  = 8'h09;
    ram[10] = 8'h1B; 
    ram[11] = 8'h0B;
    ram[12] = 8'h1C;
    ram[13] = 8'h0D;
    ram[14] = 8'h0E; 
    ram[15] = 8'h0F;
end


   always @(*) begin
    if (read)
        data_out = ram[addr]; // Read data from memory
    else if (write)
        ram[addr] = data_in;  // Write data to memory 
end

endmodule



