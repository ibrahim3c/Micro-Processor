`include "SC.v"
`include "AC.v"
`include "AR.v"
`include "DR.v"
`include "IR.v"
`include "PC.v"
`include "ram.v"
`include "CommonBus.v"
`include "ControlUnit.v"
`include "Adder&Logic.v"
`include "Decoder.v"

module Micro_Computer (
     clk,sc,AC, DR,IR,ram,PC, AR,E
);
input wire clk;

// Signals for control unit
wire [7:0] T, D, B;
wire LDAC, CLRAC, INRAC, LDAR, INRAR, LDDR, INRDR, LDIR, INRPC, CLRSC,LDPC,read,write;
wire [2:0]s;
wire  AND, ADD, LDA, STA, BUN, BSA, ISZ, CMA, CLA, CIL, CIR;
wire [2:0] count;
wire [7:0]ACData,OUT;

// Signals for adder and logic unit
output reg [7:0] AC, DR,IR,ram;
output reg [3:0] PC, AR;
reg cin;
output reg E;
output reg [7:0] sc;


wire[7:0] ac_wire,dr_wire,ir_wire,ram_wire;
wire e_wire;
wire[3:0] pc_wire,ar_wire;

initial begin
    cin=0;
end


// initial begin
//     AC = 8'h00; // Initial value for ac
//     DR = 8'h00; // Initial value for dr
//     IR = 8'h00; // Initial value for ir
//     PC = 4'h0;  // Initial value for pc
//     AR = 4'h0;  // Initial value for ar
//     cin=0;
//     sc=0;
//     ram=0;
// end
// assign AC=ac_wire;
// assign DR=dr_wire;
// assign IR=ir_wire;
// assign ram=ram_wire;
// assign pc=pc_wire;
// assign AR=ar_wire;
// assign E=e_wire;



// Output register assignments
    always @(*) begin
        AC <= ac_wire;
        DR <= dr_wire;
        IR <= ir_wire;
        ram <= ram_wire;
        PC <= pc_wire;
        AR <= ar_wire;
        E <= e_wire;
        sc<=T;
    end





// sequence Counter
Sequence_Counter3Bit2 counter (
    .clk(clk),
    .reset(CLRSC),
    .count(count)
);

Decoder3x8 decoder (
     .A(count),
     .Y(T)
);


IR_Reg ir (
    .LD(LDIR),
    .clk(clk),
    .in(OUT),
    .out(ir_wire)
);


// to get D

Decoder3x8 d2(
.A(ir_wire[6:4]), 
.Y(D)

);


// CU
ControlUnit CU (
    .T(T),
    .D(D),
    .I(ir_wire[7]),
    .B(ir_wire),
    .LDAC(LDAC),
    .CLRAC(CLRAC),
    .INRAC(INRAC),
    .LDAR(LDAR),
    .INRAR(INRAR),
    .ReadRam(read),
    .WriteRam(write),
    .LDDR(LDDR),
    .INRDR(INRDR),
    .LDIR(LDIR),
    .INRPC(INRPC),
    .CLRSC(CLRSC),
    .LDPC(LDPC),
    .s(s),
    .AND(AND),
    .ADD(ADD),
    .LDA(LDA),
    .STA(STA),
    .BUN(BUN),
    .BSA(BSA),
    .ISZ(ISZ),
    .CMA(CMA),
    .CLA(CLA),
    .CIL(CIL),
    .CIR(CIR)
);



// common bus
BUS_SEL CommonBus (
    .AR(ar_wire),
    .PC(pc_wire),
    .DR(dr_wire),
    .AC(ac_wire),
    .IR(ir_wire),
    .RAM(ram_wire),
    .S(s),
    .OUT(OUT)
);


// registers
AC_Reg  ac (
    .INR(INRAC),
    .LD(LDAC),
    .clk(clk),
    .CLR(CLRAC),
    .in(ACData),
    .out(ac_wire)
);

AR_Reg ar (
    .INR(INRAR),
    .LD(LDAR),
    .clk(clk),
    .in(OUT[3:0]),
    .out(ar_wire)
);


// Adder&Logic
AdderAndLogic4 U1 (
  .AND(AND),  
  .ADD(ADD),  
  .LDA(LDA), 
  .CMA(CMA),  
  .CIR(CIR),  
  .CIL(CIL),  
  .AC(ac_wire),  
  .DR(dr_wire),       
  .CIN(cin),     
  .E(e_wire),    
  .ACData(ACData)         
);
   

DR_Reg dr (
    .INR(INRDR),
    .LD(LDDR),
    .clk(clk),
    // .CLR(CLRDR),
    .in(OUT),
    .out(dr_wire)
);



PC_Reg pc (
    .INR(INRPC),
    .LD(LDPC),
    .clk(clk),
    .in(OUT[3:0]),
    .out(pc_wire)
);

// Memory
RAM_8x4bit RAM (
    .clk(clk),
    .read(read),
    .write(write),
    .addr(ar_wire),
    .data_in(OUT)
    ,.data_out(ram_wire)
);




endmodule



