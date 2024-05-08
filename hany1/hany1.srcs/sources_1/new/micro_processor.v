module AC_Reg(
    input INR,LD,clk,CLR
    ,input[7:0] in
    ,output reg [7:0] out
);
initial begin
    out=8'h11;
end

    always @(posedge clk ) begin
         if(CLR) out<=8'b0;
         if(LD)  out <=in;
         if(INR)  out<=out+1;
    end
endmodule



module AR_Reg(
    input INR,LD,clk/*,CLR*/
    ,input[3:0] in
    ,output reg [3:0] out
) ;
initial begin
    out=8'h0;
end


    always @(posedge clk ) begin
        // if(CLR) out<=4'b0;
         if(LD)  out <=in;
         if(INR)  out<=out+1;
    end


endmodule



module DR_Reg(
    input INR,LD,clk/*,CLR*/
    ,input[7:0] in
    ,output reg [7:0] out
) ;

initial begin
    out=8'h0;
end
    always @(posedge clk ) begin
        // if(CLR) out<=8'b0;
         if(LD)  out <=in;
         if(INR)  out<=out+1;
    end

endmodule


module IR_Reg(
    input LD,clk
    ,input[7:0] in
    ,output reg [7:0] out
) ;

initial begin
    out=8'h0;
end

    always @(posedge clk ) begin
        if(LD)  out <=in;
    end

endmodule
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

module Decoder3x8 (
    input wire [2:0] A,
    output reg [7:0] Y
);

always @*
begin
    case(A)
        3'b000: Y = 8'b00000001;
        3'b001: Y = 8'b00000010;
        3'b010: Y = 8'b00000100;
        3'b011: Y = 8'b00001000;
        3'b100: Y = 8'b00010000;
        3'b101: Y = 8'b00100000;
        3'b110: Y = 8'b01000000;
        3'b111: Y = 8'b10000000;
        default: Y = 8'b00000000;
    endcase
end

endmodule


module ControlUnit (
    input [7:0] T, input [7:0] D ,input I, input [7:0] B,
    output  LDAC, CLRAC, INRAC,LDAR,INRAR,/*CLRAR,*/ReadRam,WriteRam,LDDR,INRDR,LDIR,INRPC,CLRSC,LDPC
    ,output [0:2] s,
    output AND,ADD,LDA,STA,BUN,BSA,ISZ,CMA,CLA,CIL,CIR //Instruction singals
);


// AC
AC_ControlSingal ac (
    .T(T),
    .D(D),
    .I(I),
    .B(B),
    .LD(LDAC),
    .CLR(CLRAC),
    .INR(INRAC)
);

//AR
AR_ControlSingal ar (LDAR/*, CLR*/ , INRAR, T, D, I);

 //DR
 DR_ControlSingal dr (
    LDDR, INRDR,T, D
);
 //IR

IR_ControlSingal  ir (
    LDIR, T
);
 //PC
 PC_ControlSingal  pc (
    LDPC, D, T,INRPC
);

//RAM
 RAM_Control_Signal ram (
	.I(I),
	.T(T), 
    .D(D),
	.Write(WriteRam)
    ,.Read(ReadRam)
);

// SC

SC_ControlSignal  sc (
   T, D, I,CLRSC
);

// controlBus
wire [7:0]x;
CommonBus_ControlSignal commanBus(
.x(x), 
.D(D),
.T(T),
.I(I)
);
Selections sss(
    .x(x),
    .s(s)
);

Instructions_ControlSignal instructionControlSignal (
 T, D, I,B
,AND,ADD,LDA,STA,BUN,BSA,ISZ,CMA,CLA,CIL,CIR
);

endmodule



//  AC_Control

// B[3]=>CLA
// B[2]=>CMA
// B[1]=>CIR
// B[0]=>CIL

module AC_ControlSingal (
    input [7:0] T, input [7:0] D ,input I, input [7:0] B,
    output  LD,CLR,INR
);
wire  r;
assign r=D[7] & !I & T[3];
assign LD= (T[5]  & (D[0]|D[1]|D[2]) ) |  ( r &  ( B[0]|B[1]|B[2]));
assign CLR = r & B[3];

endmodule



//   AR_Control
module AR_ControlSingal(LD , INR, T, D, I);

	input  I;
	input [7:0] T, D;
	output LD, INR;
	
	wire D7n, Rn;
	wire a1, a2, a3;
	
	assign D7n = ~D[7];
	
	assign INR = D[5] & T[4]; //BSA 
	assign LD = (D7n & I & T[3]) | T[2] |T[0] ;
	
endmodule


//   DR_Control

module DR_ControlSingal (
    Load, INR, T, D
);

input [7:0] T,D;
	output Load, INR;

	 assign INR = D[6] & T[5];  // ISZ
	assign Load = T[4] & (D[0] | D[1] | D[2] | D[6]);

endmodule

// IR_Control
module IR_ControlSingal (
    load, T
);
input [7:0] T;

output load;

assign load=T[1];


endmodule


// AR => 8 bit =>( one bit for J,3 bits for opCode,4 bits for add)
// B[3]=>CLA
// B[2]=>CMA
// B[1]=>CIR
// B[0]=>CIL



// PC_Control

// AND,ADD, LDA, CMA,CIR,CIL
module PC_ControlSingal (
    LD, /*CLR,  I,*/ D, T,INR
);
// input  I;
input [7:0] T, D;
output LD, /*CLR,*/ INR;
assign LD=(D[4] & T[4])|(D[5] & T[5]);
assign INR=(T[1])|(D[6] & T[6]);

endmodule


//CommonBus_Control

module CommonBus_ControlSignal(x, D, T, I);

	input  I;
	input [7:0] T, D;
	output [7:0] x;	

	assign x[0]=0;
	assign x[1] = (D[4] & T[4]) | (D[5] & T[5]);  //AR 
	assign x[2] = (D[5] & T[4]) | (T[0]);  //PC 
	assign x[3] = T[6] & D[6];     //DR 
	assign x[4] = D[3] & T[4];     //AC 
	assign x[5] = T[2] ;           //IR 
    assign x[6]=0;          // TR
	assign x[7] = (T[1]) |(D[1]&T[4]) |(D[2]&T[4])|(D[0]&T[4])|(D[6]&T[4]);  // M[AR] 	
	
endmodule


module Selections (
	input [7:0]x,
	output[2:0]s
);

assign s[0]=x[1] | x[3] | x[5] | x[7];
assign s[1]=x[2] | x[3] | x[6] | x[7];
assign s[2]=x[4] | x[5] | x[6] | x[7];

endmodule





// ram_control

module RAM_Control_Signal(

	input  I,
	input [7:0] T, D,
	output wire Write, Read 
);
    assign Write = /* (T[1]) | */ (D[3] & T[4]) | (D[5] & T[4]) | (D[6] & T[6]);   // remove interrupt
	assign Read = (T[1]) |(D[1]&T[4]) |(D[2]&T[4])|(D[0]&T[4])|(D[6]&T[4]);

endmodule


// rest of instruction_control

module Instructions_ControlSignal (
 T, D, I,B
,AND,ADD,LDA,STA,BUN,BSA,ISZ,CMA,CLA,CIL,CIR
);

input  I;
input [7:0] T, D ;
input [7:0] B;
output AND,ADD,LDA,STA,BUN,BSA,ISZ,CMA,CLA,CIL,CIR ;

// Memory-Reference
assign AND= D[0] & T[5];
assign ADD= D[1] & T[5];
assign LDA= D[2] & T[5];
assign STA= D[3] & T[4];
assign BUN= D[4] & T[4];
assign BSA= D[5] & T[4];
assign ISZ= D[6] & T[6];

wire  r;
assign r=D[7] & !I & T[3];
// Register-Reference
assign CLA= r & B[3];
assign CMA= r & B[2];
assign CIR= r & B[1];
assign CIL= r & B[0];



endmodule


// sc_Control
module SC_ControlSignal (
   
   T, D, I,CLR
);
input  I;
input [7:0] T, D;
output  CLR;
wire  r;
assign r=D[7] & !I & T[3];
assign CLR=(D[0]&T[5]) | (D[1]&T[5]) | (D[2]&T[5]) | r |(D[4]&T[4]) | (D[3]&T[4])  | (D[5]&T[5]) |(D[6]&T[6]);    
endmodule


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



module BUS_SEL (
    input [7:0] DR, AC, IR, RAM, 
    input [3:0] AR, PC, 
    input [2:0] S,                       
    output  [7:0] OUT                   
);

    // wire [7:0] mux_out;  

    // Multiplexer module for selecting bits
    MUX_8to1 mux_8to1 (
        .d1(AR),
        .d2(PC),
        .d3(DR),
        .d4(AC),
        .d5(IR),
        .d7(RAM),
        .d6(8'h0),
        .d0(8'h0),
        .sel(S),
        .out(OUT)
    );

    // always @* begin
    //     case (S)
    //         3'b001: OUT = AR;  // Select AR
    //         3'b001: OUT = PC;  // Select PC
    //         3'b010: OUT = DR;  // Select DR
    //         3'b011: OUT = AC;  // Select AC
    //         3'b100: OUT = IR;  // Select IR
    //         3'b101: OUT = RAM; // Select RAM
    //         default: OUT = 8'hff;  // Default 
    //     endcase
    // end

endmodule




module MUX_8to1 (
    input [7:0]  d0, d3, d4, d5,d6,d7, // 8-bit input data
    input [2:0] sel,
    input [3:0]d2, d1,                  // Selection input
    output reg [7:0] out                   // Output data
);

    always @* begin
        case (sel)
            3'b000:out=d0;
			3'b001:out=d1;
			3'b010:out=d2;
			3'b011:out=d3;
			3'b100:out=d4;
			3'b101:out=d5;
			3'b110:out=d6;
			3'b111:out=d7;
        endcase
    end

endmodule


module Micro_Computer (
     clk,sc,AC, DR,IR,ram,PC, AR,E
);
input wire clk;

// Signals for control unit
wire [7:0] T, D, B;
wire LDAC, CLRAC, INRAC, LDAR, INRAR, LDDR, INRDR, LDIR, INRPC, CLRSC,LDPC,read,write;
wire [2:0]s;
wire  AND, ADD, LDA, STA, BUN, BSA, ISZ, CMA, CLA, CIL, CIR;

// Signals for adder and logic unit and CB 
wire [2:0] count;
wire [7:0]ACData,OUT;

//ouptut
output  [7:0] AC, DR,IR,ram;
output  [3:0] PC, AR;
reg cin;
output  E;
output  [7:0] sc;



//wires to link the modules
wire[7:0] ac_wire,dr_wire,ir_wire,ram_wire;
wire e_wire;
wire[3:0] pc_wire,ar_wire;

initial begin
    cin=0;
end


assign AC=ac_wire;
assign DR=dr_wire;
assign IR=ir_wire;
assign ram=ram_wire;
assign PC=pc_wire;
assign AR=ar_wire;
assign E=e_wire;
assign  sc=T;




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



// Output register assignments
    // always @(*) begin
    //     AC <= ac_wire;
    //     DR <= dr_wire;
    //     IR <= ir_wire;
    //     ram <= ram_wire;
    //     PC <= pc_wire;
    //     AR <= ar_wire;
    //     E <= e_wire;
    //     sc<=T;
    // end





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


`timescale 1ns / 1ps

module Micro_Computer_tb;

    // Inputs
    reg clk;

    // Outputs
    wire [7:0] AC_out;
    wire [7:0] DR_out;
    wire [7:0] IR_out;
    wire [7:0] ram_out;
    wire [3:0] PC_out;
    wire [3:0] AR_out;
    wire E_out;
    wire [7:0] sc_out;

    // Instantiate the Micro_Computer module
    Micro_Computer uut (
        .clk(clk),
        .sc(sc_out),
        .AC(AC_out),
        .DR(DR_out),
        .IR(IR_out),
        .ram(ram_out),
        .PC(PC_out),
        .AR(AR_out),
        .E(E_out)
        
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // Toggle clock every 5 time units
    end

    // Monitor outputs
    always @(posedge clk) begin
        $display("Time: %0t,T:%h, AC: %h, DR: %h, IR: %h, RAM: %h, PC: %h, AR: %h, E: %b", $time,sc_out, AC_out, DR_out, IR_out, ram_out, PC_out, AR_out, E_out);
    end

    // Initial values
    initial begin
        $dumpfile("Micro_Computer.vcd");
        $dumpvars(0, Micro_Computer_tb);
        
        // Run simulation for 100 time units
        #1000;
        $finish; // End simulation
    end

endmodule
