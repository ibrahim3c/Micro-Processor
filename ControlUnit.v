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
