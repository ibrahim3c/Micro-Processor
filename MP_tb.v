`include "MP.v"

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
        #100;
        $finish; // End simulation
    end

endmodule
