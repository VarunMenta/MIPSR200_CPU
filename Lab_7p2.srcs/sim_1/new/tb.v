`timescale 1ns / 1ps
// You can use this skeleton testbench code, the textbook testbench code, or your own
module MIPS_Testbench ();
  reg CLK;
  reg RST;
  reg [2:0] switches;
  reg br1;
  wire [6:0] segs;
  wire [3:0] an;
//  wire CS;
//  wire WE;
//  wire [31:0] Mem_Bus;
//  wire [6:0] Address;

  initial
  begin
    CLK = 0;
    switches = 3'b000;
  end

//  MIPS CPU(CLK, RST, CS, WE, Address, Mem_Bus);
//  Memory MEM(CS, WE, CLK, Address, Mem_Bus);

Complete_MIPS a1(CLK, RST, switches, br1,segs,an);

  always
  begin
    #5 CLK = !CLK;
  end

  always
  begin
    RST <= 1'b1; //reset the processor

    //Notice that the memory is initialize in the in the memory module not here

    @(posedge CLK);
    // driving reset low here puts processor in normal operating mode
    RST <= 1'b0;
    #200
//    switches = 3'b001;
//    #350
//    switches = 3'b000;
//    #350
//    switches = 3'b010;
//    #350
//    switches = 3'b011;
//    #350
    switches = 3'b100;
    #350
    switches = 3'b101;
    #350

    /* add your testing code here */
    // you can add in a 'Halt' signal here as well to test Halt operation
    // you will be verifying your program operation using the
    // waveform viewer and/or self-checking operations

    $display("TEST COMPLETE");
    $stop;
  end

endmodule

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

