`timescale 1ns / 1ps
module Complete_MIPS(CLK, RST, switches, br1, segs, an);
  // Will need to be modified to add functionality
  input CLK;
  input RST;
  input [2:0] switches;
  input br1;
  
  output[6:0] segs;
  output[3:0] an;

  wire deb_br1;
  debouncer button1(br1, CLK, deb_br1);
  
  wire CS, WE;
  wire [6:0] ADDR;
  wire [31:0] Mem_Bus;
  wire [31:0] Reg2Out;

  MIPS CPU(CLK, RST, CS, WE, ADDR, Mem_Bus, switches, Reg2Out);
  Memory MEM(CS, WE, CLK, ADDR, Mem_Bus);
  
  reg [3:0] b3 = 0;
  reg [3:0] b2 = 0;
  reg [3:0] b1 = 0;
  reg [3:0] b0 = 0;
  
  always @ (posedge CLK) 
  begin
    if(deb_br1)begin
        b3 = Reg2Out[31:28];
        b2 = Reg2Out[27:24];
        b1 = Reg2Out[23:20];
        b0 = Reg2Out[19:16];
    end
    else begin
        b3 = Reg2Out[15:12];
        b2 = Reg2Out[11:8];
        b1 = Reg2Out[7:4];
        b0 = Reg2Out[3:0];
    end    
  end
  
  wire [6:0] c3,c2,c1,c0;
  hexto7segment h3(.x(b3),.r(c3));
  hexto7segment h2(.x(b2),.r(c2));
  hexto7segment h1(.x(b1),.r(c1));
  hexto7segment h0(.x(b0),.r(c0));
  
  wire slow_clk;
  display_clk a7(.clk(CLK),.reset(RST),.slow_clk(slow_clk));
    
  wire dec;
  time_mux_state_machine a2(.clk(slow_clk),.reset(RST),.in0(c0),.in1(c1),.in2(c2),.in3(c3),.an(an),.sseg(segs),.dec(dec));
     

endmodule

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

module Memory(CS, WE, CLK, ADDR, Mem_Bus);
  input CS;
  input WE;
  input CLK;
  input [6:0] ADDR;
  inout [31:0] Mem_Bus;

  reg [31:0] data_out;
  reg [31:0] RAM [0:127];


  initial
  begin
    /* Write your Verilog-Text IO code here */
    $readmemb("lab7p2b.mem",RAM);
  end

  assign Mem_Bus = ((CS == 1'b0) || (WE == 1'b1)) ? 32'bZ : data_out;

  always @(negedge CLK)
  begin

    if((CS == 1'b1) && (WE == 1'b1))
      RAM[ADDR] <= Mem_Bus[31:0];

    data_out <= RAM[ADDR];
  end
endmodule

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

module REG(CLK, RegW, DR, SR1, SR2, Reg_In, ReadReg1, ReadReg2,Reg1In, Reg2Out);
  input CLK;
  input RegW;
  input [4:0] DR;
  input [4:0] SR1;
  input [4:0] SR2;
  input [31:0] Reg_In;
  output reg [31:0] ReadReg1;
  output reg [31:0] ReadReg2;
  input [2:0] Reg1In;
  output [31:0] Reg2Out;

  reg [31:0] REG [0:31];
  integer i;
  wire [31:0] Reg2 = REG[2];
  assign Reg2Out = Reg2;

  initial begin
    ReadReg1 = 0;
    ReadReg2 = 0;
  end

  always @(posedge CLK)
  begin

    if(RegW == 1'b1)
      REG[DR] <= Reg_In[31:0];

    ReadReg1 <= REG[SR1];
    ReadReg2 <= REG[SR2];
    REG[1] <= Reg1In;
  end
endmodule


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

`define opcode instr[31:26]
`define sr1 instr[25:21]
`define sr2 instr[20:16]
`define f_code instr[5:0]
`define numshift instr[10:6]

module MIPS (CLK, RST, CS, WE, ADDR, Mem_Bus, Reg1In, Reg2Out);
  input CLK, RST;
  output reg CS, WE;
  output [6:0] ADDR;
  inout [31:0] Mem_Bus;
  input [2:0] Reg1In;
  output [31:0] Reg2Out;
  
  parameter max_add = 32'b1111_1111_1111_1111_1111_1111_1111_1111;

  //special instructions (opcode == 000000), values of F code (bits 5-0):
  parameter add = 6'b100000;
  parameter sub = 6'b100010;
  parameter xor1 = 6'b100110;
  parameter and1 = 6'b100100;
  parameter or1 = 6'b100101;
  parameter slt = 6'b101010;
  parameter srl = 6'b000010;
  parameter sll = 6'b000000;
  parameter jr = 6'b001000;
  
  //Extended special instruction set F code
  parameter rbit = 6'b101111;
  parameter rev = 6'b110000;
  parameter add8 = 6'b101101;
  parameter sadd = 6'b110001;
  parameter ssub = 6'b110010;

  //non-special instructions, values of opcodes:
  parameter addi = 6'b001000;
  parameter andi = 6'b001100;
  parameter ori = 6'b001101;
  parameter lw = 6'b100011;
  parameter sw = 6'b101011;
  parameter beq = 6'b000100;
  parameter bne = 6'b000101;
  parameter j = 6'b000010;
  
  //Extended instructions set opcodes
  parameter jal = 6'b000011;
  parameter lui = 6'b001111;

  //instruction format
  parameter R = 2'd0;
  parameter I = 2'd1;
  parameter J = 2'd2;

  //internal signals
  reg [5:0] op, opsave;
  wire [1:0] format;
  reg [31:0] instr, alu_result;
  reg [6:0] pc, npc;
  wire [31:0] imm_ext, alu_in_A, alu_in_B, reg_in, readreg1, readreg2;
  reg [31:0] alu_result_save;
  reg alu_or_mem, alu_or_mem_save, regw, writing, reg_or_imm, reg_or_imm_save;
  reg fetchDorI;
  wire [4:0] dr;
  wire [34:0] saturate;
  reg [2:0] state, nstate;

  //combinational
  assign imm_ext = (instr[15] == 1)? {16'hFFFF, instr[15:0]} : {16'h0000, instr[15:0]};//Sign extend immediate field
  assign dr = (format == R) ? ((opsave == rbit || opsave == rev) ? instr[25:21]: instr[15:11]) : ((format == J) ? 5'b11111 :instr[20:16]); //Destination Register MUX (MUX1)
  assign alu_in_A = readreg1;
  assign alu_in_B = (reg_or_imm_save)? imm_ext : readreg2; //ALU MUX (MUX2)
  assign saturate = alu_in_A + alu_in_B;
  assign reg_in = (alu_or_mem_save)? Mem_Bus : alu_result_save; //Data MUX
  assign format = (`opcode == 6'd0)? R : (((`opcode == 6'd2)||(`opcode == 6'd3))? J : I);
  assign Mem_Bus = (writing)? readreg2 : 32'bZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ;

  //drive memory bus only during writes
  assign ADDR = (fetchDorI)? pc : alu_result_save[6:0]; //ADDR Mux
  REG Register(CLK, regw, dr, `sr1, `sr2, reg_in, readreg1, readreg2,Reg1In, Reg2Out);

  initial begin
    op = and1; opsave = and1;
    state = 3'b0; nstate = 3'b0;
    alu_or_mem = 0;
    regw = 0;
    fetchDorI = 0;
    writing = 0;
    reg_or_imm = 0; reg_or_imm_save = 0;
    alu_or_mem_save = 0;
  end
  always @(*)
  begin
    fetchDorI = 0; CS = 0; WE = 0; regw = 0; writing = 0; alu_result = 32'd0;
    npc = pc; op = jr; reg_or_imm = 0; alu_or_mem = 0; nstate = 3'd0;
    case (state)
      0: begin //fetch
        npc = pc + 7'd1; CS = 1; nstate = 3'd1;
        fetchDorI = 1;
      end
      1: begin //decode
        nstate = 3'd2; reg_or_imm = 0; alu_or_mem = 0;
        if (format == J) begin //jump, and finish
          if(`opcode == jal)
            nstate = 3'd2;
          else
            begin
            nstate = 3'd0;
            npc = instr[6:0];
            end
        end
        else if (format == R) //register instructions
          op = `f_code;
        else if (format == I) begin //immediate instructions
          reg_or_imm = 1;
          if(`opcode == lui)begin
            op = lui;
          end
          if(`opcode == lw) begin
            op = add;
            alu_or_mem = 1;
          end
          else if ((`opcode == lw)||(`opcode == sw)||(`opcode == addi)) op = add;
          else if ((`opcode == beq)||(`opcode == bne)) begin
            op = sub;
            reg_or_imm = 0;
          end
          else if (`opcode == andi) op = and1;
          else if (`opcode == ori) op = or1;
        end
      end
      2: begin //execute
        nstate = 3'd3;
        if (opsave == and1) alu_result = alu_in_A & alu_in_B;
        else if (opsave == or1) alu_result = alu_in_A | alu_in_B;
        else if (opsave == add) alu_result = alu_in_A + alu_in_B;
        else if (opsave == sub) alu_result = alu_in_A - alu_in_B;
        else if (opsave == srl) alu_result = alu_in_B >> `numshift;
        else if (opsave == sll) alu_result = alu_in_B << `numshift;
        else if (opsave == slt) alu_result = (alu_in_A < alu_in_B)? 32'd1 : 32'd0;
        else if (opsave == xor1) alu_result = alu_in_A ^ alu_in_B;
        else if (opsave == lui) alu_result = alu_in_B << 16;
        else if (opsave == rbit) begin
            begin
                alu_result[0] = alu_in_B[31];
                alu_result[1] = alu_in_B[30];
                alu_result[2] = alu_in_B[29];
                alu_result[3] = alu_in_B[28];
                alu_result[4] = alu_in_B[27];
                alu_result[5] = alu_in_B[26];
                alu_result[6] = alu_in_B[25];
                alu_result[7] = alu_in_B[24];
                alu_result[8] = alu_in_B[23];
                alu_result[9] = alu_in_B[22];
                alu_result[10] = alu_in_B[21];
                alu_result[11] = alu_in_B[20];
                alu_result[12] = alu_in_B[19];
                alu_result[13] = alu_in_B[18];
                alu_result[14] = alu_in_B[17];
                alu_result[15] = alu_in_B[16];
                alu_result[16] = alu_in_B[15];
                alu_result[17] = alu_in_B[14];
                alu_result[18] = alu_in_B[13];
                alu_result[19] = alu_in_B[12];
                alu_result[20] = alu_in_B[11];
                alu_result[21] = alu_in_B[10];
                alu_result[22] = alu_in_B[9];
                alu_result[23] = alu_in_B[8];
                alu_result[24] = alu_in_B[7];
                alu_result[25] = alu_in_B[6];
                alu_result[26] = alu_in_B[5];
                alu_result[27] = alu_in_B[4];
                alu_result[28] = alu_in_B[3];
                alu_result[29] = alu_in_B[2];
                alu_result[30] = alu_in_B[1];
                alu_result[31] = alu_in_B[0];
                
                
                
            end
        end
        else if (opsave == rev)begin
            alu_result[7:0] = alu_in_B[31:24];
            alu_result[15:8] = alu_in_B[23:16];
            alu_result[23:16] = alu_in_B[15:8];
            alu_result[31:24] = alu_in_B[7:0];
        end
        else if (opsave == sadd)begin
            if((saturate > max_add))
                alu_result = max_add;
            else
                alu_result = alu_in_A + alu_in_B;
        end
        else if (opsave == ssub)begin
            if((alu_in_A < alu_in_B))
                alu_result = 0;
            else
                alu_result = alu_in_A - alu_in_B;
        end
        else if (opsave == add8)begin
            alu_result[31:24] = alu_in_A[31:24] + alu_in_B[31:24];
            alu_result[23:16] = alu_in_A[23:16] + alu_in_B[23:16];
            alu_result[15:8] = alu_in_A[15:8] + alu_in_B[15:8];
            alu_result[7:0] = alu_in_A[7:0] + alu_in_B[7:0];
        end
        
        if (((alu_in_A == alu_in_B)&&(`opcode == beq)) || ((alu_in_A != alu_in_B)&&(`opcode == bne))) begin
          npc = pc + imm_ext[6:0];
          nstate = 3'd0;
        end
        else if (`opcode == jal)
        begin
            alu_result = pc;
            nstate = 3'd3;
        end
        else if ((`opcode == bne)||(`opcode == beq)) nstate = 3'd0;
        else if (opsave == jr) begin
          npc = alu_in_A[6:0];
          nstate = 3'd0;
        end
        
        
      end
      3: begin //prepare to write to mem
        nstate = 3'd0;
        if ((format == R)||(`opcode == addi)||(`opcode == andi)||(`opcode == ori)||(`opcode == jal)||(`opcode == lui)) regw = 1;
        else if (`opcode == sw) begin
          CS = 1;
          WE = 1;
          writing = 1;
        end
        else if (`opcode == lw) begin
          CS = 1;
          nstate = 3'd4;
        end
        if(`opcode == jal)
            npc = instr[6:0];
        
        
      end
      4: begin
        nstate = 3'd0;
        CS = 1;
        if (`opcode == lw) regw = 1;
      end
    endcase
  end //always

  always @(posedge CLK) begin

    if (RST) begin
      state <= 3'd0;
      pc <= 7'd0;
    end
    else begin
      state <= nstate;
      pc <= npc;
    end

    if (state == 3'd0) instr <= Mem_Bus;
    else if (state == 3'd1) begin
      opsave <= op;
      reg_or_imm_save <= reg_or_imm;
      alu_or_mem_save <= alu_or_mem;
    end
    else if (state == 3'd2) alu_result_save <= alu_result;

  end //always

endmodule
