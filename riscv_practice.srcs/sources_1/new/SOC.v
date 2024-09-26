`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/20/2024 07:09:37 PM
// Design Name: 
// Module Name: SOC
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Memory (
    input               clk,
    input       [31:0]  mem_addr,
    output reg  [31:0]  mem_rdata,
    input               mem_rstrb
);
    reg [31:0] MEM [0:255];
    
    `include "riscv_assembly.v"
    integer L0_=8;
    initial begin
                  ADD(x1,x0,x0);      
                  ADDI(x2,x0,31);
      Label(L0_); ADDI(x1,x1,1); 
                  BNE(x1, x2, LabelRef(L0_));
                  EBREAK();
      endASM();
    end
    
    always @ (posedge clk) begin
        if (mem_rstrb) begin
            mem_rdata <= MEM[mem_addr[31:2]];
        end
    end
endmodule

//////////////////////////////////////////////////////////////////////////////////

module Processor (
    input               clk,
    input               resetn,
    output      [31:0]  mem_addr,
    input       [31:0]  mem_rdata,
    output              mem_rstrb,
    output reg  [31:0]  x1
);

    reg [31:0] PC = 0;
    reg [31:0] instr;
    
    reg [4:0] leds;
    assign LEDS = leds;
    
    // 10 RISC-V instructions
    wire isALUreg = (instr[6:0] == 7'b0110011);
    wire isALUimm = (instr[6:0] == 7'b0010011);
    wire isBranch = (instr[6:0] == 7'b1100011);
    wire isJALR = (instr[6:0] == 7'b1100111);
    wire isJAL = (instr[6:0] == 7'b1101111);
    wire isAUIPC = (instr[6:0] == 7'b0010111);
    wire isLUI = (instr[6:0] == 7'b0110111);
    wire isLoad = (instr[6:0] == 7'b0000011);
    wire isStore = (instr[6:0] == 7'b0100011);
    wire isSYSTEM = (instr[6:0] == 7'b1110011);
    
    // Immediate Formats
    wire [31:0] Uimm = {instr[31:12], {12{1'b0}}};
    wire [31:0] Iimm = {{21{instr[31]}}, instr[30:20]};
    wire [31:0] Simm = {{21{instr[31]}}, instr[30:25], instr[11:7]};
    wire [31:0] Bimm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
    wire [31:0] Jimm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
    
    // Source and Destination registers
    wire [4:0] rs2Id = instr[24:20];
    wire [4:0] rs1Id = instr[19:15];
    wire [4:0] rdId = instr[11:7];
    
    // Function Codes
    wire [2:0] funct3 = instr[14:12];
    wire [6:0] funct7 = instr[31:25];

    // Register Bank
    reg [31:0] RegisterBank [0:31];
    reg [31:0] rs1;
    reg [31:0] rs2;
    wire [31:0] writeBackData;
    wire writeBackEn;
    
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            RegisterBank[i] = 0;
        end
    end
    
    // The ALU
    wire [31:0] aluIn1 = rs1;
    wire [31:0] aluIn2 = isALUreg | isBranch ? rs2 : Iimm;
    reg  [31:0] aluOut;
    wire [4:0]  shamt = isALUreg ? rs2[4:0] : instr[24:20]; // Shift Amt
    
    // ALU size optimizations
    
    // 33 bits subtract
    wire [32:0] aluMinus = {1'b1,~aluIn2} + {1'b0,aluIn1} + 33'b1;
    wire        EQ  = (aluMinus[31:0] == 0);
    wire        LTU = (aluMinus[32]);
    wire        LT  = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluMinus[32];
    
    // adder
    wire [31:0] aluPlus = aluIn1 + aluIn2;
    wire [31:0] PCplusImm = PC + (  instr[3] ? Jimm[31:0] :
                                    instr[4] ? Uimm[31:0] :
                                               Bimm[31:0] );
    wire [31:0] PCplus4 = PC+4;
    
    // Shifter
    wire [31:0] shifter_in = (funct3 == 3'b001) ? flip32(aluIn2) : aluIn1;
    wire [31:0] leftshift = flip31(shifter);
    wire [31:0] shifter = $signed({instr[30] & aluIn1[31], shifter_in}) >>> aluIn2[4:0];
    
    always @ (*) begin
        case(funct3)
            3'b000: aluOut = (funct7[5] & instr[5]) ? aluMinus[31:0] : aluPlus;
            3'b001: aluOut = leftshift;
            3'b010: aluOut = {31'b0, LT};
            3'b011: aluOut = {31'b0, LTU};
            3'b100: aluOut = aluIn1 ^ aluIn2;
            3'b101: aluOut = shifter;
            3'b110: aluOut = (aluIn1 | aluIn2);
            3'b111: aluOut = (aluIn1 & aluIn2);
        endcase
    end
    
    reg takeBranch;
    always @ (*) begin
        case(funct3)
            3'b000:  takeBranch =   EQ;                      // BEQ
            3'b001:  takeBranch =  !EQ;                      // BNE
            3'b100:  takeBranch =   LT;                      // BLT
            3'b101:  takeBranch =  !LT;                      // BGE
            3'b110:  takeBranch =  LTU;                      // BLTU
            3'b111:  takeBranch = !LTU;                      // BGEU
            default: takeBranch = 1'b0;
        endcase
    
    end
    
    // State Machine
    localparam FETCH_INSTR  = 0;
    localparam WAIT_INSTR   = 1;
    localparam FETCH_REGS   = 2;
    localparam EXECUTE      = 3;
    reg [1:0] state = FETCH_INSTR;
    
    // Register Write Back
    assign writeBackData = (isJAL || isJALR) ? (PC + 4) :
                            (isLUI) ? Uimm :
                            isAUIPC ? (PC + Uimm) :
                            aluOut;
                            
    assign writeBackEn   = (state == EXECUTE && 
                           (isALUreg ||
                            isALUimm ||
                            isJAL    ||
                            isJALR   ||
                            isLUI    ||
                            isAUIPC)
                           );
    wire [31:0] nextPC = ((isBranch && takeBranch) || isJAL) ? PCplusImm :
                         isJALR                   ? {aluPlus[31:1], 1'b0} :
                         PCplus4;
    
    always @ (posedge clk) begin
    if (!resetn) begin
        PC      <= 0;
        state   <= FETCH_INSTR;
    end
        else begin
            if (writeBackEn && rdId !=0) begin
                RegisterBank[rdId] <= writeBackData;
                if (rdId == 1) begin
                    x1 <= writeBackData;
                end
            end
            
            case(state)
                FETCH_INSTR: begin
                    state <= WAIT_INSTR;
                end
                WAIT_INSTR: begin
                    instr <= mem_rdata;
                    state <= FETCH_REGS;
                end
                FETCH_REGS: begin
                    rs1 <= RegisterBank[rs1Id];
                    rs2 <= RegisterBank[rs2Id];
                    state <= EXECUTE;
                end
                EXECUTE: begin
                    if(!isSYSTEM) begin
                    PC <= nextPC;
                    end
                    state <= FETCH_INSTR;
                end
            endcase
        end
    end
    
    assign mem_addr = PC;
    assign mem_rstrb = (state == FETCH_INSTR);
    
endmodule

//////////////////////////////////////////////////////////////////////////////////

module SOC (
    input  CLK,        // system clock 
    input  RESET,      // reset button
    output [4:0] LEDS, // system LEDs
    input  RXD,        // UART receive
    output TXD         // UART transmit
);

    wire clk;    // internal clock
    wire resetn; // internal reset signal, goes low on reset
    
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire        mem_rstrb;
    wire [31:0] x1;
   
    // Memory
    Memory RAM(
        .clk(clk),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb)
    );
    
    // Processor
    Processor CPU(
        .clk(clk),
        .resetn(resetn),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb),
        .x1(x1)
    );
    
    // Clock gearbox
    Clockworks CW(
      .CLK(CLK),
      .RESET(RESET),
      .clk(clk),
      .resetn(resetn)
    );
    assign LEDS = x1[4:0];
    
    assign TXD  = 1'b0; // not used for now   
    
endmodule
