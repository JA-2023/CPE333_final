`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/16/2022 03:41:37 PM
// Design Name: 
// Module Name: BranchCondGen
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


module BranchCondGen(
    input [2:0] FUNC3,
    input [31:0]RS1,
    input [31:0]RS2,
    input [6:0] BC_OPCODE,
    output logic [1:0] PCSOURCE
    );
    
    logic brn_cond;
    
    // Branch condition generator 
    assign BR_EQ = (RS1 == RS2) ? 1 : 0;
    assign BR_LT = ($signed(RS1) < $signed(RS2)) ? 1 : 0;
    assign BR_LTU = (RS1 < RS2) ? 1 : 0;
    
    
    typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
    } opcode_t;
          
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(BC_OPCODE);
    
    //determine the branch condition
    always_comb
    case(FUNC3)
        3'b000: brn_cond = BR_EQ;     //BEQ 
        3'b001: brn_cond = ~BR_EQ;    //BNE
        3'b100: brn_cond = BR_LT;     //BLT
        3'b101: brn_cond = ~BR_LT;    //BGE
        3'b110: brn_cond = BR_LTU;    //BLTU
        3'b111: brn_cond = ~BR_LTU;   //BGEU
        default: brn_cond =0;
    endcase
    
    //determine what signal to send into PC
    always_comb begin
    case(OPCODE)
        JAL: PCSOURCE =3'b011;
        JALR: PCSOURCE=3'b001;
        BRANCH: PCSOURCE=(brn_cond)?3'b010:2'b000;
        SYSTEM: PCSOURCE = (FUNC3==3'b000)? 3'b101:3'b000;
        default: PCSOURCE=3'b000; 
    endcase
    end

endmodule
