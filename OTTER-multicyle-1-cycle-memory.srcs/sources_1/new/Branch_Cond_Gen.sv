`timescale 1ns / 1ps

module BranchCondGen (input FUNC3,
                      input RS1, 
                      input RS2,
                      output logic [3:0] CU_PCSOURCE);
    
    logic brn_cond;
    
    // Branch condition generator 
    assign br_eq = (RS1 == RS2) ? 1 : 0;
    assign br_lt = ($signed(RS1) < $signed(RS2)) ? 1 : 0;
    assign br_ltu = (RS1 < RS2) ? 1 : 0;
    
    always_comb
    case(CU_FUNC3)
        3'b000: brn_cond = CU_BR_EQ;     //BEQ 
        3'b001: brn_cond = ~CU_BR_EQ;    //BNE
        3'b100: brn_cond = CU_BR_LT;     //BLT
        3'b101: brn_cond = ~CU_BR_LT;    //BGE
        3'b110: brn_cond = CU_BR_LTU;    //BLTU
        3'b111: brn_cond = ~CU_BR_LTU;   //BGEU
        default: brn_cond =0;
    endcase
    
    always_comb begin
    case(CU_OPCODE)
        JAL: CU_PCSOURCE =3'b011;
        JALR: CU_PCSOURCE=3'b001;
        BRANCH: CU_PCSOURCE=(brn_cond)?3'b010:2'b000;
        SYSTEM: CU_PCSOURCE = (CU_FUNC3==Func3_PRIV)? 3'b101:3'b000;
        default: CU_PCSOURCE=3'b000; 
    endcase
    end
        
endmodule 