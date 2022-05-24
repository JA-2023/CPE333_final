`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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
        

typedef struct packed{
    opcode_t opcode;
    logic [31:0] REG_PC;
    logic [31:0] REG_IR;
    logic REG_regWrite;
    logic REG_memWrite;
    logic REG_memRead2;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] REG_alu_fun;
    logic [31:0] REG_opA;
    logic [31:0] REG_opB;
    logic [1:0] REG_rf_wr_sel;
    logic [31:0] REG_rs1;
    logic [31:0] REG_rs2;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr; //address of the two source registers (rs1,rs2) and destination (rd)
    logic [31:0] REG_I_immed, REG_J_immed, REG_B_immed, REG_S_immed, REG_U_immed;
    logic [31:0] REG_aluRes;
    logic [3:0] REG_PCsource;
    logic [31:0] REG_mem2_out;
} pipe_reg;


module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    logic [6:0] opcode;
    //program counter signals
    wire [31:0] pc_in, pc_out, jalr_pc, branch_pc, jal_pc, pc_4;
    logic [1:0] pc_sel;
    
    //ALU related signals
    wire [31:0] rs1, rs2;
    wire [31:0] aluBin,aluAin,aluResult;
    wire [3:0]alu_fun;
    wire [1:0] alu_muxb_sel;
    wire alu_muxa_sel;
    
    //immidate gen signals
    wire [31:0] I_immed, J_immed, B_immed, S_immed, U_immed;
    //memory/reg signals
    wire [31:0] IR, mem2_out;
    //control signals
    wire memRead1,memRead2;
    wire regWrite,memWrite;
    reg pcWrite;
    
    wire [1:0] mSize;
    logic [1:0]RF_WR_SEL;
    wire [31:0] reg_mux_out;
    
    //hazard unit signals
    logic IF_DE_ctrl, stall_signal;
    logic PCwrite_ctrl;
    logic [31:0] OP_A, OP_B;
    logic [1:0] forwardA_sel, forwardB_sel;

    
//==== Instruction Fetch ===========================================

     opcode_t OPCODE;
     assign OPCODE_t = opcode_t'(opcode);
     assign opcode = IR[6:0];
     assign pc_4 = pc_out + 4;
     logic [1:0] pc_sel_prev;
     
    //program counter mux
     Mult4to1 PC_Mux (.In1(pc_4), .In2(jalr_pc), .In3(branch_pc), .In4(jal_pc), .Sel(pc_sel), .Out(pc_in));
     
     //program counter
     ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(PCwrite_ctrl), .PC_DIN(pc_in), .PC_COUNT(pc_out));
    
     //instruction to decode register
     pipe_reg if_de_reg;
    
     //put values into register
     always_ff @(posedge CLK) begin
     if(IF_DE_ctrl == 1)
     begin
         if_de_reg.REG_PC <= pc_out; 
     end
        //signal is used to make sure pipeline stalls for two cycles
        pc_sel_prev <= pc_sel;
     end
//==== Instruction Decode ===========================================
    
    logic haz_rs1_used, haz_rs2_used;
    
    always_comb
    begin 
    haz_rs1_used = ((IR[19:15] != 0)
                        && (IR[6:0] != LUI)
                        && (IR[6:0] != AUIPC));
           
   haz_rs2_used = ((IR[24:20] != 0)
                            && (IR[6:0] != LUI)
                            && (IR[6:0] != AUIPC)
                            && (IR[6:0] != JAL)
                            && (IR[6:0] != JALR)
                            && (IR[6:0] != LOAD)
                            && (IR[6:0] != OP_IMM));  
    end
   
    HazardUnit HazUnit(.DE_EX_memread(de_ex_reg.REG_memRead2),.DE_EX_rd(de_ex_reg.rd_addr),.DE_EX_rt(de_ex_reg.rs2_addr),.IF_DE_rs(IR[19:15]),.CLK(CLK),
                        .IF_DE_rt(IR[24:20]),.IF_DE_ctrl(IF_DE_ctrl),.memread1(memRead1),.rs1_used(haz_rs1_used), .rs2_used(haz_rs2_used),
                        .PCwrite_ctrl(PCwrite_ctrl),.stall_signal(stall_signal));
    
    OTTER_registerFile Register(.Read1(IR[19:15]), .Read2(IR[24:20])
                                    ,.WriteReg(mem_wb_reg.rd_addr),.WriteData(reg_mux_out)
                                    ,.RegWrite(mem_wb_reg.REG_regWrite),.Data1(rs1),.Data2(rs2),.clock(CLK));

    OTTER_CU_Decoder decoder (.CU_OPCODE(IR[6:0]), .CU_FUNC3(IR[14:12]), .CU_FUNC7(IR[31:25]),
                     .CU_ALU_SRCA(alu_muxa_sel), .CU_ALU_SRCB(alu_muxb_sel), .CU_ALU_FUN(alu_fun)
                     ,.CU_RF_WR_SEL(RF_WR_SEL),.REG_WRITE(regWrite),.MEM_WRITE(memWrite),.MEM_READ2(memRead2));
                                                                                
    //picks between the immidate values and rs1 vlaues and is sent to register then forwarding unit                            
    Mult2to1 ALU_MUX_A (.In1(rs1), .In2(U_immed), .Sel(alu_muxa_sel), .Out(OP_A));
    
    //picks between the immidate values and rs2 vlaues and is sent to register then forwarding unit  
    Mult4to1 ALU_MUX_B (.In1(rs2), .In2(I_immed), .In3(S_immed),
                        .In4(if_de_reg.REG_PC), .Sel(alu_muxb_sel), .Out(OP_B));    

    //immediate generator
    assign I_immed = { {21{IR[31]}}, IR[30:20] };
    assign S_immed = { {21{IR[31]}}, IR[30:25], IR[11:7]};
    assign B_immed = { {20{IR[31]}}, IR[7], IR[30:25], IR[11:8], 1'b0};
    assign U_immed = { IR[31:12], {12{1'b0}}};
    assign J_immed = { {12{IR[31]}}, IR[19:12], IR[20], IR[30:21], 1'b0};
    
    logic [31:0] de_ex_SW_rs2;
    //decode to execute register 
     pipe_reg de_ex_reg;
    //insert signals to register
     always_ff @(posedge CLK) begin

        begin
        //checks if the pipeline needs to stall and will make the control signals 0 to create a bubble
        if(stall_signal == 1)
        begin
            de_ex_reg.REG_regWrite <= 0;
            de_ex_reg.REG_memWrite <= 0;
            de_ex_reg.REG_memRead2 <= 0;
            de_ex_reg <= 0;
        end
        //normal pipeline operation
        else
        begin
            de_ex_reg.REG_regWrite <= regWrite;
            de_ex_reg.REG_memWrite <= memWrite;
            de_ex_reg.REG_memRead2 <= memRead2;
        end    
            de_ex_reg.REG_PC <= if_de_reg.REG_PC;
            de_ex_reg.REG_IR <= IR;
            de_ex_reg.REG_alu_fun <= alu_fun;
            de_ex_reg.REG_rf_wr_sel <= RF_WR_SEL;
            de_ex_reg.rs1_addr <= IR[19:15];
            de_ex_reg.rs2_addr <= IR[24:20];
            de_ex_reg.rd_addr <= IR[11:7];
            de_ex_reg.REG_rs1 <= OP_A;
            de_ex_reg.REG_rs2 <= OP_B;
            de_ex_reg.REG_I_immed <= I_immed;
            de_ex_reg.REG_J_immed <= J_immed;
            de_ex_reg.REG_B_immed <= B_immed;
            
            //signal added to accomadate store word hazards
            de_ex_SW_rs2 <= rs2;
            
            //checking opcaode to determine if rs1/rs2 are used
            de_ex_reg.rs1_used <= ((IR[19:15] != 0)
                                    && (IR[6:0] != LUI)
                                    && (IR[6:0] != AUIPC));
           
           de_ex_reg.rs2_used <= ((IR[24:20] != 0)
                                    && (IR[6:0] != LUI)
                                    && (IR[6:0] != AUIPC)
                                    && (IR[6:0] != JAL)
                                    && (IR[6:0] != JALR)
                                    && (IR[6:0] != LOAD)
                                    && (IR[6:0] != OP_IMM));     
                                    
           de_ex_reg.rd_used <= ((IR[11:7] != 0)
                                    && (IR[6:0] != BRANCH)
                                    && (IR[6:0] != STORE));                                          
        end
        //branch detection
        //check if anything besides the next PC value is seleted for the PC mux
        if((pc_sel != 0) || (pc_sel_prev != 0))
        begin
            //clear pipeline register
            de_ex_reg <= 0;
        end
        
     end

//==== Execute ======================================================


    //the select signal from the forward unit picks which unit should go into the ALU
    Mult4to1 ForwardA (.In1(de_ex_reg.REG_rs1), .In2(reg_mux_out), .In3(ex_mem_reg.REG_aluRes),
                       .In4(0), .Sel(forwardA_sel), .Out(aluAin));
                       
    ///the select signal from the forward unit picks which unit should go into the ALU
    Mult4to1 ForwardB (.In1(de_ex_reg.REG_rs2), .In2(reg_mux_out), .In3(ex_mem_reg.REG_aluRes),
                       .In4(0), .Sel(forwardB_sel), .Out(aluBin));  
                  
    //target generator
    assign jal_pc = de_ex_reg.REG_PC + de_ex_reg.REG_J_immed;
    assign jalr_pc = aluAin + de_ex_reg.REG_I_immed; //takes the forwarded value
    assign branch_pc = de_ex_reg.REG_PC + de_ex_reg.REG_B_immed;
         
    BranchCondGen branchGen (.BC_OPCODE(de_ex_reg.REG_IR[6:0]), .FUNC3(de_ex_reg.REG_IR[14:12]),
                             .RS1(aluAin), .RS2(aluBin), .PCSOURCE(pc_sel)); 
     
     // Creates a RISC-V ALU
    OTTER_ALU ALU (.ALU_fun(de_ex_reg.REG_alu_fun), .A(aluAin), .B(aluBin), .ALUOut(aluResult)); // the ALU
    
    //creates forwarding unit
    Forwarding_unit forward (.ex_mem_RegWrite(ex_mem_reg.REG_regWrite),.mem_wb_regWrite(mem_wb_reg.REG_regWrite),
                            .Ex_mem_rd(ex_mem_reg.rd_addr),.de_ex_rs(de_ex_reg.rs1_addr),.de_ex_rt(de_ex_reg.rs2_addr),
                            .mem_wb_rd(mem_wb_reg.rd_addr),.forwardA_sel(forwardA_sel),.forwardB_sel(forwardB_sel),
                            .de_ex_rs1_used(de_ex_reg.rs1_used),.de_ex_rs2_used(de_ex_reg.rs2_used));
    
    //execute to memory register
    pipe_reg ex_mem_reg;

     always_ff @(posedge CLK) begin
        
        //move signals down pipeline  
        ex_mem_reg.REG_PC <= de_ex_reg.REG_PC;
        ex_mem_reg.REG_IR <= de_ex_reg.REG_IR;
        ex_mem_reg.rd_addr <= de_ex_reg.rd_addr;
        ex_mem_reg.REG_regWrite <= de_ex_reg.REG_regWrite;
        ex_mem_reg.REG_memWrite <= de_ex_reg.REG_memWrite;
        ex_mem_reg.REG_memRead2 <= de_ex_reg.REG_memRead2;
        ex_mem_reg.REG_aluRes <= aluResult;
        ex_mem_reg.REG_rf_wr_sel <= de_ex_reg.REG_rf_wr_sel;
        
        //logic to check if it is store operation and send the correct rs2 value
        if(de_ex_reg.REG_IR[6:0] == STORE)
            ex_mem_reg.REG_rs2 <= de_ex_SW_rs2;
        else
            ex_mem_reg.REG_rs2 <= de_ex_reg.REG_rs2;
     end
//==== Memory ======================================================
    
    assign IOBUS_ADDR = ex_mem_reg.REG_aluRes;
    assign IOBUS_OUT = ex_mem_reg.REG_rs2;

                                   
   OTTER_mem_byte BYTE_MEM (.MEM_CLK(CLK),.MEM_ADDR1(pc_out),.MEM_ADDR2(ex_mem_reg.REG_aluRes),.MEM_DIN2(ex_mem_reg.REG_rs2),
                            .MEM_WRITE2(ex_mem_reg.REG_memWrite),.MEM_READ1(memRead1),.MEM_READ2(ex_mem_reg.REG_memRead2),
                            .ERR(),.MEM_DOUT1(IR),.MEM_DOUT2(mem2_out),.IO_IN(IOBUS_IN),.IO_WR(IOBUS_WR),.MEM_SIZE(ex_mem_reg.REG_IR[13:12]),.MEM_SIGN(ex_mem_reg.REG_IR[14]));
   pipe_reg mem_wb_reg;
   
    always_ff @(posedge CLK) begin
        mem_wb_reg.REG_PC <= ex_mem_reg.REG_PC;
        mem_wb_reg.rd_addr <= ex_mem_reg.rd_addr;
        mem_wb_reg.REG_rf_wr_sel <= ex_mem_reg.REG_rf_wr_sel;
        mem_wb_reg.REG_aluRes <= ex_mem_reg.REG_aluRes;
        mem_wb_reg.REG_regWrite <= ex_mem_reg.REG_regWrite;
     end
        
//==== Write Back ==================================================
    logic [31:0] mem_pc4;

    assign mem_pc4 = mem_wb_reg.REG_PC + 4;
//register mux
    Mult4to1 reg_mux (.In1(mem_pc4), .In2(csr_reg), .In3(mem2_out),
                        .In4(mem_wb_reg.REG_aluRes), .Sel(mem_wb_reg.REG_rf_wr_sel), .Out(reg_mux_out));          
endmodule
