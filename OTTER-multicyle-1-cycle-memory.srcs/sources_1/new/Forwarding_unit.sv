`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/29/2022 12:43:07 PM
// Design Name: 
// Module Name: Forwarding_unit
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


module Forwarding_unit(
    input ex_mem_RegWrite,
    input mem_wb_regWrite,
    input [4:0] Ex_mem_rd,
    input [4:0] de_ex_rs,
    input [4:0] de_ex_rt,
    input [4:0] mem_wb_rd,
    input de_ex_rs1_used,
    input de_ex_rs2_used,
    output logic [1:0] forwardA_sel,
    output logic [1:0] forwardB_sel
    );

    always_comb
    begin
    //Ex forward
        if((ex_mem_RegWrite == 1) && (Ex_mem_rd != 0) && (Ex_mem_rd == de_ex_rs) && de_ex_rs1_used)  ///rs_used
            forwardA_sel  = 'b10;
     //MEM Forward   
        else if((mem_wb_regWrite == 1) && (mem_wb_rd != 0) && (Ex_mem_rd != de_ex_rs) && de_ex_rs1_used //rs_used
                &&(mem_wb_rd == de_ex_rs ))
            forwardA_sel  = 'b01;
        else
            forwardA_sel = 'b0;
    end 
    always_comb    
    begin 
        //ex forward
        if((ex_mem_RegWrite == 1) && (Ex_mem_rd != 0) && (Ex_mem_rd == de_ex_rt) && de_ex_rs2_used) 
            forwardB_sel = 'b10;
        //mem forward     
        else if((mem_wb_regWrite == 1) && (mem_wb_rd != 0) && (Ex_mem_rd != de_ex_rt) && de_ex_rs2_used &&(mem_wb_rd == de_ex_rt))
            forwardB_sel = 'b01;
        else
            forwardB_sel = 'b0; 
    end                
endmodule
