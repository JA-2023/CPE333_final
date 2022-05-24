`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/02/2022 09:59:25 AM
// Design Name: 
// Module Name: HazardDetectionUnit
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


module HazardUnit(
    input DE_EX_memread,
    input [4:0] DE_EX_rd, 
    input [4:0] DE_EX_rt,
    input [4:0] IF_DE_rs,
    input [4:0] IF_DE_rt,
    input rs1_used,
    input rs2_used,
    input CLK,
    output logic IF_DE_ctrl,
    output logic PCwrite_ctrl,
    output logic stall_signal,
    output logic memread1
    );
    

    logic read_signal = 0;
    always_ff @(posedge CLK)
    begin

        if((DE_EX_memread ==1) && ((DE_EX_rd == IF_DE_rs ) || (DE_EX_rd == IF_DE_rt )) && (rs1_used || rs2_used))
        begin 
            //stall the PC and DE_EX registers
            PCwrite_ctrl = 'b0;
            IF_DE_ctrl = 'b0;
            //insert bubble by making control signals to 0
            stall_signal = 'b1;
            memread1 = 'b0;
        end
        else
        begin
            //default where the pipeline operates normally
            PCwrite_ctrl = 'b1;
            IF_DE_ctrl = 'b1;
            memread1 = 'b1;
            stall_signal = 'b0;
        end
        
    end
endmodule
