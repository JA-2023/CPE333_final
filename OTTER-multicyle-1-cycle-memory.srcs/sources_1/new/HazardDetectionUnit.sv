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


module HazardDetectionUnit(
    input DE_EX_memread,
    input [4:0] DE_EX_rt,
    input [4:0] IF_DE_rs,
    input [4:0] IF_DE_rt,
    output logic IF_DE_ctrl,
    output logic PCwrite_ctrl,
    output logic stall_signal
    );
    
    //
    always_comb 
    begin
        if((DE_EX_memread == 1) && (DE_EX_rt == IF_DE_rs) || (DE_EX_rt ==  IF_DE_rt))
        begin 
            //stall the PC and DE_EX registers
            PCwrite_ctrl = 'b0;
            IF_DE_ctrl = 'b0;
            //insert bubble by making control signals to 0
            stall_signal = 'b1;
        end
        else
        begin
            //default where the pipeline operates normally
            PCwrite_ctrl = 'b1;
            IF_DE_ctrl = 'b1;
            stall_signal = 'b0;
        end
        
    end
endmodule
