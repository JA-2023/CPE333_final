`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/07/2022 10:36:41 PM
// Design Name: 
// Module Name: Wrapper_SIM
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


module Otter_Wrapper_SIM();


   logic CLK;
   logic BTNL;
   logic BTNC;
   logic [15:0] SWITCHES;
   logic [15:0] LEDS;
   logic [7:0] CATHODES;
   logic [3:0] ANODES;

    OTTER_Wrapper wrapper_sim(.*);
    
    always
    begin
        CLK = 0; #5; CLK = 1; #5;
    end    
    
    initial
    begin
    
    BTNL = 0;
	BTNC = 0;
	SWITCHES = 0;
	
	end
    
    endmodule
