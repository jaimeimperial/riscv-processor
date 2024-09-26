`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/20/2024 07:20:49 PM
// Design Name: 
// Module Name: Clockworks
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

module Clockworks 
#(parameter SLOW = 0)
(
   input  CLK,   // clock pin of the board
   input  RESET, // reset pin of the board
   output clk,   // (optionally divided) clock for the design.
   output resetn // (optionally timed) negative reset for the design (more on this later)
);

assign resetn = 1;

reg [SLOW:0] slow_CLK = 0;

always @(posedge CLK)
begin
      slow_CLK <= slow_CLK + 1;
end

assign clk = slow_CLK[SLOW];

endmodule
