`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/20/2024 07:12:55 PM
// Design Name: 
// Module Name: bench_ 1
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


module bench_1;

reg CLK;
reg RESET = 0;
reg RXD = 1'b0;

wire [4:0] LEDS;
wire TXD;
    
SOC uut(
    .CLK(CLK),
    .RESET(RESET),
    .RXD(RXD),
    .LEDS(LEDS),
    .TXD(TXD)
);
    
reg [4:0] prev_LEDS = 0;
initial
begin
    CLK = 0;
    forever
    begin
        #1 CLK = ~CLK;
        if (LEDS != prev_LEDS)
        begin
            $display("LEDS = %b", LEDS);
        end
        prev_LEDS <= LEDS;
    end
end

endmodule
