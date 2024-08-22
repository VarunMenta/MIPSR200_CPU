`timescale 1ns / 1ps
 
module display_clk(
    input clk,
    input reset,
    output slow_clk
);
 
//Create a simple counter
    
reg [18:0] counter = 0;
 
assign slow_clk = counter[18];
 
always@(posedge clk) begin
    if (reset)
        counter = 0;
    else 
        counter = counter + 1; //resets counter 
end
 
endmodule