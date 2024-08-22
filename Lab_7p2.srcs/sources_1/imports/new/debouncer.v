`timescale 1ns / 1ps

module debouncer(
    input signal, clk,
    output stable
    );
    
    wire bclk;
    bounce_clk a1(.clk(clk),.bclk(bclk));
    
    reg out1 = 0;
    reg out2 = 0;
    
    assign stable = out2;
    
    always @(posedge bclk)
    begin
       out1 <= signal;
       out2 <= out1;
    end
    
    
endmodule

module bounce_clk(
    input clk,
    output bclk
);
    reg [23:0] count_val = 5000000;
    reg [24:0] counter = 0;
    assign bclk = counter == count_val;
   
    always @ (posedge clk)
    begin
        if (counter == (count_val))
            counter = 0;
        else
            counter = counter + 1;
    end 

endmodule
