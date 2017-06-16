`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:55:28 05/16/2017 
// Design Name: 
// Module Name:    addr 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module addr(
    input clk,
    input signal,
    input [4:0]address,
    output reg[4:0] input_a
    );
    
    reg [4:0]count=0;
    always@(posedge signal)
    begin
        if(address==5'b0)
            count<=count+5'b1;
        else
            count<=0;
    end
    
    always@(posedge clk)
    begin
        if(address==5'b0)
            input_a<=count;
        else
            input_a<=address;
    
    end
    

    
    
    
endmodule




