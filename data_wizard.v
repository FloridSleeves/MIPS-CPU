`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:39:46 05/10/2017 
// Design Name: 
// Module Name:    data_wizard 
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
module dwiz_check(
    input [4:0]if2idRs,
    input [4:0]if2idRt,
    
    input [4:0]id2exRt,
    input id2ex_MemRead,
    output reg PCWrite,
    output reg IF2IDWrite,
    output reg con_mux
    
    );
    always@(if2idRs or if2idRt or id2exRt or id2ex_MemRead)
    begin
        if(id2ex_MemRead&(id2exRt!=0)&((if2idRs==id2exRt)|(if2idRt==id2exRt)) )
        begin
            PCWrite=0;
            IF2IDWrite=0;
            con_mux=0;
        end
        else
        begin
            PCWrite=1;
            IF2IDWrite=1;
            con_mux=1;
        end
    end
    
    
    

endmodule
