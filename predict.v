`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:06:11 05/18/2017 
// Design Name: 
// Module Name:    predict 
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
module predict(
    input PCSrc0,
    input BInstr,
    input [31:0]PCPlus4ID,
    input [31:0]PCPlus4MEM,
    input [31:0]PCBranch,
    //input clk,
    output reg JumpPre,
    output reg [31:0]JumpPrePC
    );
    
    reg [1:0]st=2'b11;
    reg [31:0]PCrecord=32'hFFFF_FFFF;
    
    
    always@(BInstr)
    begin
        if(BInstr)//Branch
        begin
            if(PCrecord==PCPlus4ID)
            begin
                case(st)
                    2'b00:JumpPre<=1;
                    2'b01:JumpPre<=1;
                    2'b10:JumpPre<=0;
                    2'b11:JumpPre<=0;
                endcase
            end
            else
            begin
                PCrecord<=PCPlus4ID;
            end
        end
        else JumpPre<=0;
    end
    
    always@(PCPlus4MEM or PCSrc0)
    begin
        if(PCPlus4MEM==PCrecord)
        begin
            if(PCSrc0)
            begin
                case(st)
                    2'b00:st<=2'b00;
                    2'b01:st<=2'b00;
                    2'b10:st<=2'b01;
                    2'b11:st<=2'b10;
                endcase
                JumpPrePC<=PCBranch;
            end
            else
            begin
                case(st)
                    2'b00:st<=2'b01;
                    2'b01:st<=2'b10;
                    2'b10:st<=2'b11;
                    2'b11:st<=2'b11;
                endcase
            end    
        end
        else ;
    end


endmodule
