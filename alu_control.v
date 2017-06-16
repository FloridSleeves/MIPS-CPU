`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:54:14 05/09/2017 
// Design Name: 
// Module Name:    alu_control 
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
module alu_control(
    input [1:0]ALUOp,
    input [5:0]Funct,
    input [5:0]Opcode,
    output reg[4:0]ALUControl,
    output reg sign_care
    );
    always@(ALUOp or Funct or Opcode)
    begin
        case(ALUOp)
            2'b00:;//LW
            2'b01:;//SW
            2'b10://R-Type
                begin
                    case(Funct)
                        6'h20:sign_care=1;
                        6'h21:sign_care=0;
                        6'h22:sign_care=1;
                        6'h23:sign_care=0;
                        6'h24:sign_care=0;
                        6'h25:sign_care=0;
                        6'h26:sign_care=0;
                        6'h27:sign_care=0;
                        6'h4:sign_care=0;
                        6'h7:sign_care=0;
                        6'h6:sign_care=0;
                        6'h2a:sign_care=0;
                        6'h2b:sign_care=0;
                        6'h8:sign_care=0;
                    endcase   
                end
            2'b11://imm or without alu
                begin
                    case(Opcode)
                        6'h4:sign_care=0;
                        6'h7:sign_care=0;
                        6'h1:sign_care=0;
                        6'h5:sign_care=0;
                        6'h8:sign_care=1;
                        6'h9:sign_care=0;
                        6'hc:sign_care=0;
                        6'hd:sign_care=0;
                        6'he:sign_care=0;
                        6'hf:sign_care=0;
                        default:sign_care=0;
						endcase
                end
        endcase
    end
    always@(ALUOp or Funct or Opcode)
    begin
        case(ALUOp)
            2'b00:ALUControl=5'd2;//LW
            2'b01:ALUControl=5'd2;//SW
            2'b10://R-Type
                begin
                    case(Funct)
                        6'h20:ALUControl=5'd2;
                        6'h21:ALUControl=5'd2;
                        6'h22:ALUControl=5'd6;
                        6'h23:ALUControl=5'd6;
                        6'h24:ALUControl=5'd0;
                        6'h25:ALUControl=5'd1;
                        6'h26:ALUControl=5'd3;
                        6'h27:ALUControl=5'd12;
                        6'h4:ALUControl=5'd16;
                        6'h7:ALUControl=5'd18;
                        6'h6:ALUControl=5'd17;
                        6'h2a:ALUControl=5'd19;
                        6'h2b:ALUControl=5'd20;
                        6'h8:ALUControl=5'd2;
                    endcase   
                end
            2'b11://imm or without alu
                begin
                    case(Opcode)
                        6'h1:ALUControl=5'd6;
                        6'h4:ALUControl=5'd6;
                        6'h5:ALUControl=5'd6;
                        6'h7:ALUControl=5'd6;
                        6'h8:ALUControl=5'd2;
                        6'h9:ALUControl=5'd2;
								6'hb:ALUControl=5'd20;
                        6'hc:ALUControl=5'd0;
                        6'hd:ALUControl=5'd8;
                        6'he:ALUControl=5'd9;
                        6'hf:ALUControl=5'd10;
                        default:ALUControl=5'd0;
						endcase
                end
        endcase
    end
endmodule
