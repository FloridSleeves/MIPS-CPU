`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:04:46 05/10/2017 
// Design Name: 
// Module Name:    dwiz_forward 
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
module dwiz_forward(
    input [4:0]id2exRs,
    input [4:0]id2exRt,
    input [4:0]ex2memRd,
    input [4:0]mem2wbRd,
    input ex2memRegWrite,
    input mem2wbRegWrite,
    input ex2memMemWrite,
    output reg[1:0]alua_mux,
    output reg[1:0]alub_mux,
    output reg mem_in_mux
    );
    always@(id2exRs or id2exRt or ex2memRd or mem2wbRd or ex2memRegWrite or mem2wbRegWrite or ex2memMemWrite)
    begin
        //mem alu 
        if(ex2memRegWrite&(ex2memRd!=0)&(ex2memRd==id2exRs) )
            alua_mux<=2'b10;
        else
        begin
            //wb alu
            if(mem2wbRegWrite&(mem2wbRd!=0)&(mem2wbRd==id2exRs) &!(ex2memRegWrite&(ex2memRd!=0)&(ex2memRd==id2exRs)))
                alua_mux<=2'b01;
            else alua_mux<=2'b00;
        end
        //mem alu
        if(ex2memRegWrite&(ex2memRd!=0)&(ex2memRd==id2exRt) )
            alub_mux<=2'b10;
        else
        begin
            //wb alu
            if(mem2wbRegWrite&(mem2wbRd!=0)&(mem2wbRd==id2exRt)&!(ex2memRegWrite&(ex2memRd!=0)&(ex2memRd==id2exRt)))
                alub_mux<=2'b01;
            else alub_mux<=2'b00;
        end
        //mem wb
        if(ex2memMemWrite&mem2wbRegWrite&(ex2memRd!=0)&(ex2memRd==mem2wbRd))
            mem_in_mux<=1;
        else
            mem_in_mux<=0;
            
        
    end
    
endmodule
