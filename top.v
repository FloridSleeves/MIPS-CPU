`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:05:05 05/09/2017 
// Design Name: 
// Module Name:    top 
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
module cpu(
    input clk,
   input interrupt
	 //input rst_n,
    // input [4:0]input_a,
    //input check,
   // output [31:0]CPUData,
	// output complete,
    // output [31:0]regData
    );
   //parameter COMPLETE=32'd300;
	parameter INT_DEAL=32'd356;
    wire [31:0]ReadData;
    wire flush;
    wire overflow;
    wire overflow_flush;
    wire [31:0]SignImm;
    
    wire [31:0]SrcA;
	wire [31:0]SrcB;
    
    wire [31:0]ALUResult;
    wire [4:0]Des_reg;

    wire MemtoReg;
    wire RegDst;

    wire ALUSrc;    
	
	wire MemWrite;
	wire PCWrite;
    wire pBranch;
    wire nBranch;
    wire zBranch;
    wire JumpPre;
    wire [31:0]JumpPrePC;
    
    wire RegWrite;
    wire MemRead;
    
    wire Jump;
    
   
	 reg [63:0]IF2ID=0;
	 reg [200:0]ID2EX=0;
	 reg [200:0]EX2MEM=0;
	 reg [200:0]MEM2WB=0;
	 reg [31:0]PC=0;
     
     wire IF2IDWrite;
     wire con_mux;
     
	  
     //IF
     
     wire [1:0]PCSrc;
     wire [31:0]Instr;
     
    //wire test=overflow_flush|interrupt;
//	 assign complete=(PC==COMPLETE);
     
     always@(posedge clk or negedge rst_n)
     begin
			if(~rst_n)
				PC<=0;
			else 
			begin
			
				if(interrupt)
					PC<=INT_DEAL;
				else
				begin
			  //jump and branch first because the following ins may not even happened
					if(PCSrc==2'b01)
						 if(pre_wrong1)PC<=EX2MEM[103:72];
						 else 
						 begin
							if(JumpPre==1)
									PC<=JumpPrePC+32'd4;
							else
								PC<=PC+32'd4;
						 end
					else 
					begin
						if(PCSrc==2'b10)
							 PC<=EX2MEM[142:111];
						else 
						begin
							if(pre_wrong2)
								PC<=EX2MEM[175:144];
							else
							begin
								if(JumpPre==1)
									PC<=JumpPrePC+32'd4;
								else
								begin
									if(overflow_flush) 
										 PC<=INT_DEAL;
									else 
									begin
										 if(PCWrite==0 );//if stall then wait,the overflow may not happen
										 else
											PC<=PC+4;
								
									end
								end
							end
						end
					end
				end
				
			end
     end
     
	  wire [31:0]FinalPC=((PCSrc==2'b01)&(~pre_wrong1)&JumpPre |(PCSrc==2'b00)&(~pre_wrong2)&JumpPre)?JumpPrePC:PC;
     
     ins uins(.a(FinalPC[8:2]),.d(0),.clk(clk),.we(0),.spo(Instr));
     always@(posedge clk)
     begin
        if(flush | overflow_flush |interrupt |~rst_n )
            IF2ID<=0;
        else
        begin
            if(IF2IDWrite==0);
            else
                IF2ID<={FinalPC+4,Instr};//PC has +4
        end 
	  end 
     
     //ID
    
    wire [31:0]tA;
    wire [31:0]tB;
    wire [1:0]ALUOp;
    wire [31:0]PCJump;
    
    assign PCJump={IF2ID[63:60],IF2ID[25:0],2'b0};
    predict u_pre(.PCSrc0(PCSrc[0]),.BInstr(pBranch|nBranch|zBranch),
    .PCPlus4ID(IF2ID[63:32]),.PCPlus4MEM(EX2MEM[175:144]),
    .PCBranch(EX2MEM[103:72]),.JumpPre(JumpPre),.JumpPrePC(JumpPrePC)
    );
    dwiz_check u_dcheck(.if2idRs(IF2ID[25:21]),.if2idRt(IF2ID[20:16]),.id2exRt(Des_reg),.id2ex_MemRead(ID2EX[181]),.PCWrite(PCWrite),.IF2IDWrite(IF2IDWrite),.con_mux(con_mux));  
    wire JumpR;
    wire Jal;
    control ucon(
    .Op(IF2ID[31:26]),.Funct(IF2ID[5:0]),.clk(clk),.rst(1),
    .MemtoReg(MemtoReg),.MemRead(MemRead),.MemWrite(MemWrite),.pBranch(pBranch),.zBranch(zBranch),.nBranch(nBranch),.ALUSrc(ALUSrc),.RegDst(RegDst),.RegWrite(RegWrite),
    .ALUOp(ALUOp),.Jump(Jump),.JumpR(JumpR),.Jal(Jal));
    wire [31:0]r3_din;
    
    wire [4:0]r3_addr;
    assign r3_addr=MEM2WB[4:0];
    
    wire [4:0]r1_addr;
    assign r1_addr=IF2ID[25:21];
    //assign regData=tA;
   // assign CPUData=tA;
    reg_file ureg(
    .clk(clk),.r1_addr(r1_addr),
    .r2_addr(IF2ID[20:16]),
    .r3_addr(r3_addr),
    .r3_din(r3_din),
    .r3_wr(MEM2WB[70]),
    .r1_dout(tA),.r2_dout(tB),.rst_n(rst_n));
    
    assign SignImm=(IF2ID[15]==1)?{16'hFFFF,IF2ID[15:0]}:{16'd0,IF2ID[15:0]};
    wire [43:0]Control={MemRead,PCJump,Jump,RegDst,ALUOp,ALUSrc,nBranch,zBranch,pBranch,MemWrite,RegWrite,MemtoReg};
    always@(posedge clk)
     begin
        if(flush | overflow_flush | interrupt | ~rst_n  )
            ID2EX<=0;
        else
            ID2EX<={(con_mux==1)?Jal:1'b0,(con_mux==1)?JumpR:1'b0,IF2ID[31:26],IF2ID[25:21],(con_mux==1)?Control:44'b0,IF2ID[63:32],tA,tB,SignImm,IF2ID[20:16],IF2ID[15:11]};
     end
     
     //EX
     wire zero;
	wire pos;
    wire neg;
    wire [31:0]PCBranch;
    wire [1:0]alu_a_mux;
    wire [1:0]alu_b_mux;
    
    dwiz_forward u_dforward(.id2exRs(ID2EX[186:182]),.id2exRt(ID2EX[9:5]),
        .ex2memRd(EX2MEM[4:0]),.mem2wbRd(MEM2WB[4:0]),
        .ex2memRegWrite(EX2MEM[105]),.mem2wbRegWrite(MEM2WB[70]),
        .ex2memMemWrite(EX2MEM[104]),
        .alua_mux(alu_a_mux),.alub_mux(alu_b_mux));
    
    wire signed [31:0]add1=ID2EX[137:106];
	 wire signed [31:0]add2=(ID2EX[41:10])<<2;
    assign PCBranch=add1+add2;
    assign SrcA=(alu_a_mux[1]==1)?
                EX2MEM[68:37]:
                ((alu_a_mux[0]==1)?r3_din:ID2EX[105:74]);
    wire [31:0]SrcBReg;
    assign SrcBReg=(alu_b_mux[1]==1)?EX2MEM[68:37]:
                ((alu_b_mux[0]==1)?r3_din:ID2EX[73:42]);
    assign SrcB=(ID2EX[144]==1)?ID2EX[41:10]:SrcBReg;
    wire [4:0]ALUControl;
    assign overflow_flush=overflow&(PCSrc==2'b00);
	reg [31:0]EPC=0;
    reg [31:0]Cause=0;
    
    always@(posedge clk)
    begin
        if(interrupt)
        begin
            EPC<=ID2EX[137:106];
            Cause[6:2]<=5'd0;//OUTINT
        end
        else if(overflow_flush)//won't jump and overflow
        begin
            EPC<=ID2EX[137:106];
            Cause[6:2]<=5'd12;//OVERFLOW
        end    
    end
    
    wire sign_care;
    alu_control u_aluc(.ALUOp(ID2EX[146:145]),.Funct(ID2EX[15:10]),.Opcode(ID2EX[192:187]),.ALUControl(ALUControl),.sign_care(sign_care));
    alu u_alu(.alu_a(SrcA),.alu_b(SrcB),.alu_op(ALUControl),.alu_out(ALUResult),.zero(zero),.pos(pos),.neg(neg),.overflow(overflow),.sign_care(sign_care));
    
    assign Des_reg=(ID2EX[194]==1)?5'd31:((ID2EX[147]==0)?ID2EX[9:5]:ID2EX[4:0]);
    wire [31:0]FinalPCJump=( ID2EX[193]==1)?ALUResult:ID2EX[180:149];
    always@(posedge clk)
    begin
        if(flush | overflow_flush |interrupt |~rst_n )
            EX2MEM<=0;
        else
            EX2MEM<={ID2EX[194],ID2EX[137:106],ID2EX[193],FinalPCJump,ID2EX[148],ID2EX[143:140],ID2EX[139:138],PCBranch,neg,zero,pos,ALUResult,SrcBReg,Des_reg};
    end
    
    //MEM
    wire [31:0]MemOut;
	 
	 wire [31:0]ID2EXPCP4=ID2EX[137:106];
	 wire [31:0]EX2MEMPCBranch=EX2MEM[103:72]+32'd4;
	 
	 wire NZPBranch=(EX2MEM[109]&EX2MEM[71])|(EX2MEM[108]&EX2MEM[70] )|(EX2MEM[107]&EX2MEM[69] );
	 wire nextPlus4=(ID2EX[137:106]==EX2MEM[175:144]+32'd4);
	 wire pre_wrong1= (nextPlus4) & NZPBranch;//not branch but ought to
	 wire pre_wrong2= (~nextPlus4) & (~NZPBranch) &(EX2MEM[109]|EX2MEM[108]|EX2MEM[107]);//branch but not ought to

    assign PCSrc[0]=(EX2MEM[109]&EX2MEM[71])|(EX2MEM[108]&EX2MEM[70] )|(EX2MEM[107]&EX2MEM[69] );//if Branch 1
    assign PCSrc[1]=EX2MEM[110]|EX2MEM[143]|EX2MEM[176];//Jump or JumpR or Jal
    assign flush=PCSrc[1]|(PCSrc[0]&pre_wrong1&NZPBranch)| (pre_wrong2&(~NZPBranch)& (EX2MEM[109]|EX2MEM[108]|EX2MEM[107]));
    wire [4:0]address={2'd0,EX2MEM[68:37]>>2};
    //assign CPUData=(check==1)?MemOut:32'd0;
    data u_dataun(.a(address),.d(EX2MEM[36:5]),.clk(clk),.we(EX2MEM[106]),.spo(MemOut));
    wire tempJal=EX2MEM[176];
    wire [1:0]tempWB=EX2MEM[105:104];
    wire [31:0]tempPCPlus4=EX2MEM[175:144];
    always@(posedge clk)
    begin
        if(~rst_n )
			MEM2WB<=0;
		  else
			MEM2WB<={tempJal,tempPCPlus4,tempWB,MemOut[31:0],EX2MEM[68:37],EX2MEM[4:0]};
    end
     
    //WB
     assign r3_din=(MEM2WB[103]==1)?MEM2WB[102:71]:((MEM2WB[69]==1)?MEM2WB[68:37]:MEM2WB[36:5]);
endmodule
