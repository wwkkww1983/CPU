module ALU(in1, in2, ALUCtl, Sign, OpCode, out, Branch);
	input [31:0] in1, in2;//它是31位的!
	input [4:0] ALUCtl;
	input Sign;
    input [5:0] OpCode;//0-5之间的部分
	output [31:0] out;
	output Branch;
	
	assign Branch = 
	    (OpCode == 6'h04)? (out == 0): 
        (OpCode == 6'h05)? (out != 0): 
        (OpCode == 6'h07)? (out > 0): 0;// Branch的方法

    //beq bne blez bgtz bltz,只有这些是jump的指令,否则就是
	wire ss;
	assign ss = {in1[31], in2[31]};//是否是负的
	
	wire lt_31;
	assign lt_31 = (in1[30:0] < in2[30:0]);//是这一段比如
	
	wire lt_signed;
	assign lt_signed = (in1[31] ^ in2[31])? 
		((ss == 2'b01)? 0: 1): lt_31;
	
    assign out = 
        (ALUCtl == 5'b00000)? in1 & in2:
		(ALUCtl == 5'b00001)? in1 | in2:
		(ALUCtl == 5'b00010)? in1 + in2:
		(ALUCtl == 5'b00110)? in1 - in2:
		(ALUCtl == 5'b00111)? {31'h00000000, Sign? lt_signed: (in1 < in2)}:
		(ALUCtl == 5'b01100)? ~(in1 | in2):
		(ALUCtl == 5'b01101)? in1 ^ in2:
		(ALUCtl == 5'b10000)? (in2 << in1[4:0]):
		(ALUCtl == 5'b11000)? (in2 >> in1[4:0]):
		(ALUCtl == 5'b11001)? ({{32{in2[31]}}, in2} >> in1[4:0]):
        (ALUCtl == 5'b11010)? in1:
		32'h00000000;// 
        
endmodule