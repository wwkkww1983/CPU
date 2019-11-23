module ALUControl(ALUOp, Funct, ALUCtl, Sign); 
	input [3:0] ALUOp;//ALU的操作数
	input [5:0] Funct;//函数
	output reg [4:0] ALUCtl;//ALUCtl的出寄存器
	output Sign;//output的sign
	
	parameter aluAND = 5'b00000;
	parameter aluOR  = 5'b00001;
	parameter aluADD = 5'b00010;
	parameter aluSUB = 5'b00110;
	parameter aluXOR = 5'b01101;
	parameter aluSLL = 5'b10000;
	parameter aluSRL = 5'b11000;
	parameter aluSRA = 5'b11001;
    parameter aluMOV = 5'b11010;
	
	assign Sign = (ALUOp[2:0] == 3'b010)? ~Funct[0]: ~ALUOp[3];
	
	reg [4:0] aluFunct;
	always @(*)
		case (Funct)
			6'b00_0000: aluFunct <= aluSLL;// 00_0000是 <= aluSLL操作的类型
			6'b00_0010: aluFunct <= aluSRL;
			6'b00_0011: aluFunct <= aluSRA;
			6'b00_0110: aluFunct <= aluSRL;
			6'b00_1011: aluFunct <= aluMOV;
			6'b00_0110: aluFunct <= aluSRL;
			6'b10_0000: aluFunct <= aluADD;
			6'b10_0001: aluFunct <= aluADD;
			6'b10_0010: aluFunct <= aluSUB;
			6'b10_0011: aluFunct <= aluSUB;
			6'b10_0100: aluFunct <= aluAND;
			6'b10_0101: aluFunct <= aluOR;
			6'b10_0110: aluFunct <= aluXOR;

			default: aluFunct <= aluADD;
		endcase
	
	always @(*)
		case (ALUOp[2:0])
			3'b000: ALUCtl <= aluADD;//add是最后的
			3'b001: ALUCtl <= aluSUB;//所有b相关的指令都作为sub
			3'b010: ALUCtl <= aluOR;//后5位没东西
			3'b011: ALUCtl <= aluXOR;//异或
			3'b100: ALUCtl <= aluFunct;//Alu操作100是上面的Function来判断
			default: ALUCtl <= aluADD;
		endcase

endmodule