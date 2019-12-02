module Control(
    input [31:0] Instruction,
    input PC_31,
	output [2:0] PCSrc, 
    output [1:0] RegDst, 
    output RegWrite, 
	output MemRead, 
    output MemWrite, 
    output [1:0] MemtoReg,
    output ALUSrc1,
    output ALUSrc2,
    output ExtOp,
    output LuOp,
    output [3:0] ALUOp,
    output Exception,
    output DoRead);
    wire [5:0] OpCode, Function;
    assign OpCode = Instruction[31:26];//后5位
    assign Function = Instruction[5:0];//最后的
    
    assign Exception = ~PC_31 &
                    ~(OpCode == 6'h00 &(
                        Function == 6'h21 | Function == 6'h24 |
                        Function == 6'h08 | Function == 6'h0b |
                        Function == 6'h25 | Function == 6'h00 |
                        Function == 6'h03 | Function == 6'h02 |
                        Function == 6'h06 | Function == 6'h26 ))&
                    ~(  OpCode == 6'h09 | OpCode == 6'h0c |
                        OpCode == 6'h04 | OpCode == 6'h07 |
                        OpCode == 6'h02 | OpCode == 6'h03 |
                        OpCode == 6'h20 | OpCode == 6'h0f |
                        OpCode == 6'h23 | OpCode == 6'h0d |
                        OpCode == 6'h28 | OpCode == 6'h2b |
                        OpCode == 6'h0e | OpCode == 6'h05 );// 

    assign PCSrc[2:0]=Exception? 3'b101:
            ( OpCode == 6'h04 | OpCode == 6'h05 | OpCode == 6'h07 &Instruction[20:16] == 5'b00 )?
        3'b001:
            ( OpCode == 6'h03 | OpCode == 6'h02 )?
        3'b010:
            ( OpCode == 6'h00 & Function == 6'h08 )?
        3'b011: 3'b000;//J相关指令没有
    
    assign RegWrite = (Exception)?1:
            ( OpCode == 6'h04 | OpCode == 6'h07 |
            OpCode == 6'h05 | OpCode == 6'h02 |
            OpCode == 6'h2b | OpCode == 6'h28 |
            (OpCode == 6'h00 & Function == 6'h08))? 0:1;//要写进寄存器

    assign RegDst[1:0] = 
            ( Exception )? 2'b11:
            ( OpCode == 6'h09 | OpCode == 6'h0c |
            OpCode == 6'h20 | OpCode == 6'h0f |
            OpCode == 6'h23 | OpCode == 6'h0d |
            OpCode == 6'h28 | OpCode == 6'h0e )? 2'b01:
            ( OpCode == 6'h03 )? 2'b10: 2'b00;

    assign MemRead = ( OpCode == 6'h23 || OpCode==6'h20 )? 1:0;
    assign MemWrite = ( OpCode == 6'h2b || OpCode == 6'h28 )? 1:0;

    assign MemtoReg[1:0] = ( OpCode == 6'h03 | Exception )?2'b10:
        ( OpCode == 6'h23 | OpCode == 6'h20 )? 2'b01: 2'b00;

    assign ALUSrc1 = ( OpCode == 6'h00 & ( Function == 6'h00 | 
        Function == 6'h03 | Function == 6'h02 ))?1:0;
    assign ALUSrc2 = ( OpCode == 6'h09 | OpCode == 6'h0c | OpCode == 6'h20 |
            OpCode == 6'h23 | OpCode == 6'h0d | OpCode == 6'h28 | OpCode == 6'h0f |
            OpCode == 6'h2b | OpCode == 6'h0e )?1:0;
            
    assign ExtOp = (OpCode == 6'h09 | OpCode == 6'h0c | OpCode == 6'h0d |OpCode == 6'h0e | OpCode == 6'h04 | 
        OpCode == 6'h07 | OpCode == 6'h05)?0: 1;//是否0扩展unsigned

    assign LuOp = ( OpCode == 6'h0f )? 1: 0;

    assign ALUOp[2:0] = ( OpCode == 6'h00)?3'b100:
        (OpCode == 6'h04 | OpCode == 6'h05 | OpCode == 6'h06 | OpCode == 6'h07)? 3'b001:
        (OpCode == 6'h0c)? 3'b010:(OpCode == 6'h0d)? 3'b011:(OpCode == 6'h0e )? 3'b101: 3'b000;// 010是and,011是or,001是sub100是
    
    
    assign ALUOp[3] = OpCode[0];

    assign DoRead = (OpCode == 6'h02 | OpCode == 6'h03
        | OpCode == 6'h0f )? 0:1;

endmodule