module ID_EX(reset, clk, Stall, Flush_IF_and_ID, 
    ID_PCSrc, ID_ALUOp, ID_Instruction, IF_PC, ID_PC_plus_4, ID_LU_out, ID_Databus1, ID_Databus2, 
    ID_ALUSrc1, ID_ALUSrc2, ID_MemRead, ID_MemWrite, ID_MemtoReg, ID_RegWrite, ID_RegDst, MovNoWrite_ID,
    EX_PCSrc, EX_ALUOp, EX_Instruction, EX_PC_plus_4, EX_LU_out, EX_Databus1, EX_Databus2, 
    EX_ALUSrc1, EX_ALUSrc2, EX_MemRead, EX_MemWrite, EX_MemtoReg, EX_RegWrite, EX_RegDst, MovNoWrite_EX);
    input reset, clk, Stall, Flush_IF_and_ID, ID_ALUSrc1, ID_ALUSrc2, ID_MemRead, ID_MemWrite, ID_RegWrite, MovNoWrite_ID;//这些是要写进去的
    output reg EX_ALUSrc1, EX_ALUSrc2, EX_MemRead, EX_MemWrite, EX_RegWrite, MovNoWrite_EX;
    input [1:0] ID_MemtoReg, ID_RegDst;
    output reg [1:0] EX_MemtoReg, EX_RegDst;
    input [2:0] ID_PCSrc;
    output reg [2:0] EX_PCSrc;
    input [3:0] ID_ALUOp;
    output reg [3:0] EX_ALUOp;
    input [31:0] ID_Instruction, ID_Databus1, ID_Databus2, ID_PC_plus_4, ID_LU_out, IF_PC;
    output reg [31:0] EX_Instruction, EX_Databus1, EX_Databus2, EX_PC_plus_4, EX_LU_out;

    always @(posedge reset or posedge clk)
        if (reset) begin
            EX_ALUSrc1 <= 0;//Ex_ALUSrc1是用来确定alu执行第一个参数的
            EX_ALUSrc2 <= 0;//Ex_ALUSrc2是用确定第二个参数的
            EX_MemRead <= 0;
            EX_MemWrite <= 0;//不写
            EX_RegWrite <= 0;
            EX_MemtoReg <= 2'b0;//读到1200号reg里面
            EX_RegDst <= 2'b0;//读到RegDst里面
            EX_PCSrc <= 2'b0;
            EX_ALUOp <= 4'b0;
            EX_Databus1 <= 32'b0;
            EX_Databus2 <= 32'b0;
            EX_LU_out <= 32'b0;// LU_out是用来做对外的
            EX_Instruction <= 32'h00000000;
            EX_PC_plus_4 <= 32'h80000008;//这个呢...+4下一条是下个地方的            Mov
            MovNoWrite_EX <= 0;
        end
        else if (Stall | Flush_IF_and_ID) begin
            EX_ALUSrc1 <= ID_ALUSrc1;//Stall或者Flush的话,那么Ex的就是ID的
            EX_ALUSrc2 <= ID_ALUSrc2;//
            EX_MemRead <= 0;//不读
            EX_MemWrite <= 0;//不写
            EX_RegWrite <= 0;//RegWrite这个阶段也不写
            MovNoWrite_EX <= MovNoWrite_ID;
            EX_MemtoReg <= ID_MemtoReg;//ID_MemtoReg
            EX_RegDst <= ID_RegDst;
            EX_PCSrc <= ID_PCSrc;
            EX_ALUOp <= ID_ALUOp;
            EX_Databus1 <= ID_Databus1;
            EX_Databus2 <= ID_Databus2;
            EX_LU_out <= ID_LU_out;//EX_LU_out就是ID_LU_out
            EX_Instruction <= ID_Instruction;//ID_Instruction
            EX_PC_plus_4 <= IF_PC;// 
        end
        else begin
            EX_ALUSrc1 <= ID_ALUSrc1;//ID_ALUSrc1
            EX_ALUSrc2 <= ID_ALUSrc2;//没有任何问题
            EX_MemRead <= ID_MemRead;//ID中间出来的,原封不动
            EX_MemWrite <= ID_MemWrite;
            EX_RegWrite <= ID_RegWrite;
            EX_MemtoReg <= ID_MemtoReg;//这里是这样的
            EX_RegDst <= ID_RegDst;
            EX_PCSrc <= ID_PCSrc;
            EX_ALUOp <= ID_ALUOp;
            EX_Databus1 <= ID_Databus1;
            EX_Databus2 <= ID_Databus2;
            EX_LU_out <= ID_LU_out;
            EX_Instruction <= ID_Instruction;
            EX_PC_plus_4 <= ID_PC_plus_4;
            MovNoWrite_EX <= MovNoWrite_ID;
        end

endmodule