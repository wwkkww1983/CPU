module EX_MEM(reset, clk, Stall, EX_Instruction, EX_MemRead, EX_MemWrite, EX_RegWrite, EX_MemtoReg, EX_Write_register, EX_ALU_out, EX_Databus2, EX_PC_plus_4, MovNoWrite_EX,
    MEM_MemRead, MEM_MemWrite, MEM_Instruction, MEM_RegWrite, MEM_MemtoReg, MEM_Write_register, MEM_ALU_out, MEM_Databus2, MEM_PC_plus_4, MovNoWrite_MEM);
    input reset, clk, EX_MemRead, EX_MemWrite, EX_RegWrite, MovNoWrite_EX;//EX_RegWrite这个是
    output reg MEM_MemRead, MEM_MemWrite, MEM_RegWrite, MovNoWrite_MEM;
    input [1:0] EX_MemtoReg;
    output reg [1:0] MEM_MemtoReg;
    input [4:0] EX_Write_register;
    output reg [4:0] MEM_Write_register;
    input [31:0] EX_ALU_out, EX_Databus2, EX_PC_plus_4 ,EX_Instruction;
    output reg [31:0] MEM_ALU_out, MEM_Databus2, MEM_PC_plus_4,MEM_Instruction;
    input Stall;

    always @(posedge reset or posedge clk)
        if (reset) begin
            MEM_Instruction <= 32'b0;
            MEM_MemRead <= 0;
            MEM_MemWrite <= 0;
            MEM_RegWrite <= 0;
            MEM_MemtoReg <= 2'b0;
            MEM_Write_register <= 5'b0;
            MEM_ALU_out <= 32'b0;
            MEM_Databus2 <= 32'b0;
            MEM_PC_plus_4 <= 32'b0;
            MovNoWrite_MEM <= 0;
        end
        else if (~Stall) begin
            MEM_Instruction <= EX_Instruction;
            MEM_MemRead <= EX_MemRead;
            MEM_MemWrite <= EX_MemWrite;
            MEM_RegWrite <= EX_RegWrite;
            MEM_MemtoReg <= EX_MemtoReg;
            MEM_Write_register <= EX_Write_register;
            MEM_ALU_out <= EX_ALU_out;
            MEM_Databus2 <= EX_Databus2;
            MEM_PC_plus_4 <= EX_PC_plus_4;
            MovNoWrite_MEM <= MovNoWrite_EX;
        end

endmodule