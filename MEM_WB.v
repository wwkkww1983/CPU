module MEM_WB(reset, clk, MEM_RegWrite, MEM_MemtoReg, MEM_Write_register, MEM_ALU_out, MEM_ReadData, MEM_PC_plus_4, 
    WB_RegWrite, WB_MemtoReg, WB_Write_register, WB_ALU_out, WB_ReadData, WB_PC_plus_4);
    input reset, clk, MEM_RegWrite;
    output reg WB_RegWrite;
    input [1:0] MEM_MemtoReg;
    output reg [1:0] WB_MemtoReg;
    input [4:0] MEM_Write_register;
    output reg [4:0] WB_Write_register;
    input [31:0] MEM_ALU_out, MEM_ReadData, MEM_PC_plus_4;
    output reg [31:0] WB_ALU_out, WB_ReadData, WB_PC_plus_4;

    always @(posedge reset or posedge clk)
        if (reset) begin//reset的话,所有的都是以不读写
            WB_RegWrite <= 0;
            WB_MemtoReg <= 2'b0;
            WB_Write_register <= 5'b0;
            WB_ALU_out <= 32'b0;
            WB_ReadData <= 32'b0;
            WB_PC_plus_4 <= 32'b0;
        end
        else begin//都按照原来的
            WB_RegWrite <= MEM_RegWrite;
            WB_MemtoReg <= MEM_MemtoReg;
            WB_Write_register <= MEM_Write_register;
            WB_ALU_out <= MEM_ALU_out;
            WB_ReadData <= MEM_ReadData;
            WB_PC_plus_4 <= MEM_PC_plus_4;
        end

endmodule