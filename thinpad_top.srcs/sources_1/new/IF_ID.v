module IF_ID(reset, clk, Flush, Stall, IF_PC, IF_PC_plus_4, IF_Instruction, ID_Instruction, ID_PC_plus_4);
    input reset, clk, Flush, Stall;
    input [31:0] IF_PC, IF_PC_plus_4, IF_Instruction;
    output reg [31:0] ID_Instruction, ID_PC_plus_4;

    /*阻塞时，id指令保持不变，不阻塞时，接收来自if的指令*/

    always @(posedge reset or posedge clk)//reset的时候
        if (reset) begin//reset的时候
            ID_Instruction <= 32'b0;//ID_Instruction是32'b0指令为b0(全是0)
            ID_PC_plus_4 <= 32'h80000000;//下一条地址是到reset空间
        end
        else if (Flush) begin//Flush:在预测失败的时候打出GG
            ID_Instruction <= 32'b0;//有内鬼,终止结算!
            ID_PC_plus_4 <= IF_PC;
        end
        else if (Stall) begin
            ID_Instruction <= ID_Instruction;//
            ID_PC_plus_4 <= ID_PC_plus_4;//stall的话插入一个气泡
        end
        else begin
            ID_Instruction <= IF_Instruction;//下一阶段的指令
            ID_PC_plus_4 <= IF_PC_plus_4;//
        end

endmodule