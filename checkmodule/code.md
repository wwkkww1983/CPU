# 各部件接口与编码

## ALU

(in1, in2, ALUCtl, Sign, OpCode, out, Branch)
- in1, in2 （输入的两操作数）
- ALUCtl （ALU的控制信号，用于控制ALU的行为）
    - 5'b00000 &
    - 5'b00001 |
    - 5'b00010 +
    - 5'b00110 -
    - 5'b00111 in1 < in2 可设置是否为带符号数
    - 5'b01100 或非
    - 5'b01101 ^
    - 5'b10000 in2 << in1[4:0] 左移
    - 5'b11000 in2 >> in1[4:0] 右移
    - 5'b11001 带符号右移
    - 5'b11010 in1
    - 0
- Sign （当前操作数是否为带符号数）
- out （ALU操作后的结果）
- Branch （是否分支跳转）
    - OpCode == 6'h04 BEQ 两操作数相等
    - OpCode == 6‘h05 BNE 两操作数不等
    - OpCode == 6'h07 BGTZ in1 > in2


