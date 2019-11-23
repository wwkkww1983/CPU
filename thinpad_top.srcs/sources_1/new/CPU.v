module CPU(reset, clk, seg, AN, baseram_data, baseram_addr, baseram_be, baseram_ce, baseram_oe, baseram_we);
    input reset, sysclk;
    //output[6:0] seg;
    //output[3:0] AN;
    inout wire[31:0] baseram_data;
    output wire[19:0] baseram_addr;
    output wire[3:0] baseram_be;
    output wire baseram_ce;
    output wire baseram_oe;
    output wire baseram_we;
    wire IRQ, Exception, Branch, Stall, Flush_IF, Flush_IF_and_ID;
    wire ExtOp, LuOp, MOV;
    wire ID_MemRead, EX_MemRead, MEM_MemRead;
    wire ID_MemWrite, EX_MemWrite, MEM_MemWrite;
    wire ID_RegWrite, EX_RegWrite, MEM_RegWrite, WB_RegWrite;
    wire ID_ALUSrc1, EX_ALUSrc1, ID_ALUSrc2, EX_ALUSrc2;
    wire [2:0] ID_PCSrc, EX_PCSrc;
    wire [1:0] ID_RegDst, EX_RegDst;
    wire [1:0] ID_MemtoReg, EX_MemtoReg, MEM_MemtoReg, WB_MemtoReg;
	wire [3:0] ID_ALUOp, EX_ALUOp;
    wire [31:0] PC_next, IF_PC, IRQ_PC;//
    reg [31:0] PC = 32'h00000000;//PC这个是下一条指令地址?

    reg [3:0] cnt = 0;
    reg clk = 0;
    always @(posedge sysclk)
    begin
        cnt <= cnt + 1;
        if (cnt == 10) begin
            clk <= ~clk;//(1/10)分频
            cnt <= 0;
        end
    end

    assign IF_PC = Stall? PC: PC_next;//如果是stall就是PC,是PC_NEXT

    reg ce;

    always @(posedge clk) begin
        if(reset) begin
            ce <= 1'b0;
        end else begin
            ce <= 1'b1;
        end
    end


    always @(posedge reset or posedge clk)//不reset的话,就继续
        if (reset) begin
			PC <= 32'h00000000;//reset则回到最开始的位置
		end else begin
			PC <= IF_PC;
        end
    end

    wire [31:0] IF_PC_4, ID_PC_4, EX_PC_4, MEM_PC_4, WB_PC_4;
    assign IF_PC_4[30:0] = PC[30:0] +3'd4;
    assign IF_PC_4[31] = PC[31];
    wire [31:0] IF_Instruction, ID_Instruction, EX_Instruction;
    
    //TODO:取指相关
    Inst_Mem Inst_Mem(.ce(ce), .Address(PC),.Instruction(IF_Instruction),.baseram_data(baseram_data),
        .baseram_addr(baseram_addr),.baseram_be(baseram_be),.baseram_oe(baseram_oe),.baseram_we(baseram_we)
    );

    assign Flush_IF = (ID_PCSrc == 3'b010) | (ID_PCSrc == 3'b011) | IRQ;// j类和r
    assign Flush_IF_and_ID = Branch & ( EX_PCSrc == 3'b001 ) & !IRQ;//刷新清零,下一条读进来的指令
    
    assign Stall = EX_MemRead &
        ((EX_Instruction[20:16] == ID_Instruction[25:21]) |
        (EX_Instruction[20:16] == ID_Instruction[20:16]));//数据冲突需要stall一个周期
    IF_ID IF_ID(.reset(reset), .clk(clk), .Flush( Flush_IF || Flush_IF_and_ID), .Stall(Stall),
         .IF_PC(IF_PC), .IF_PC_plus_4(IF_PC_4), .IF_Instruction(IF_Instruction), .ID_Instruction(ID_Instruction), 
         .ID_PC_Plus_4(ID_PC_4));

    Control control1(.Instruction(ID_Instruction), .PC_31(ID_PC_4[31]), .IRQ(IRQ), 
		.PCSrc(ID_PCSrc), .RegDst(ID_RegDst), .RegWrite(ID_RegWrite), 
		.MemRead(ID_MemRead), .MemWrite(ID_MemWrite), .MemtoReg(ID_MemtoReg),
		.ALUSrc1(ID_ALUSrc1), .ALUSrc2(ID_ALUSrc2), .ExtOp(ExtOp), .LuOp(LuOp), .ALUOp(ID_ALUOp), .MOV(MOV), .Exception(Exception));

    wire [31:0] ID_Data1, ID_Data2, ID_Data3;
    wire [31:0] EX_Data1, EX_Data2, EX_Data1w, EX_Data2w, EX_Data3;
    wire [31:0] MEM_Data2, MEM_Data3;

    wire [4:0] EX_Registerw, MEM_Registerw, WB_Registerw;
    RegisterFile reg( .reset(reset), .clk(clk), .RegWrite(WB_RegWrite), .Read_register1(ID_Instruction[25:21]), 
        .Read_register2(ID_Instruction[20:16]), .Write_register(WB_Registerw), 
        .Write_data(ID_Data3), .Read_data1(ID_Data1), .Read_data2(ID_Data2), .DoRead(ID_PCSrc == 3'b010));

    wire [31:0] Ext_out;
    assign Ext_out = {ExtOp? {16{ID_Instruction[15]}}: 16'h0000, ID_Instruction[15:0]};

    wire [31:0] ID_LU_out, EX_LU_out;// lui 处理
	assign ID_LU_out = LuOp? {ID_Instruction[15:0], 16'h0000}: Ext_out;// 扩展后15位

    wire [31:0] Jump_target;
	assign Jump_target = {ID_PC_4[31:28], ID_Instruction[25:0], 2'b00};//这个是在jump到的地方合起来的

    ID_EX ID_EX(reset, clk, Stall, Flush_IF_and_ID,
        ID_PCSrc, ID_ALUOp, ID_Instruction,IF_PC, (IRQ? IRQ_PC: ID_PC_4),ID_LU_out, EX_Data1w, EX_Data2w,
        ID_ALUSrc1, ID_ALUSrc2, ID_MemRead,ID_MemWrite, ID_MemtoReg, ID_RegWrite, ID_RegDst, 
        EX_PCSrc, EX_ALUOp, EX_Instruction, EX_PC_4, EX_LU_out, EX_Data1, EX_Data2, 
        EX_ALUSrc1, EX_ALUSrc2, EX_MemRead, EX_MemWrite, EX_MemtoReg, EX_RegWrite, EX_RegDst);

    wire [4:0] ALUToken;
    wire Signed;
    ALUControl aluctrl(.ALUOp(EX_ALUOp), .Funct(EX_Instruction[5:0]), .ALUCtl(ALUToken), .Sign(Signed));

    wire[31:0] ALU_in1, ALU_in2, EX_ALU_out, MEM_ALU_out, WB_ALU_out;

    assign ALU_in1 = EX_ALUSrc1 ?{27'h00000, EX_Instruction[10:6]}: MOV? 32'h00000000 : EX_Data1;
    assign ALU_in2 = EX_ALUSrc2? EX_LU_out: EX_Data2;
    ALU alu(.in1(ALU_in1), .in2(ALU_in2), .ALUCtl(ALUToken), .Sign(Signed), .OpCode(EX_Instruction[31:26]), .out(EX_ALU_out), .Branch(Branch));

    wire [31:0] Branch_target;
	assign Branch_target = Branch? EX_PC_4 + {EX_LU_out[29:0], 2'b00}: EX_PC_4;// EX 阶段

    assign PC_net = (EX_PCSrc == 3'b001 & Branch)? Branch_target:
            (ID_PCSrc == 3'b000 | ID_PCSrc == 3'b001)? IF_PC_4:
            (ID_PCSrc == 3'b010)? Jump_target:
            (ID_PCSrc == 3'b011)? EX_Data1w:
            (ID_PCSrc == 3'b100)? 32'h80000004:
            (ID_PCSrc == 3'b101)? 32'h80000008 : IF_PC_4;

    assign EX_Registerw = 
        (EX_RegDst == 2'b00)? EX_Instruction[15:11]:
        (EX_RegDst == 2'b01)? EX_Instruction[20:16]:
        (EX_RegDst == 2'b10)? 5'd31:
        (EX_RegDst == 2'b11)? 5'd26: 5'd26;//要得到写的内容!


    wire MovNoWrite = ( MOV & ALU_in2 == 32'h00000000 );//是MOVN况且还不用MOV

    EX_MEM EX_MEM(reset, clk, EX_MemRead, EX_MemWrite, EX_RegWrite, EX_MemtoReg, EX_Registerw, EX_ALU_out, EX_Data2, EX_PC_4, 
        MEM_MemRead, MEM_MemWrite, MEM_RegWrite, MEM_MemtoReg, MEM_Registerw, MEM_ALU_out, MEM_Data2, MEM_PC_4, MovNoWrite);    
    
    wire [31:0] MEM_ReadData, WB_ReadData;
    //TODO: writeBack
    DataMemory DataMemory(.reset(reset), .clk(clk), .ce(ce), .Address(MEM_ALU_out), .Write_data(MEM_Data2), .Read_data(MEM_ReadData),
        .Op(EX_Instruction[31:26]), .MemRead()
        );

    MEM_WB MEM_WB(reset, clk, MEM_RegWrite, MEM_MemtoReg, MEM_Registerw, MEM_ALU_out, MEM_ReadData, MEM_PC_4, 
        WB_RegWrite, WB_MemtoReg, WB_Registerw, WB_ALU_out, WB_ReadData, WB_PC_4);

    assign ID_Data3 = (WB_MemtoReg == 2'b00)? WB_ALU_out: (WB_MemtoReg == 2'b01)? WB_ReadData: {1'b0, WB_PC_4[30:0]};//注意循环内存!
    assign EX_Data3 = (EX_MemtoReg == 2'b00)? EX_ALU_out: {1'b0, EX_PC_4[30:0]};
    assign MEM_Data3 = (MEM_MemtoReg == 2'b00)? MEM_ALU_out: (MEM_MemtoReg == 2'b01)? MEM_ReadData: {1'b0, MEM_PC_4[30:0]};

    wire [1:0] ForwardA, ForwardB;//按照ppt上的算法处理冒险

    assign ForwardA = (~(MovNoWrite) & EX_RegWrite & (EX_Registerw != 0 ) & (EX_Registerw == ID_Instruction[25:21]))? 2'b01:
                    (MEM_RegWrite & (MEM_Registerw != 0) & (MEM_Registerw == ID_Instruction[25:21]))? 2'b10:
                    (WB_RegWrite & (WB_Registerw != 0) & (WB_Registerw == ID_Instruction[25:21]))? 2'b11: 2'b00;

    assign ForwardB = ( ~(MovNoWrite) & EX_RegWrite & (EX_Registerw != 0) & (EX_Registerw == ID_Instruction[20:16]))? 2'b01:
                    (MEM_RegWrite &(MEM_Registerw != 0 )& (MEM_Registerw == ID_Instruction[20:16]))? 2'b10:
                    (WB_RegWrite & (WB_Registerw != 0) & (MEM_Registerw == ID_Instruction[20:16]))? 2'b11: 2'b00;

    assign EX_Data1w = 
        (ForwardA == 2'b01)? EX_Data3: 
        (ForwardA == 2'b10)? MEM_Data3: //前两条指令冲突的话(已经写到ID里面去了)
        (ForwardA == 2'b11)? ID_Data3: ID_Data1;//Ex_Databus
    assign EX_Data2w = 
        (ForwardB == 2'b01)? EX_Data3: 
        (ForwardB == 2'b10)? MEM_Data3: 
        (ForwardB == 2'b11)? ID_Data3: ID_Data2;//ForwardB是冒险的地方在哪儿

endmodule
        

