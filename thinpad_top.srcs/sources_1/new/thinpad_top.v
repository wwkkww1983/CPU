`default_nettype none
module thinpad_top(
    input wire clk_50M,           //50MHz 时钟输入
    input wire clk_11M0592,       //11.0592MHz 时钟输入（备用，可不用）

    input wire clock_btn,         //BTN5手动时钟按钮�?关，带消抖电路，按下时为1
    input wire reset_btn,         //BTN6手动复位按钮�?关，带消抖电路，按下时为1

    input  wire[3:0]  touch_btn,  //BTN1~BTN4，按钮开关，按下时为1
    input  wire[31:0] dip_sw,     //32位拨码开关，拨到“ON”时�?1
    output wire[15:0] leds,       //16位LED，输出时1点亮
    output wire[7:0]  dpy0,       //数码管低位信号，包括小数点，输出1点亮
    output wire[7:0]  dpy1,       //数码管高位信号，包括小数点，输出1点亮

    //CPLD串口控制器信�?
    output wire uart_rdn,         //读串口信号，低有�?
    output wire uart_wrn,         //写串口信号，低有�?
    input wire uart_dataready,    //串口数据准备�?
    input wire uart_tbre,         //发�?�数据标�?
    input wire uart_tsre,         //数据发�?�完毕标�?
    
    //BaseRAM信号
    inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共�?
    output wire[19:0] base_ram_addr, //BaseRAM地址
    output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持�?0
    output wire base_ram_ce_n,       //BaseRAM片�?�，低有�?
    output wire base_ram_oe_n,       //BaseRAM读使能，低有�?
    output wire base_ram_we_n,       //BaseRAM写使能，低有�?

    //ExtRAM信号
    inout wire[31:0] ext_ram_data,  //ExtRAM数据
    output wire[19:0] ext_ram_addr, //ExtRAM地址
    output wire[3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持�?0
    output wire ext_ram_ce_n,       //ExtRAM片�?�，低有�?
    output wire ext_ram_oe_n,       //ExtRAM读使能，低有�?
    output wire ext_ram_we_n,       //ExtRAM写使能，低有�?

    //直连串口信号
    output wire txd,  //直连串口发�?�端
    input  wire rxd,  //直连串口接收�?

    //Flash存储器信号，参�?? JS28F640 芯片手册
    output wire [22:0]flash_a,      //Flash地址，a0仅在8bit模式有效�?16bit模式无意�?
    inout  wire [15:0]flash_d,      //Flash数据
    output wire flash_rp_n,         //Flash复位信号，低有效
    output wire flash_vpen,         //Flash写保护信号，低电平时不能擦除、烧�?
    output wire flash_ce_n,         //Flash片�?�信号，低有�?
    output wire flash_oe_n,         //Flash读使能信号，低有�?
    output wire flash_we_n,         //Flash写使能信号，低有�?
    output wire flash_byte_n,       //Flash 8bit模式选择，低有效。在使用flash�?16位模式时请设�?1

    //USB 控制器信号，参�?? SL811 芯片手册
    output wire sl811_a0,
    //inout  wire[7:0] sl811_d,     //USB数据线与网络控制器的dm9k_sd[7:0]共享
    output wire sl811_wr_n,
    output wire sl811_rd_n,
    output wire sl811_cs_n,
    output wire sl811_rst_n,
    output wire sl811_dack_n,
    input  wire sl811_intrq,
    input  wire sl811_drq_n,

    //网络控制器信号，参�?? DM9000A 芯片手册
    output wire dm9k_cmd,          
    inout  wire[15:0] dm9k_sd,
    output wire dm9k_iow_n,
    output wire dm9k_ior_n,
    output wire dm9k_cs_n,
    output wire dm9k_pwrst_n,
    input  wire dm9k_int,

    //图像输出信号
    output wire[2:0] video_red,    //红色像素�?3�?
    output wire[2:0] video_green,  //绿色像素�?3�?
    output wire[1:0] video_blue,   //蓝色像素�?2�?
    output wire video_hsync,       //行同步（水平同步）信�?
    output wire video_vsync,       //场同步（垂直同步）信�?
    output wire video_clk,         //像素时钟输出
    output wire video_de           //行数据有效信号，用于区分消隐�?
);

/* =========== Demo code begin =========== */

// PLL分频示例
wire locked, clk_10M, clk_20M;
// pll_example clock_gen 
//  (
//   // Clock out ports
//   .clk_out1(clk_10M), // 时钟输出1，频率在IP配置界面中设�?
//   .clk_out2(clk_20M), // 时钟输出2，频率在IP配置界面中设�?
//   // Status and control signals
//   .reset(reset_btn), // PLL复位输入
//   .locked(locked), // 锁定输出�?"1"表示时钟稳定，可作为后级电路复位
//  // Clock in ports
//   .clk_in1(clk_50M) // 外部时钟输入
//  );

reg reset_of_clk10M;
// 异步复位，同步释�?
always@(posedge clk_10M or negedge locked) begin
    if(~locked) reset_of_clk10M <= 1'b1;
    else        reset_of_clk10M <= 1'b0;
end

always@(posedge clk_10M or posedge reset_of_clk10M) begin
    if(reset_of_clk10M)begin
        // Your Code
    end
    else begin
        // Your Code
    end
end

// 不使用内存�?�串口时，禁用其使能信号

// assign base_ram_be_n = 4'b0000;
// assign ext_ram_be_n = 4'b0000;


// 数码管连接关系示意图，dpy1同理
// p=dpy0[0] // ---a---
// c=dpy0[1] // |     |
// d=dpy0[2] // f     b
// e=dpy0[3] // |     |
// b=dpy0[4] // ---g---
// a=dpy0[5] // |     |
// f=dpy0[6] // e     c
// g=dpy0[7] // |     |
//           // ---d---  p

// 7段数码管译码器演示，将number�?16进制显示在数码管上面
reg[7:0] number;
SEG7_LUT segL(.oSEG1(dpy0), .iDIG(number[3:0])); //dpy0是低位数码管
SEG7_LUT segH(.oSEG1(dpy1), .iDIG(number[7:4])); //dpy1是高位数码管

reg[15:0] led_bits;
assign leds = led_bits;

integer state, cnt;
//reg my_uart_rdn;
//reg my_uart_wrn;
// reg my_base_ram_ce_n;      //BaseRAM片�?�，低有�?
// reg my_base_ram_oe_n;       //BaseRAM读使能，低有�?
// reg my_base_ram_we_n;  
//assign uart_rdn = my_uart_rdn;
//assign uart_wrn = my_uart_wrn;
// assign base_ram_ce_n = my_base_ram_ce_n;//
// assign base_ram_oe_n = my_base_ram_oe_n;
// assign base_ram_we_n = my_base_ram_we_n;
// reg base_ram_data_z;

reg[19:0] addr;
reg[7:0] data;

wire Exception, Branch, Flush_IF, Flush_IF_and_ID;
wire ExtOp, LuOp;
wire ID_MemRead, EX_MemRead, MEM_MemRead;
wire ID_MemWrite, EX_MemWrite, MEM_MemWrite;
wire ID_RegWrite, EX_RegWrite, MEM_RegWrite, WB_RegWrite;
wire ID_ALUSrc1, EX_ALUSrc1, ID_ALUSrc2, EX_ALUSrc2;
wire [2:0] ID_PCSrc, EX_PCSrc;
wire [1:0] ID_RegDst, EX_RegDst;
wire [1:0] ID_MemtoReg, EX_MemtoReg, MEM_MemtoReg, WB_MemtoReg;
wire [3:0] ID_ALUOp, EX_ALUOp;
wire [31:0] PC_next;//
wire [31:0] IF_PC;
reg [31:0] PC = 32'h80000000;//PC这个是下�?条指令地�??
wire Stall, Stall1, Stall2, Stall3;

reg [3:0] count = 0;
reg clk = 0;

wire reset;

wire DoRead;

assign reset = reset_btn;

always @(posedge clk_11M0592)
    begin
        clk <= ~clk;
    end


assign IF_PC = (Stall == 0)? PC_next: 
            (ID_PCSrc == 3'b010)? PC_next: PC;//如果是stall就是PC,是PC_NEXT

reg ce = 0;



always @(*) begin
    if(reset) begin
        ce <= 1'b0;
    end else begin
        ce <= 1'b1;
    end
end

always @(posedge reset or posedge clk)//不reset的话,就继�?
    if (reset) begin
		PC <= 32'h80000000;//reset则回到最�?始的位置
	end else begin
		PC <= IF_PC;
    end


wire [31:0] IF_PC_4, ID_PC_4, EX_PC_4, MEM_PC_4, WB_PC_4;
assign IF_PC_4[30:0] = PC[30:0] +3'd4;
assign IF_PC_4[31] = PC[31];
wire [31:0] IF_Instruction, ID_Instruction, EX_Instruction ,MEM_Instruction;
wire LastFlush;
wire ok;
reg stals = 0;
reg stall2 = 0;

    assign Flush_IF = (ID_PCSrc == 3'b010) | (ID_PCSrc == 3'b011) ;// j类和r
    assign Flush_IF_and_ID = Branch & ( EX_PCSrc == 3'b001 ) & (~LastFlush);// 刷新清零,下一条读进来的指令刷新清�?

    assign Stall2 = (MEM_MemRead | MEM_MemWrite);
    assign Stall3 = ~ok;
    assign Stall1 = (EX_MemRead &
        ((EX_Instruction[20:16] == ID_Instruction[25:21]) |
        (EX_Instruction[20:16] == ID_Instruction[20:16]))) | (ID_MemRead & EX_MemWrite) |(ID_MemWrite & EX_MemWrite);

    assign Stall = Stall1 | Stall2 | Stall3;// 数据冲突�?要stall�?个周�?

    IF_ID IF_ID(.reset(reset), .clk(clk), .Flush( Flush_IF || Flush_IF_and_ID), .Stall(Stall),
         .IF_PC(IF_PC), .IF_PC_plus_4(IF_PC_4), .IF_Instruction(IF_Instruction), .ID_Instruction(ID_Instruction), .ID_PC_plus_4(ID_PC_4));

    Control control1(.Instruction(ID_Instruction), .PC_31(ID_PC_4[31]),
		.PCSrc(ID_PCSrc), .RegDst(ID_RegDst), .RegWrite(ID_RegWrite), 
		.MemRead(ID_MemRead), .MemWrite(ID_MemWrite), .MemtoReg(ID_MemtoReg),
		.ALUSrc1(ID_ALUSrc1), .ALUSrc2(ID_ALUSrc2), .ExtOp(ExtOp), .LuOp(LuOp), .ALUOp(ID_ALUOp), .Exception(Exception),
        .DoRead(DoRead));

    wire [31:0] ID_Data1, ID_Data2, ID_Data3;
    wire [31:0] EX_Data1, EX_Data2, EX_Data1w, EX_Data2w, EX_Data3;
    wire [31:0] MEM_Data2, MEM_Data3;

    wire [4:0] EX_Registerw, MEM_Registerw, WB_Registerw;
    RegisterFile register( .reset(reset), .clk(clk), .RegWrite(WB_RegWrite), .Read_register1(ID_Instruction[25:21]), 
        .Read_register2(ID_Instruction[20:16]), .Write_register(WB_Registerw), 
        .Write_data(ID_Data3), .Read_data1(ID_Data1), .Read_data2(ID_Data2), .DoRead(DoRead));

    wire [31:0] Ext_out;
    assign Ext_out = {ExtOp? 16'h0000 : {16{ID_Instruction[15]}}, ID_Instruction[15:0]};

    wire [31:0] ID_LU_out, EX_LU_out;// lui 处理
	assign ID_LU_out = LuOp? {ID_Instruction[15:0], 16'h0000}: Ext_out;// 扩展�?15�?

    wire [31:0] Jump_target;
	assign Jump_target = {ID_PC_4[31:28], ID_Instruction[25:0], 2'b00};//这个是在jump到的地方合起来的

    wire MovNoWrite_ID, MovNoWrite_EX, MovNoWrite_MEM, MovNoWrite_WB;

    assign MovNoWrite_ID = (ID_Instruction[31:26] == 6'h00 & ID_Instruction[5:0] == 6'h0b & EX_Data2w == 32'h00000000 );

    ID_EX ID_EX(reset, clk, Stall, Flush_IF_and_ID, LastFlush,
        ID_PCSrc, ID_ALUOp, ID_Instruction, ID_PC_4 ,ID_LU_out, EX_Data1w, EX_Data2w,
        ID_ALUSrc1, ID_ALUSrc2, ID_MemRead, ID_MemWrite, ID_MemtoReg, ID_RegWrite, ID_RegDst, MovNoWrite_ID,
        EX_PCSrc, EX_ALUOp, EX_Instruction, EX_PC_4, EX_LU_out, EX_Data1, EX_Data2, 
        EX_ALUSrc1, EX_ALUSrc2, EX_MemRead, EX_MemWrite, EX_MemtoReg, EX_RegWrite, EX_RegDst, MovNoWrite_EX);

    wire [4:0] ALUToken;
    wire Signed;
    ALUControl aluctrl(.ALUOp(EX_ALUOp), .Funct(EX_Instruction[5:0]), .ALUCtl(ALUToken), .Sign(Signed));

    wire[31:0] ALU_in1, ALU_in2, EX_ALU_out, MEM_ALU_out, WB_ALU_out;

    assign ALU_in1 = EX_ALUSrc1 ?{27'h00000, EX_Instruction[10:6]}: EX_Data1;
    assign ALU_in2 = EX_ALUSrc2? EX_LU_out: EX_Data2;
    ALU alu(.in1(ALU_in1), .in2(ALU_in2), .ALUCtl(ALUToken), .Sign(Signed), .OpCode(EX_Instruction[31:26]), .out(EX_ALU_out), .Branch(Branch));

    wire [31:0] Branch_target;
    wire last_stall2;
    assign last_stall2 = stall2;
    wire [31:0] Last_PC_next, Last_Jump, jtarget;
    reg [31:0] Last_PC;
    assign Last_PC_next = Last_PC;
	assign Branch_target = Branch? EX_PC_4 + {EX_LU_out[29:0], 2'b00}: EX_PC_4;// EX 阶段

    assign PC_next = (EX_PCSrc == 3'b001 & Branch)? Branch_target:
            (ID_PCSrc == 3'b000 | ID_PCSrc == 3'b001)? IF_PC_4:
            (ID_PCSrc == 3'b010)? Jump_target:
            (ID_PCSrc == 3'b011)? EX_Data1w:
            (ID_PCSrc == 3'b100)? 32'h80000004:
            (ID_PCSrc == 3'b101)? 32'h80000008 : IF_PC_4;

    assign EX_Registerw = 
        (EX_RegDst == 2'b00)? EX_Instruction[15:11]:
        (EX_RegDst == 2'b01)? EX_Instruction[20:16]:
        (EX_RegDst == 2'b10)? 5'd31://如果�? 的话，那�?
        (EX_RegDst == 2'b11)? 5'd26: 5'd26;//要得到写的内�?!
    EX_MEM EX_MEM(reset, clk, Stall3, EX_Instruction, EX_MemRead, EX_MemWrite, EX_RegWrite, EX_MemtoReg, EX_Registerw, EX_ALU_out, EX_Data2, EX_PC_4,MovNoWrite_EX, 
        MEM_MemRead, MEM_MemWrite, MEM_Instruction, MEM_RegWrite, MEM_MemtoReg, MEM_Registerw, MEM_ALU_out, MEM_Data2, MEM_PC_4, MovNoWrite_MEM);    
    
    wire [31:0] MEM_ReadData, WB_ReadData;

    wire MovNoWrite;
    assign MovNoWrite = (MEM_Instruction[31:26] == 6'h00 & MEM_Instruction[5:0] == 6'h0b & MEM_Data2 == 32'h00000000 );

    MEM_WB MEM_WB(reset, clk, MEM_RegWrite, MEM_MemtoReg, MEM_Registerw, MEM_ALU_out, MEM_ReadData, MEM_PC_4, MovNoWrite_MEM,
        WB_RegWrite, WB_MemtoReg, WB_Registerw, WB_ALU_out, WB_ReadData, WB_PC_4, MovNoWrite_WB);

    assign ID_Data3 = (WB_MemtoReg == 2'b00)? WB_ALU_out: (WB_MemtoReg == 2'b01)? WB_ReadData: {WB_PC_4[31:0]};//注意循环内存!
    assign EX_Data3 = (EX_MemtoReg == 2'b00)? EX_ALU_out: {1'b0, EX_PC_4[30:0]};
    assign MEM_Data3 = (MEM_MemtoReg == 2'b00)? MEM_ALU_out: (MEM_MemtoReg == 2'b01)? MEM_ReadData: {MEM_PC_4[31:0]};

    wire [1:0] ForwardA, ForwardB;//按照ppt上的算法处理冒险

    assign ForwardA = (~(MovNoWrite_EX) & EX_RegWrite & (EX_Registerw != 0 ) & (EX_Registerw == ID_Instruction[25:21]))? 2'b01:
                    (~(MovNoWrite_MEM) & MEM_RegWrite & (MEM_Registerw != 0) & (MEM_Registerw == ID_Instruction[25:21]))? 2'b10:
                    (~(MovNoWrite_WB) & WB_RegWrite & (WB_Registerw != 0) & (WB_Registerw == ID_Instruction[25:21]))? 2'b11: 2'b00;

    assign ForwardB = ( ~(MovNoWrite_EX) & EX_RegWrite & (EX_Registerw != 0) & (EX_Registerw == ID_Instruction[20:16]))? 2'b01:
                    ( ~(MovNoWrite_MEM) & MEM_RegWrite &(MEM_Registerw != 0 )& (MEM_Registerw == ID_Instruction[20:16]))? 2'b10:
                    (~(MovNoWrite_WB) & WB_RegWrite & (WB_Registerw != 0) & (WB_Registerw == ID_Instruction[20:16]))? 2'b11: 2'b00;

    assign EX_Data1w = 
        (ForwardA == 2'b01)? EX_Data3: 
        (ForwardA == 2'b10)? MEM_Data3: //前两条指令冲突的�?(已经写到ID里面去了)
        (ForwardA == 2'b11)? ID_Data3: ID_Data1;//Ex_Databus
    assign EX_Data2w = 
        (ForwardB == 2'b01)? EX_Data3: 
        (ForwardB == 2'b10)? MEM_Data3: 
        (ForwardB == 2'b11)? ID_Data3: ID_Data2;//ForwardB是冒险的地方在哪�?

    /*ramprep ramprep( .mem_addr(MEM_ALU_out), .mem_ce(MEM_MemRead | MEM_MemWrite),.inst_addr(PC),.inst_ce(ce),
        .base_chosen_inst(base_ram_inst), .ext_chosen_inst(ext_ram_inst), .base_chosen_mem(base_ram_mem), .ext_chosen_mem(ext_ram_mem),
        .RW(rw));*/

    /*wire [3:0] sel;

    assign sel = (MEM_Instruction[31:26] != 6'b101000) ? 4'b1111: 
        (MEM_ALU_out[1:0] == 2'b11)? 4'b1000: 
        (MEM_ALU_out[1:0] == 2'b10)? 4'b0100:
        (MEM_ALU_out[1:0] == 2'b01)? 4'b0010: 4'b0001;*/
wire [31:0] MEM_DataS,  mem_addr;

reg [31:0] Last_ReadData, Last_ALU_out, Last_Data2;

// always @(posedge clk) begin
    
//     stall2 <= Stall2;
//     Last_PC <= PC_next;
//     Last_ReadData <= MEM_ReadData;
//     Last_ALU_out <= MEM_ALU_out;
//     Last_Data2 <= MEM_Data2;
//     stals = 0;
//     /*if(MEM_Instruction[31:26] == 6'b101000 & MEM_MemWrite ) begin
//         stals = 1;
//     end else begin
//         stals = 0;
//     end*/
// end

wire mem_ce, mem_we;

assign mem_ce = /*stals? 1:*/ MEM_MemRead | MEM_MemWrite;

assign mem_we = /*stals? 1: (MEM_Instruction[31:26] == 6'b101000)? 0: */MEM_MemWrite;

assign mem_addr = /*(stals == 0)?*/ MEM_ALU_out ;/*: Last_ALU_out;*///上一轮是否已经stals�?

reg clk2;
reg clk3;

always @(posedge clk or negedge clk ) begin
    if( clk == 1 )begin
        clk2 <= 0;
    end else begin
        clk2 <= 1;
    end
    
end

always @(posedge clk2 or negedge clk2 ) begin
    if( clk2 == 1)begin 
        clk3 <= 0;
    end else begin
        clk3 <= 1;
    end
end
assign MEM_DataS = 
      /* (stals == 0)?*/ MEM_Data2;
      /* (mem_addr[1:0] == 2'b00)?{Last_ReadData[31:8],Last_Data2[7:0]}:
       (mem_addr[1:0] == 2'b01)?{Last_ReadData[31:15],Last_Data2[7:0],Last_ReadData[7:0]}:
       (mem_addr[1:0] == 2'b10)?{Last_ReadData[31:24],Last_Data2[7:0],Last_ReadData[15:0]}:
       {MEM_Data2[7:0],Last_ReadData[23:0]};//把sb放在这里处理*/

    ram ram( .clk(clk), .rst(reset), .inst_ce(ce && ~Stall), .inst_addr(PC), .inst( IF_Instruction ), .mem_ce( mem_ce ), .mem_we( mem_we ),
        .mem_addr(mem_addr), .mem_data_i(MEM_DataS), .mem_data_o( MEM_ReadData ), .base_ram_data( base_ram_data), .base_ram_addr(base_ram_addr),
        .base_ram_be_n(base_ram_be_n), .base_ram_ce_n(base_ram_ce_n), .base_ram_oe_n(base_ram_oe_n), .base_ram_we_n(base_ram_we_n), 
        .ext_ram_data(ext_ram_data), .ext_ram_addr(ext_ram_addr), .ext_ram_be_n(ext_ram_be_n), .ext_ram_ce_n(ext_ram_ce_n), 
        .ext_ram_oe_n(ext_ram_oe_n), .ext_ram_we_n(ext_ram_we_n), .Op( /*stals? 6'b000000:*/ MEM_Instruction[31:26] ), .stall(Stall),
        .uart_rdn(uart_rdn), .uart_wrn(uart_wrn), .uart_dataready(uart_dataready), .uart_tbre(uart_tbre), .uart_tsre(uart_tsre), .ok(ok));



endmodule
