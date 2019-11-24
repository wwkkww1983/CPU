module RegisterFile(reset, clk, RegWrite, Read_register1, Read_register2, Write_register, Write_data, Read_data1, Read_data2, DoRead);
	input reset, clk;
	input RegWrite, DoRead;//写到哪个寄存器里去 or 是否Read
	input [4:0] Read_register1, Read_register2, Write_register;
	input [31:0] Write_data;
	output [31:0] Read_data1, Read_data2;
	
	reg [31:0] RF_data[31:1];

    wire [31:0] Show_data[31:1];
    
    genvar j;
    
generate
    for(j = 1; j < 32; j = j + 1)
        assign Show_data[j] = RF_data[j];
endgenerate	

	assign Read_data1 = (Read_register1 != 5'b00000 & DoRead )? RF_data[Read_register1]: 32'h00000000 ;
	assign Read_data2 = (Read_register2 != 5'b00000 & DoRead )? RF_data[Read_register2]: 32'h00000000 ;
	
	integer i;
	always @(posedge reset or posedge clk)
		if (reset)
			for (i = 1; i < 32; i = i + 1)// 对于寄存器之外除了29之外,都写入0,否则写入800
				if (i != 29) RF_data[i] <= 32'h00000000;
                else RF_data[i] <= 32'h00000800;
		else if (RegWrite & (Write_register != 5'b00000))// 写寄存器的32位号不是0(错误写入)
			RF_data[Write_register] <= Write_data;// 那么,写寄存器写到RF_data里面去

endmodule