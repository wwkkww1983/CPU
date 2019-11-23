module DataMemory( reset, ce, clk, Address, Write_data, Read_data, 
    MemRead, MemWrite, IRQ_now, IRQ, seg, AN );

    input reset,clk;
    input [31:0] Address, Write_data;

    input MemRead, MemWrite, IRQ_now;

    output reg [31:0] Read_data;
    output IRQ;
    output [6:0] seg;
	output [3:0] AN;

    inout wire[31:0] baseram_data;
    output wire[19:0] baseram_addr;


    assign 


    always @(posedge reset or posedge clk)
        if (reset begin)
            baseram_data <= 32'h00000000;
        end
        else begin
            if( MemWrite ):



    