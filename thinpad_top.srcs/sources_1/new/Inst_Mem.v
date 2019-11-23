module Inst_Mem(
    input wire ce,
    input wire [31:0] Address, 
    output wire[31:0] Instruction,

    inout wire[31:0] baseram_data,
    output wire[19:0] baseram_addr,

    output wire baseram_ce,
    output wire baseram_oe,
    output wire baseram_we
    );

    reg baseram_ce_reg = 1'b0;
    reg baseram_oe_reg = 1'b0;
    reg baseram_we_reg = 1'b0;
    reg [19:0] baseram_addr_reg = 20'b0;
    reg [31:0] innerram_data = 32'b0;
    reg [32:0] inst_reg = 32'b0;
    
    assign baseram_data = innerram_data;
    assign baseram_addr = {Address[19:2],2'b00};
    assign baseram_be = 4'b0000;
    assign baseram_ce = baseram_ce_reg;
    assign baseram_oe = baseram_oe_reg;
    assign baseram_we = baseram_we_reg;
    assign Instruction = inst_reg;


    always @ ( * ) begin
    
        if ( ce == 1'b0 ) begin
            inst_reg <= 32'h00000000;
            baseram_ce_reg <= 1'b1;
            baseram_we_reg <= 1'b1;            
            baseram_oe_reg <= 1'b1;

        end else begin
            
            innerram_data <= 32'bz;
            baseram_ce_reg <= 1'b0;//低有效,开始读
            baseram_we_reg <= 1'b1;//
            baseram_oe_reg <= 1'b0;//
        end
    end

    always @* begin
        
        if (ce == 1'b0) begin
            inst_reg <= 32'h00000000;
        end else begin
            inst_reg[7:0] = baseram_data[31:24];
            inst_reg[15:8] = baseram_data[23:16];
            inst_reg[23:16] = baseram_data[15:8];
            inst_reg[31:24] = baseram_data[7:0];    

        end 
    end

endmodule