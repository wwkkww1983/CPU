module Bus(
    input wire clk,
    input wire rst,

    //inst
    input wire inst_ce,
    input wire[31:0] inst_addr,
    output wire[31:0] inst,


    //mem
    input wire mem_ce,
    input wire mem_we,
    input wire[31:0] mem_addr,
    input wire[31:0] mem_in_data,
    output reg[31:0] mem_out_data,

    //devices
    inout wire[31:0] base_ram_data,
    output wire[19:0] base_ram_addr,
    output wire[3:0] base_ram_be_n,
    output wire base_ram_ce_n,
    output wire base_ram_oe_n,
    output wire base_ram_we_n,

    inout wire[31:0] ext_ram_data,
    output wire[19:0] ext_ram_addr,
    output wire[3:0] ext_ram_be_n,
    output wire ext_ram_ce_n,
    output wire ext_ram_oe_n,
    output wire ext_ram_we_n,

    input wire stall,

    output wire uart_rdn,
    output wire uart_wrn,
    input wire uarrt_dataready,
    input wire uart_tbre,
    input wire uart_tsre
);

    reg base_ram_ce_n_reg = 1'b1;
    reg base_ram_oe_n_reg = 1'b1;
    reg base_ram_we_n_reg = 1'b1;

    reg ext_ram_ce_n_reg = 1'b1;
    reg ext_ram_oe_n_reg = 1'b1;
    reg ext_ram_we_n_reg = 1'b1;

    reg[31:0] base_data = 32'bz; 
    reg[31:0] ext_data = 32'bz;

    reg base_read = 1'b0;
    reg base_write = 1'b0;
    reg ext_read = 1'b0;
    reg ext_write = 1'b0;

    assign base_ram_data = (write_flag && base_write) ? base_data : 32'bz;
    assign base_ram_ce_n = base_ram_ce_n_reg;
    assign base_ram_oe_n = base_ram_oe_n_reg;
    assign base_ram_we_n = base_ram_we_n_reg;

    assign ext_ram_data = (write_flag && ext_write) ? ext_data : 32'bz;
    assign ext_ram_ce_n = ext_ram_ce_n_reg;
    assign ext_ram_oe_n = ext_ram_oe_n_reg;
    assign ext_ram_we_n = ext_ram_oe_n_reg;

    reg ext_sel = 1'b0;
    reg write_flag;

    always @(*) begin
        base_write = 1'b1;
        base_read = 1'b1;
        ext_write = 1'b1;
        ext_read = 1'b1;
        if (mem_ce) begin
            if (mem_addr >= 32'h80000000 && mem_addr < 32'80400000) begin
                ext_sel = 1'b0;
            end else if (mem_addr >= 32'h80400000 && mem_addr < 32'h80800000) begin
                ext_sel = 1'b1;
            end
        end
    end

    always @(*) begin
        if (rst = 1'b1) begin
            base_ram_ce_n_reg = 1'b1;
            base_ram_oe_n_reg = 1'b1;
            base_ram_we_n_reg = 1'b1;
            ext_ram_ce_n_reg = 1'b1;
            ext_ram_oe_n_reg = 1'b1;
            ext_ram_we_n_reg = 1'b1;

            base_data <= 32'bz;
            ext_data <= 32'bz;
        end else begin
            base_ram_ce_n_reg = 1'b1;
            base_ram_oe_n_reg = 1'b1;
            base_ram_we_n_reg = 1'b1;
            ext_ram_ce_n_reg = 1'b1;
            ext_ram_oe_n_reg = 1'b1;
            ext_ram_we_n_reg = 1'b1;
        end
    end
endmodule