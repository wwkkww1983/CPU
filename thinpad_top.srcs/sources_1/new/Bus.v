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
    output wire base_ram_ce_n,
    output wire base_ram_oe_n,
    output wire base_ram_we_n,

    inout wire[31:0] ext_ram_data,
    output wire[19:0] ext_ram_addr,
    output wire ext_ram_ce_n,
    output wire ext_ram_oe_n,
    output wire ext_ram_we_n
);

    reg base_ram_ce_n_reg;
    reg base_ram_oe_n_reg;
    reg base_ram_we_n_reg;

    reg ext_ram_ce_n_reg;
    reg ext_ram_oe_n_reg;
    reg ext_ram_we_n_reg;

    reg[31:0] base_data = 32'bz; 
    reg[31:0] ext_data = 32'bz;

    reg base_read = 1'b0;
    reg base_write = 1'b0;
    reg ext_read = 1'b0;
    reg ext_write = 1'b0;
    reg [19:0] base_ram_addr_reg;

    assign base_ram_data = (base_write == 1'b1) ? base_data : 32'bz;
    assign base_ram_addr = base_ram_addr_reg;
    // assign base_ram_addr = (inst_ce == 1'b0) ? inst_addr[21:2] : mem_addr[21:2];
    assign base_ram_ce_n = base_ram_ce_n_reg;
    assign base_ram_oe_n = base_ram_oe_n_reg;
    assign base_ram_we_n = base_ram_we_n_reg;

    assign ext_ram_data = (ext_write) ? ext_data : 32'bz;
    assign ext_ram_addr = mem_addr[21:2];
    assign ext_ram_ce_n = ext_ram_ce_n_reg;
    assign ext_ram_oe_n = ext_ram_oe_n_reg;
    assign ext_ram_we_n = ext_ram_oe_n_reg;

    reg ext_sel = 1'b0;
    reg [31:0] inst_reg = 32'b0;
    assign inst = inst_reg;
    always @(*) begin
        base_write = 1'b0;
        ext_write = 1'b0;
        if (mem_ce) begin
            if (mem_addr >= 32'h80000000 && mem_addr < 32'h80400000) begin
                ext_sel = 1'b0;
            end else if (mem_addr >= 32'h80400000 && mem_addr < 32'h80800000) begin
                ext_sel = 1'b1;
            end
        end
    end

    always @(*) begin
        if (rst == 1'b1) begin
            base_ram_ce_n_reg <= 1'b1;
            base_ram_oe_n_reg <= 1'b1;
            base_ram_we_n_reg <= 1'b1;
            ext_ram_ce_n_reg <= 1'b1;
            ext_ram_oe_n_reg <= 1'b1;
            ext_ram_we_n_reg <= 1'b1;

            base_data <= 32'bz;
            ext_data <= 32'bz;
            base_write <= 1'b0;
            ext_write <= 1'b0;
        end else begin
            base_ram_ce_n_reg <= 1'b1;
            base_ram_oe_n_reg <= 1'b1;
            base_ram_we_n_reg <= 1'b1;

            ext_ram_ce_n_reg <= 1'b1;
            ext_ram_oe_n_reg <= 1'b1;
            ext_ram_we_n_reg <= 1'b1;

            base_write <= 1'b0;
            ext_write <= 1'b0;
            base_data <= 32'bz;
            ext_data <= 32'bz;
            if (inst_ce == 1'b0) begin
                base_ram_addr_reg = inst_addr[21:2];
            end else begin
                base_ram_addr_reg = mem_addr[21:2];
            end
            if (mem_ce == 1'b0) begin // 有数据操作
                if (ext_sel) begin  //操作在ext上
                    ext_ram_ce_n_reg <= 1'b0;
                    if (mem_we == 1'b1) begin //写操作
                        ext_data <= mem_in_data;
                        ext_ram_oe_n_reg <= 1'b1;
                        ext_ram_we_n_reg <= 1'b0;
                        ext_write <= 1'b1;
                    end else begin //读操作
                        ext_ram_oe_n_reg <= 1'b0;
                        ext_ram_we_n_reg <= 1'b1;
                        ext_write <= 1'b0;
                    end
                end else begin //操作在base上
                    if (mem_we == 1'b1) begin
                        base_ram_ce_n_reg <= 1'b0;
                        base_data <= mem_in_data;
                        base_ram_oe_n_reg <= 1'b1;
                        base_ram_we_n_reg <= 1'b0;
                        base_write <= 1'b1;
                    end else begin
                        base_ram_ce_n_reg <= 1'b0;
                        base_ram_oe_n_reg <= 1'b0;
                        base_ram_we_n_reg <= 1'b1;
                        base_write <= 1'b0;
                    end
                    
                end
            end
            if (inst_ce == 1'b0) begin //有读指令
                base_ram_ce_n_reg <= 1'b0;
                base_write <= 1'b0;
                base_ram_oe_n_reg <= 1'b0;
                base_ram_we_n_reg <= 1'b1;
            end
        end
    end

    always @* begin
        if (rst == 1'b1 || inst_ce == 1'b1) 
            inst_reg <= 32'b0;
        else if (inst_ce == 1'b0)
            inst_reg <= base_ram_data;
        else inst_reg <= 32'b0;
    end

    always @* begin
        if (rst == 1'b1 || mem_ce == 1'b1) begin
            mem_out_data <= 32'b0;
        end else if (mem_ce) begin
            if (mem_we == 1'b0) begin
                if (ext_sel)
                    mem_out_data <= ext_data;
                else mem_out_data <= base_data;
            end else begin
                mem_out_data <= 32'b0;
            end
        end
    end
endmodule