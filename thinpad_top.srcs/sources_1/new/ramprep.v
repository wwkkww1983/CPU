module ramprep(
    input wire[31:0] mem_addr,
    input wire mem_ce,
    input wire[31:0] inst_addr,
    input wire inst_ce,
    output wire base_chosen_inst,
    output wire ext_chosen_inst,
    output wire base_chosen_mem,
    output wire ext_chosen_mem,
    output wire RW
);

reg base_sel_inst;
reg ext_sel_inst;
reg base_sel_mem;
reg ext_sel_mem;
reg RW_rush;

assign base_chosen_inst = base_sel_inst;
assign ext_chosen_inst = ext_sel_inst;
assign base_chosen_mem = base_sel_mem;
assign ext_chosen_mem = ext_sel_mem;
assign RW = RW_rush;//这线咱弄上去

always @ (*) begin
        base_sel_inst <= 1'b0;
        ext_sel_inst <= 1'b0;
        base_sel_mem <= 1'b0;
        ext_sel_mem <= 1'b0;
        RW_rush <= 1'b0;
        if (mem_ce) begin
            if (mem_addr >= 32'h80000000 && mem_addr < 32'h80400000 ) begin         //
                base_sel_mem <= 1'b1;                                               // 如果是4-8，那么
            end else if (mem_addr >= 32'h80400000 && mem_addr < 32'h80800000) begin //
                ext_sel_mem <= 1'b1;
            end
        end
        if (inst_ce) begin//如果是inst的话那么就先这样了
            if (inst_addr >= 32'h80000000 && inst_addr < 32'h80400000) begin
                if(base_sel_mem == 1'b1) begin
                    RW_rush <= 1'b1;
                    base_sel_inst <= 1'b0;
                end else begin
                    base_sel_inst <= 1'b1;
                end
            end else if (inst_addr >= 32'h80400000 && inst_addr < 32'h80800000) begin//如果两次需要读同一个地方的话
                if(ext_sel_mem == 1'b1) begin
                    RW_rush <= 1'b1;
                    ext_sel_inst <= 1'b0;
                end else begin
                    ext_sel_inst <= 1'b1;
                end
             end
        end
    end
endmodule