module ram(
    input wire                 clk,
    input wire                 rst,

    // inst
    input wire                 inst_ce,
    input wire[31:0]           inst_addr,
    output wire[31:0]          inst,
 
    // mem
    input wire                 mem_ce,  //memory的ce
    input wire                 mem_we,  //memory的we
    input wire[31:0]           mem_addr,//memory的addr
    // input wire[3:0]            mem_sel, //
    input wire[31:0]           mem_data_i, // write
    output reg[31:0]           mem_data_o, // read

    // devices
    inout wire[31:0]           base_ram_data, //baseram的一些
    output wire[19:0]          base_ram_addr, 
    output wire[3:0]           base_ram_be_n, 
    output wire                base_ram_ce_n,
    output wire                base_ram_oe_n,
    output wire                base_ram_we_n,

    inout wire[31:0]           ext_ram_data,
    output wire[19:0]          ext_ram_addr,
    output wire[3:0]           ext_ram_be_n,
    output wire                ext_ram_ce_n,
    output wire                ext_ram_oe_n,
    output wire                ext_ram_we_n,

    input wire[5:0]            Op,

    input wire                 stall,
    
    output wire                uart_rdn,         
    output wire                uart_wrn,         
    input wire                 uart_dataready,    
    input wire                 uart_tbre,         
    input wire                 uart_tsre         
);

    // regs for always assign
    reg base_ram_ce_n_reg ;
    reg base_ram_oe_n_reg;
    reg base_ram_we_n_reg;
    reg [3:0] base_ram_be_n_reg = 4'b0000;

    reg ext_ram_ce_n_reg;
    reg ext_ram_oe_n_reg;
    reg ext_ram_we_n_reg;
    reg [3:0] ext_ram_be_n_reg = 4'b0000;

    reg [31:0] inner_ram_data = 32'bz;
    reg [31:0] inst_reg = 32'b0;

    reg write_flag;
    reg base_ext_sel = 1'b1; // true is base, false is ext
    //reg inst_mem_sel; // true is inst, false is mem
    wire [31:0] read_from_ram_data;
    
    reg uart_read_or_write; // true for read, false for write
    reg uart_rdn_reg;
    reg uart_wrn_reg;
    reg uart_reading_flag; // true means is reading, false means is not reading
    wire uart_writable;
    assign uart_rdn = uart_rdn_reg;
    assign uart_wrn = uart_wrn_reg;
    assign uart_writable = (uart_tbre == 1'b1 && uart_tsre == 1'b1) ? 1'b1 : 1'b0;
    
    assign base_ram_data = (write_flag && base_ext_sel) ? inner_ram_data : 32'bz;  // assign according to write_flag!!!
    assign base_ram_addr = (inst_ce == 1'b1 && stall == 0) ? inst_addr[21:2] : mem_addr[21:2];//如果是的话，那么先看21:2，或者是
    assign base_ram_be_n = base_ram_be_n_reg;   //
    assign base_ram_ce_n = base_ram_ce_n_reg;
    assign base_ram_oe_n = base_ram_oe_n_reg;
    assign base_ram_we_n = base_ram_we_n_reg;

    assign ext_ram_data = (write_flag && !base_ext_sel) ? inner_ram_data : 32'bz;
    assign ext_ram_addr = (inst_ce == 1'b1 && stall == 0) ? inst_addr[21:2] : mem_addr[21:2];
    assign ext_ram_be_n = ext_ram_be_n_reg;
    assign ext_ram_ce_n = ext_ram_ce_n_reg;
    assign ext_ram_oe_n = ext_ram_oe_n_reg;
    assign ext_ram_we_n = ext_ram_we_n_reg;

    assign inst = inst_reg;
    assign read_from_ram_data = base_ext_sel ? base_ram_data : ext_ram_data;    //
    
    always @ (*) begin
        base_ext_sel <= 1'b1;
        if (mem_ce) begin
            if (mem_addr >= 32'h80000000 && mem_addr < 32'h80400000 ) begin         //
                base_ext_sel <= 1'b1;                                               // 如果是4-8，那么
            end else if (mem_addr >= 32'h80400000 && mem_addr < 32'h80800000) begin //
                base_ext_sel <= 1'b0;
            end
        end
        else if (inst_ce) begin//如果是inst的话那么就先这样了
            if (inst_addr >= 32'h80000000 && inst_addr < 32'h80400000) begin
                base_ext_sel <= 1'b1;
            end else if (inst_addr >= 32'h80400000 && inst_addr < 32'h80800000) begin
                base_ext_sel <= 1'b0;
             end
        end 
    end

always @ (*) begin
        if (rst == 1'b1) begin //如果reset的话
            uart_wrn_reg <= 1'b1;
            uart_rdn_reg <= 1'b1;

            base_ram_ce_n_reg <= 1'b1;
            base_ram_oe_n_reg <= 1'b1;
            base_ram_we_n_reg <= 1'b1;
            base_ram_be_n_reg <= 4'b1111;

            ext_ram_ce_n_reg <= 1'b1;
            ext_ram_oe_n_reg <= 1'b1;
            ext_ram_we_n_reg <= 1'b1;
            ext_ram_be_n_reg <= 4'b1111;
            
            inner_ram_data <= 32'bz;
            write_flag <= 1'b0;
        
        end else begin
            uart_wrn_reg <= 1'b1;
            uart_rdn_reg <= 1'b1;

            base_ram_ce_n_reg <= 1'b1;
            base_ram_we_n_reg <= 1'b1;
            base_ram_oe_n_reg <= 1'b1;
            base_ram_be_n_reg <= 4'b1111;

            ext_ram_ce_n_reg <= 1'b1;
            ext_ram_oe_n_reg <= 1'b1;
            ext_ram_we_n_reg <= 1'b1;
            ext_ram_be_n_reg <= 4'b1111;
            
            inner_ram_data <= 32'bz;
            write_flag <= 1'b0;

            if (mem_ce == 1'b1) begin                                        
                if (mem_addr == 32'hbfd003f8 || mem_addr == 32'hbfd003fc) begin
                    if (mem_we && mem_addr == 32'hbfd003f8) begin                  // uart + sb write to SerialData
                        write_flag <= 1'b1;
                        inner_ram_data <= {24'b0, mem_data_i[7:0]};
                        uart_wrn_reg <= clk;//1'b0;
                    end else begin                                                  // uart + lb (read)
                        if (mem_addr == 32'hbfd003f8) begin                         // read from SerialData
                            uart_rdn_reg <= clk;//1'b0;
                        end
                    end
                end else begin
                    if (base_ext_sel) begin                                         // mem + base
                        base_ram_ce_n_reg <= 1'b0;
                        if(mem_we) begin
                            inner_ram_data <= 32'bz;
                            case (Op)
                                6'b101000 : begin
                                    if(mem_addr[1:0]==2'b11) begin
                                        base_ram_be_n_reg <= 4'b0111;
                                        inner_ram_data[31:24] <= mem_data_i[7:0];
                                    end else if(mem_addr[1:0]==2'b10) begin
                                        base_ram_be_n_reg <= 4'b1011;
                                        inner_ram_data[23:16] <= mem_data_i[7:0];
                                    end else if(mem_addr[1:0]== 2'b01) begin
                                        base_ram_be_n_reg <= 4'b1101;
                                        inner_ram_data[15:8] <= mem_data_i[7:0];
                                    end else begin
                                        base_ram_be_n_reg <= 4'b1110;
                                        inner_ram_data[7:0] <= mem_data_i[7:0];
                                    end
                                end
                                6'b101011: begin
                                    base_ram_be_n_reg <= 4'b0000;
                                    inner_ram_data <= mem_data_i;
                                end
                                6'b000000: begin
                                    base_ram_be_n_reg <= 4'b0000;
                                    inner_ram_data <= mem_data_i;
                                end
                            endcase
                            base_ram_oe_n_reg <= 1'b1;
                            base_ram_we_n_reg <= 1'b0;
                            write_flag <= 1'b1;
                        end else begin                      // mem + base + read
                            base_ram_be_n_reg <= 4'b0000;
                            base_ram_oe_n_reg <= 1'b0;
                            base_ram_we_n_reg <= 1'b1;
                            write_flag <= 1'b0;
                        end
                    end else begin                          // mem + ext
                        ext_ram_ce_n_reg <= 1'b0;
                        if(mem_we) begin
                            inner_ram_data <= 32'bz;
                            case (Op)
                                6'b101000 : begin
                                    if(mem_addr[1:0]==2'b11) begin
                                        ext_ram_be_n_reg <= 4'b0111;
                                        inner_ram_data[31:24] <= mem_data_i[7:0];
                                    end else if(mem_addr[1:0]==2'b10) begin
                                        ext_ram_be_n_reg <= 4'b1011;
                                        inner_ram_data[23:16] <= mem_data_i[7:0];
                                    end else if(mem_addr[1:0]== 2'b01) begin
                                        ext_ram_be_n_reg <= 4'b1101;
                                        inner_ram_data[15:8] <= mem_data_i[7:0];
                                    end else begin
                                        ext_ram_be_n_reg <= 4'b1110;
                                        inner_ram_data[7:0] <= mem_data_i[7:0];
                                    end
                                end
                                6'b101011: begin
                                    ext_ram_be_n_reg <= 4'b0000;
                                    inner_ram_data <= mem_data_i;
                                end
                                6'b000000: begin
                                    ext_ram_be_n_reg <= 4'b0000;
                                    inner_ram_data <= mem_data_i;
                                end
                            endcase
                            ext_ram_oe_n_reg <= 1'b1;
                            ext_ram_we_n_reg <= 1'b0;
                            write_flag <= 1'b1;
                        end else begin                      // mem + base + read
                            ext_ram_be_n_reg <= 4'b0000;
                            ext_ram_oe_n_reg <= 1'b0;
                            ext_ram_we_n_reg <= 1'b1;
                            write_flag <= 1'b0;
                        end
                    end
                end
            end else if (inst_ce == 1'b1) begin  // inst
                uart_wrn_reg <= 1'b1;
                uart_rdn_reg <= 1'b1;
                write_flag = 1'b0;
                if (base_ext_sel) begin                 // inst + base
                    base_ram_be_n_reg <= 4'b0000;
                    base_ram_ce_n_reg <= 1'b0;
                    base_ram_we_n_reg <= 1'b1;
                    base_ram_oe_n_reg <= 1'b0;
                    ext_ram_ce_n_reg <= 1'b1;
                    ext_ram_oe_n_reg <= 1'b1;
                    ext_ram_we_n_reg <= 1'b1;
                    ext_ram_be_n_reg <= 4'b0000;
                end else begin                          // inst + ext
                    ext_ram_be_n_reg <= 4'b0000;
                    ext_ram_ce_n_reg <= 1'b0;
                    ext_ram_we_n_reg <= 1'b1;
                    ext_ram_oe_n_reg <= 1'b0;
                    base_ram_ce_n_reg <= 1'b1;
                    base_ram_we_n_reg <= 1'b1;
                    base_ram_oe_n_reg <= 1'b1;
                    base_ram_be_n_reg <= 4'b0000;
                end
            end
        end
    end

    // load inst
    always @* begin
        if (rst == 1'b1 || inst_ce == 1'b0) begin
            inst_reg <= 32'b0;
        end else if(inst_ce == 1'b1 && mem_ce == 1'b0 && stall == 0) begin
            inst_reg <= read_from_ram_data;
        end else begin
            inst_reg <= 32'b0;
        end
    end
    // load
always @* begin
        if (rst == 1'b1 || mem_ce == 1'b0) begin
            mem_data_o <= 32'b0;
        end else if (mem_ce == 1'b1 && mem_addr == 32'hbfd003fc && mem_we == 1'b0) begin // Serial Status
            mem_data_o <= {30'b0, uart_dataready, uart_writable};
        end else if (mem_ce == 1'b1 && mem_addr == 32'hbfd003f8 && mem_we == 1'b0) begin // Serial Data
            mem_data_o <= {24'b0, base_ram_data[7:0]};
        end else if(mem_we == 1'b0) begin // read
            case (Op)
                6'b100000: begin//LB
                    if(mem_addr[1:0]==2'b11) begin
                        mem_data_o <= {{24{read_from_ram_data[31]}}, read_from_ram_data[31:24]};
                    end else if(mem_addr[1:0]==2'b10) begin
                        mem_data_o <= {{24{read_from_ram_data[23]}}, read_from_ram_data[23:16]};
                    end else if(mem_addr[1:0]== 2'b01) begin
                        mem_data_o <= {{24{read_from_ram_data[15]}}, read_from_ram_data[15:8]};
                    end else begin
                        mem_data_o <= {{24{read_from_ram_data[7]}}, read_from_ram_data[7:0]};
                    end
                end
                6'b100011: begin
                    mem_data_o <= read_from_ram_data;
                end
                6'b101000: begin
                    mem_data_o <= read_from_ram_data;
                end
                default: mem_data_o <= 32'h00000000;
            endcase
        end else begin // write ram or write 
            mem_data_o <= 32'b0;
        end
end

endmodule