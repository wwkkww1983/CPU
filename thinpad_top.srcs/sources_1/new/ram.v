module ram(
    input wire          rst,

    input wire                 inst_ce,
    input wire[31:0]   inst_addr,
    output wire[31:0]      inst,
 
    input wire                 mem_ce,  //memory的ce
    input wire                 mem_we,  //memory的we
    input wire[31:0]       mem_addr,//memory的addr
    input wire[31:0]       mem_data_i, // write
    output reg[31:0]       mem_data_o, // read

    inout wire[31:0] base_ram_data, //baseram的一些
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
    
    input wire base_sel_mem,
    input wire ext_sel_mem,
    input wire base_sel_inst,
    input wire ext_sel_inst,
    
    input wire[5:0]     Op
);

    reg base_ram_ce_n_reg = 1'b1;
    reg base_ram_oe_n_reg = 1'b1;
    reg base_ram_we_n_reg = 1'b1;
    reg [3:0] base_ram_be_n_reg = 4'b0000;

    reg ext_ram_ce_n_reg = 1'b1;
    reg ext_ram_oe_n_reg = 1'b1;
    reg ext_ram_we_n_reg = 1'b1;
    reg [3:0] ext_ram_be_n_reg = 4'b0000;

    reg [31:0] inner_ram_data = 32'bz;
    reg [31:0] inst_reg = 32'b0;

    reg base_write_flag;
    reg ext_write_flag;
    //reg inst_mem_sel; // true is inst, false is mem

    wire[31:0] read_mem_from_ram_data;
    wire[31:0] read_inst_from_ram_data;

    // reg uart_rdn_reg;
    // reg uart_wrn_reg;
    //reg uart_reading_flag; // true means is reading, false means is not reading
    // wire uart_writable;
    // assign uart_rdn = uart_rdn_reg;
    // assign uart_wrn = uart_wrn_reg;
    // assign uart_writable = (uart_tbre == 1'b1 && uart_tsre == 1'b1) ? 1'b1 : 1'b0;

    assign base_ram_data = (base_write_flag && (base_sel_mem || base_sel_inst)) ? inner_ram_data : 32'bz;  // assign according to write_flag!!!
    assign base_ram_addr = (inst_ce == 1'b0 ) ? inst_addr[21:2] : mem_addr[21:2];//如果是的话，那么先看21:2，或者是
    assign base_ram_be_n = base_ram_be_n_reg;   //
    assign base_ram_ce_n = base_ram_ce_n_reg;
    assign base_ram_oe_n = base_ram_oe_n_reg;
    assign base_ram_we_n = base_ram_we_n_reg;

    assign ext_ram_data = (ext_write_flag && (ext_sel_mem || ext_sel_inst)) ? inner_ram_data : 32'bz;
    assign ext_ram_addr = (inst_ce == 1'b0 ) ? inst_addr[21:2] : mem_addr[21:2];
    assign ext_ram_be_n = ext_ram_be_n_reg;
    assign ext_ram_ce_n = ext_ram_ce_n_reg;
    assign ext_ram_oe_n = ext_ram_oe_n_reg;
    assign ext_ram_we_n = ext_ram_we_n_reg;

    assign inst = inst_reg;
    assign read_inst_from_ram_data = base_sel_inst ? base_ram_data : ( ext_sel_inst? ext_ram_data: 32'h00000000);
    assign read_mem_from_ram_data = base_sel_mem ? base_ram_data : ( ext_sel_mem? ext_ram_data: 32'h00000000);

    always @ (*) begin
        if (rst == 1'b1) begin //如果reset的话

            base_ram_ce_n_reg <= 1'b1;
            base_ram_oe_n_reg <= 1'b1;
            base_ram_we_n_reg <= 1'b1;
            base_ram_be_n_reg <= 4'b1111;

            ext_ram_ce_n_reg <= 1'b1;
            ext_ram_oe_n_reg <= 1'b1;
            ext_ram_we_n_reg <= 1'b1;
            ext_ram_be_n_reg <= 4'b1111;
            
            inner_ram_data <= 32'bz;
            base_write_flag <= 1'b0;
            ext_write_flag <= 1'b0;
        end else begin

            base_ram_ce_n_reg <= 1'b1;
            base_ram_we_n_reg <= 1'b1;
            base_ram_oe_n_reg <= 1'b1;
            base_ram_be_n_reg <= 4'b1111;

            ext_ram_ce_n_reg <= 1'b1;
            ext_ram_oe_n_reg <= 1'b1;
            ext_ram_we_n_reg <= 1'b1;
            ext_ram_be_n_reg <= 4'b1111;
            
            // uart_wrn_reg <= 1'b1;
            // uart_rdn_reg <= 1'b1;
            
            inner_ram_data <= 32'bz;
            base_write_flag <= 1'b0;
            ext_write_flag <= 1'b0;

            if (mem_ce == 1'b1) begin
                    if(base_sel_mem) begin          // 要写
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
                            endcase
                            base_ram_oe_n_reg <= 1'b1;
                            base_ram_we_n_reg <= 1'b0;
                            base_write_flag <= 1'b1;
                        end else begin                      // mem + base + read
                            base_ram_be_n_reg <= 4'b0000;
                            base_ram_oe_n_reg <= 1'b0;
                            base_ram_we_n_reg <= 1'b1;
                            base_write_flag <= 1'b0;
                        end
                    end else if(ext_sel_mem) begin
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
                            endcase
                            ext_ram_oe_n_reg <= 1'b1;
                            ext_ram_we_n_reg <= 1'b0;
                            ext_write_flag <= 1'b1;
                        end else begin                      // mem + base + read
                            ext_ram_be_n_reg <= 4'b0000;
                            ext_ram_oe_n_reg <= 1'b0;
                            ext_ram_we_n_reg <= 1'b1;
                            ext_write_flag <= 1'b0;
                        end
                    end
                end
            if ( inst_ce == 1'b1 ) begin
                
                if (base_sel_inst) begin                 // inst + base
                    base_ram_be_n_reg <= 4'b0000;
                    base_ram_ce_n_reg <= 1'b0;
                    base_ram_we_n_reg <= 1'b1;
                    base_ram_oe_n_reg <= 1'b0;
                    base_write_flag <= 1'b0;
                end else if(ext_sel_inst) begin                          // inst + ext
                    ext_ram_be_n_reg <= 4'b0000;
                    ext_ram_ce_n_reg <= 1'b0;
                    ext_ram_we_n_reg <= 1'b1;
                    ext_ram_oe_n_reg <= 1'b0;
                    ext_write_flag <= 1'b0;
                end
            end
        end
    end

    always @* begin
        if (rst == 1'b1 || inst_ce == 1'b0) begin//不通inst
            inst_reg <= 32'h00000000;
        end else begin
            inst_reg <= read_inst_from_ram_data;
        end
    end

    always @* begin//
        if (rst == 1'b1 || mem_ce == 1'b0) begin
            mem_data_o <= 32'h00000000;
        end else if(mem_we == 1'b0) begin // read
            case (Op)
                6'b100000: begin//LB
                    if(mem_addr[1:0] == 2'b11) begin
                        mem_data_o <= {{24{read_mem_from_ram_data[31]}}, read_mem_from_ram_data[31:24]};
                    end else if(mem_addr[1:0]== 2'b10) begin
                        mem_data_o <= {{24{read_mem_from_ram_data[23]}}, read_mem_from_ram_data[23:16]};
                    end else if(mem_addr[1:0] == 2'b01) begin
                        mem_data_o <= {{24{read_mem_from_ram_data[15]}}, read_mem_from_ram_data[15:8]};
                    end else begin
                        mem_data_o <= {{24{read_mem_from_ram_data[7]}}, read_mem_from_ram_data[7:0]};
                    end
                end
                6'b001111: begin//LUI
                    mem_data_o <= {read_mem_from_ram_data[15:0],16'h0000};
                end
                6'b100011: begin
                    mem_data_o <= read_mem_from_ram_data;
                end
                default: mem_data_o <= 32'h00000000;
            endcase
        end else begin // write ram or write uart
            mem_data_o <= 32'h00000000;
        end
    end
                        
endmodule                    