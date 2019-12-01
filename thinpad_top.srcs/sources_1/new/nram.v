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
    
    output reg                 uart_rdn,         
    output reg                 uart_wrn,         
    input wire                 uart_dataready,    
    input wire                 uart_tbre,         
    input wire                 uart_tsre,
    output reg                 ok 
);

wire write = mem_ce && mem_we;

reg[7:0] uart_status, uart_data;
wire is_uart = mem_addr == 32'hbfd003f8 || mem_addr == 32'hbfd003fc;
integer read_status, write_status;

always @(negedge clk, posedge rst) begin
    if (rst) begin
        read_status <= 1;
        write_status <= 1;
        uart_wrn <= 1;
        uart_rdn <= 1;
    end else begin
        if (mem_addr == 32'hbfd003fc) begin
            if (write) begin
                ok <= 1;
            end else if (mem_ce && ~mem_we) begin
                case (read_status)
                    1 : begin
                        ok <= 0;
                        uart_rdn <= 1;
                        read_status <= 2;
                    end
                    2 : begin
                        ok <= 1;
                        read_status <= 1;
                        uart_status <= {30'b0, uart_dataready, uart_tbre && uart_tsre};
                    end
                endcase
            end
        end else if (mem_addr == 32'hbfd003f8) begin
            if (mem_ce && mem_we) begin
                case (write_status)
                    1 : begin
                        ok <= 0;
                        uart_wrn <= 0;
                        write_status <= 2;
                    end
                    2 : begin
                        ok <= 0;
                        uart_wrn <= 1;
                        write_status <= 3;
                    end
                    3 : begin
                        ok <= 0;
                        if (uart_tbre)
                            write_status <= 4;
                    end
                    4 : begin
                        if (uart_tbre) begin
                            ok <= 1;
                            write_status <= 1;
                        end else begin
                            ok <= 1;
                            write_status <= 4;
                        end
                    end
                endcase
            end else if (mem_ce && ~mem_we) begin
                case (read_status)
                    1 : begin
                        ok <= 0;
                        uart_rdn <= 1;
                        read_status <= 2;
                    end
                    2 : begin
                        ok <= 0;
                        if (uart_dataready) begin
                            uart_rdn <= 0;
                            read_status <= 3;
                        end else begin
                            read_status <= 1;
                        end
                    end
                    3 : begin
                        ok <= 1;
                        uart_data <= base_ram_data;
                        uart_rdn <= 1;
                        read_status <= 1;
                    end
                endcase
            end
        end else begin
            ok <= 1;
        end
    end 
end

wire [31:0] write_data;
wire ext_sel;
wire [31:0] addr;

assign ext_sel = mem_ce ? mem_addr[22] : inst_addr[22];
assign write_data = mem_data_i;

assign addr = mem_ce ? mem_addr : inst_addr;

assign base_ram_ce_n = is_uart || ext_sel;
assign base_ram_oe_n = is_uart || write;
assign base_ram_we_n = is_uart || ~write;
assign base_ram_be_n = 0;
assign base_ram_addr = addr[21:2];
assign base_ram_data = ~write ? 32'bz : write_data;

assign ext_ram_ce_n = is_uart || ~ext_sel;
assign ext_ram_oe_n = is_uart || write;
assign ext_ram_we_n = is_uart || ~write;
assign ext_ram_be_n = 0;
assign ext_ram_addr = addr[21:2];
assign ext_ram_data = ~write ? 32'bz : write_data;

wire [31:0] data;

assign data = addr == 32'hbfd003fc ? uart_status : 
                (addr == addr == 32'hbfd003f8 ? uart_data : 
                (~ext_sel ? base_ram_data : ext_ram_data));
assign inst = inst_ce ? data : 32'b0;
assign mem_data_o = mem_ce && ~mem_we ? data : 32'b0;
endmodule