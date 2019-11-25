module DataMemory( reset, ce, Address, Write_data, Read_data, Op, MemRead, MemWrite
     ,ext_ram_data, ext_ram_addr, ext_ram_ce_n, ext_ram_we_n, ext_ram_oe_n );//此地需要能够

    input reset, ce;
    input [31:0] Address, Write_data;
    input [5:0] Op;

    input MemRead, MemWrite;

    output reg [31:0] Read_data;

    inout wire[31:0] ext_ram_data;
    output wire[19:0] ext_ram_addr;
    output wire ext_ram_ce_n;
    output wire ext_ram_we_n;
    output wire ext_ram_oe_n;


    reg ext_ram_ce_n_reg = 1'b0;
    reg ext_ram_we_n_reg = 1'b0;
    reg ext_ram_oe_n_reg = 1'b0;

    reg [31:0] inner_ram_data = 32'b0;
    
    assign ext_ram_data = inner_ram_data;
    assign ext_ram_addr = {2'b00,Address[19:2]};
    assign ext_ram_ce_n = ext_ram_ce_n_reg;
    assign ext_ram_oe_n = ext_ram_oe_n_reg;
    assign ext_ram_we_n = ext_ram_we_n_reg;

    always @(*) begin
        if(ce == 1'b0) begin
            inner_ram_data <= 32'b0;
            ext_ram_ce_n_reg <= 1'b1;
            ext_ram_we_n_reg <= 1'b1;
            ext_ram_oe_n_reg <= 1'b1;
        end else begin
            if( MemRead ) begin
                inner_ram_data <= 32'bz;
                ext_ram_ce_n_reg <= 1'b0;
                ext_ram_we_n_reg <= 1'b1;
                ext_ram_oe_n_reg <= 1'b0;
            end else if( MemWrite ) begin
                inner_ram_data <= 32'bz;
                ext_ram_ce_n_reg <= 1'b0;
                ext_ram_we_n_reg <= 1'b0;
                ext_ram_oe_n_reg <= 1'b1;
            end
        end
    end

    always @(*) begin
        if (reset == 1'b1 || ce == 1'b0 ) begin
            Read_data <= 32'h00000000;
        end else if( MemRead ) begin
            case (Op)
                6'b100000: begin//LB
                    if(Address[1:0] == 2'b11) begin
                        Read_data <= {{24{ext_ram_data[31]}}, ext_ram_data[31:24]};
                    end else if(Address[1:0]== 2'b10) begin
                        Read_data <= {{24{ext_ram_data[23]}}, ext_ram_data[23:16]};
                    end else if(Address[1:0] == 2'b01) begin
                        Read_data <= {{24{ext_ram_data[15]}}, ext_ram_data[15:8]};
                    end else begin
                        Read_data <= {{24{ext_ram_data[7]}},ext_ram_data[7:0]};
                    end
                end
                6'b001111: begin//LUI
                    Read_data <= {ext_ram_data[15:0],16'h0000};
                end
                6'b100011: begin
                    Read_data <= ext_ram_data;
                end
                default: Read_data <= 32'h00000000;
            endcase
        end else begin
            case (Op)
                6'b101000 : begin
                    if(Address[1:0]==2'b11) begin
                        inner_ram_data[31:24] <= Write_data[7:0];
                    end else if(Address[1:0]==2'b10) begin
                        inner_ram_data[23:16] <= Write_data[7:0];
                    end else if(Address[1:0]== 2'b01) begin
                        inner_ram_data[15:8] <= Write_data[7:0];
                    end else begin
                        inner_ram_data[7:0] <= Write_data[7:0];
                    end
                end
                6'b101011: begin
                    inner_ram_data = Write_data;
                end
            endcase
            //Read_data <= 32'bz;
        end
    end
endmodule



    