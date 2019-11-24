module test;

  /* Make a reset that pulses once. */
  reg reset = 0;
  initial begin
     # 17 reset = 1;
     # 11 reset = 0;
     # 29 reset = 1;
     # 11 reset = 0;
     # 100 $finish;
  end

  initial begin
    # 1 ALUctl <= 5'b00000;
    in1 <= 32'hffffffff;
    in2 <= 32'h000f010e;
    # 10 ALUctl <= 5'b00001;
    in1 <= 32'hffff0000;
    in2 <= 32'h000f010e;
    # 20 ALUctl <= 5'b00010;
    in1 <= 32'hffffffff;
    in2 <= 32'h00000011;
    # 30 ALUctl <= 5'b00110;
    in1 <= 32'h00000001;
    in2 <= 32'h00000002;
    # 40 ALUctl <= 5'b00111;
    in1 <= 32'hffffffff;
    in2 <= 32'h00000000;
    sign <= 1;
    OpCode <= 6'h07;
    # 50 ALUctl <= 5'b11010;
    in1 <= 32'hffff0000;
    

  end

  /* Make a regular pulsing clock. */
  reg clk = 0;
  always #5 clk = !clk;
  reg [4:0] ALUctl;
  reg [31:0] in1;
  reg [31:0] in2;
  reg sign;
  reg [5:0] OpCode;
  wire [31:0] value;
  wire Branch;
  ALU a1(in1, in2, ALUctl, sign, OpCode, value, Branch);
  initial
    begin
     $dumpfile("test.vcd");
     $dumpvars(0, test);
     $monitor("At time %t, out = %h (%0d) branch = %b",
              $time, value, value, Branch);
    end
endmodule // test