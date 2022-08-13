`ifndef ADD_NORMALIZER_V_
`define ADD_NORMALIZER_V_

`timescale 1ns / 1ps

module add_normalizer (
  input             sign,
  input      [ 4:0] exponent,
  input      [10:0] mantissa_add,
  output reg [15:0] result,
  input             if_carray,
  input             if_sub
);

  reg [4:0] number_of_zero_lead;
  reg [10:0] norm_mantissa_add;
  reg [9:0] mantissa_tmp;

  wire [4:0] shift_left_exp;
  wire c1;

  always @ (*) begin
    if (mantissa_add[10:4] == 7'b0000_001) begin
      number_of_zero_lead = 5'd6;
      norm_mantissa_add   = (mantissa_add << 4'd6);
    end else if (mantissa_add[10:5] == 6'b0000_01) begin 
      number_of_zero_lead = 5'd5;
      norm_mantissa_add   = (mantissa_add << 4'd5);
    end else if (mantissa_add[10:6] == 5'b0000_1) begin
      number_of_zero_lead = 5'd4;
      norm_mantissa_add   = (mantissa_add << 4'd4);
    end else if (mantissa_add[10:7] == 4'b0001) begin
      number_of_zero_lead = 5'd3;
      norm_mantissa_add   = (mantissa_add << 4'd3);
    end else if (mantissa_add[10:8] == 3'b001) begin
      number_of_zero_lead = 5'd2;
      norm_mantissa_add   = (mantissa_add << 4'd2);
    end else if (mantissa_add[10:9] == 2'b01) begin
      number_of_zero_lead = 5'd1;
      norm_mantissa_add   = (mantissa_add << 4'd1);
    end else begin 
      number_of_zero_lead = 5'd0;
      norm_mantissa_add   = mantissa_add[10:0];
    end 
  end

  always @(*) begin
    result[15]      = sign;
    if (!if_sub) begin 
      result[14:10] = if_carray ? exponent + 1'b1 : exponent;
      result[9:0]   = if_carray ? mantissa_add[10:1] : mantissa_add[9:0];
    end else begin 
      result[14:10] = shift_left_exp;
      result[9:0]   = norm_mantissa_add[9:0];
    end 
  end

  cla_nbit #(.n(5)) u1(exponent,~number_of_zero_lead+1'b1,1'b0,shift_left_exp,c1);

endmodule

`endif
`ifndef CONTROL_V_
`define CONTROL_V_


`timescale 1ns / 1ps
`default_nettype none

// Really 3x3, a done output??


// 6 logic cycles + 2 (buffered?) delay cycles
module control #(
  parameter W = 16,
  parameter N = 3
) (
  input  wire                         i_clk,
  input  wire                         i_rst,
  input  wire                         i_en,
  input  wire                         i_mode,
  input  wire [    W * N * N - 1 : 0] i_A,
  input  wire [    W * N * N - 1 : 0] i_B,
  output wire [    W * N * N - 1 : 0] o_C,
  output wire                         o_done,

  // debug
  output wire [    W * N * 2 - 1 : 0] debug_pe_a,
  output wire [    W * N * 2 - 1 : 0] debug_pe_b
);
  
  reg [3 : 0] states, next_states;

  reg  [W - 1 : 0] a00, a01, a02;
  wire [W - 1 : 0] a01_q, a02_q;

  reg  [W - 1 : 0] b00, b01, b02;
  wire [W - 1 : 0] b01_q, b02_q;

  wire [W * N - 1 : 0] A_in;
  wire [W * N - 1 : 0] B_in;

  assign A_in = {a02_q, a01_q, a00};
  assign B_in = {b02_q, b01_q, b00};  

  // 0000 is the idle / standby state
  always @(posedge i_clk) begin
    if (i_rst | ~i_en) begin
      states <= 4'b0000;
    end
    else if (i_en) begin
      states <= next_states;
    end
  end
  
  always @(*) begin
    if (next_states == 4'b1001) begin
      // Done: Force waiting
      // This can also be done by switching off input
      next_states = 4'b1001;
    end else begin
      next_states = states + 4'b0001;
    end
  end

  assign o_done = (states == 4'b1001);	
  
  always @(*) begin
    case (states)
      4'b0001: begin
        a00 = i_A[    W - 1 : 0 * W];
        a01 = i_A[2 * W - 1 : 1 * W];
        a02 = i_A[3 * W - 1 : 2 * W];
       
        
        b00 = i_B[    W - 1 : 0 * W];
        b01 = i_B[2 * W - 1 : 1 * W];
        b02 = i_B[3 * W - 1 : 2 * W];
      end
      4'b0010: begin
        a00 = i_A[4 * W - 1 : 3 * W];
        a01 = i_A[5 * W - 1 : 4 * W];
        a02 = i_A[6 * W - 1 : 5 * W];
  
        b00 = i_B[4 * W - 1 : 3 * W];
        b01 = i_B[5 * W - 1 : 4 * W];
        b02 = i_B[6 * W - 1 : 5 * W];
      end
      4'b0011: begin
        a00 = i_A[7 * W - 1 : 6 * W];
        a01 = i_A[8 * W - 1 : 7 * W];
        a02 = i_A[9 * W - 1 : 8 * W];
        
        b00 = i_B[7 * W - 1 : 6 * W];
        b01 = i_B[8 * W - 1 : 7 * W];
        b02 = i_B[9 * W - 1 : 8 * W];
      end
      default: begin
	a00 = 0;
	a01 = 0;
	a02 = 0;
	      
	b00 = 0;
	b01 = 0;
	b02 = 0;
      end
    endcase
  end

  systolic #(.W(W), .N(N)) sys(.i_clk(i_clk), .i_rst(i_rst), .i_en(i_en), .i_mode(i_mode), .i_A(A_in), .i_B(B_in), .o_C(o_C), .debug_pe_a(debug_pe_a), .debug_pe_b(debug_pe_b));

  delay2 #(.WIDTH(W), .DEPTH(1)) delayA1(.clk(i_clk), .reset(i_rst), .data_in(a01), .data_out(a01_q));
  delay2 #(.WIDTH(W), .DEPTH(2)) delayA2(.clk(i_clk), .reset(i_rst), .data_in(a02), .data_out(a02_q));

  delay2 #(.WIDTH(W), .DEPTH(1)) delayB1(.clk(i_clk), .reset(i_rst), .data_in(b01), .data_out(b01_q));
  delay2 #(.WIDTH(W), .DEPTH(2)) delayB2(.clk(i_clk), .reset(i_rst), .data_in(b02), .data_out(b02_q));


endmodule
`default_nettype wire
`endif
// No pipelined/piplined MAC
// Version: 1.0

// Description:

// Function : mac_out = in_a * in_b + in_c.  Both work for INT8 and FP16 mode. Default INT8 and FP16 are signed number
// Exception : error detection for overflow and underflow in FP16 mode
`ifndef MAC_UNIT_V_
`define MAC_UNIT_V_


`timescale 1ns / 1ps

module mac_unit
(
`ifdef PIPELINE
  input            clk,
  input            rst_n,
`endif
  input     [15:0] in_a, // multiplier input1
  input     [15:0] in_b, // multiplier input2
  input     [15:0] in_c, // adder input2 ; adder input1 = in_a*in_b
  input 	   mode,
  //output    [15:0] mac_out,
  output    [15:0] mac_out,
  output 	   error
);

  wire [15:0] mul_out;

  int_fp_add add(
  `ifdef PIPELINE
    .clk   (clk    ),
    .rst_n (rst_n  ),
  `endif 
    .mode  (mode   ),
    .a     (mul_out),
    .b     (in_c   ),
    .c     (mac_out)
  );

  int_fp_mul mul(
  `ifdef PIPELINE
    .clk   (clk    ),
    .rst_n (rst_n  ),
  `endif 
    .mode  (mode   ),
    .a     (in_a   ),
    .b     (in_b   ),
    .c     (mul_out),
    .error (error  )
  );

endmodule
`endif
`ifndef PE_2_V_
`define PE_2_V_


`timescale 1ns / 1ps
`default_nettype none

module PE #(
  //parameter W = 32
  parameter W = 16
) (
  input  wire                 i_clk,
  input  wire                 i_rst,
  input  wire                 i_en,
  input  wire                 i_mode,
  input  wire [    W - 1 : 0] i_A,
  input  wire [    W - 1 : 0] i_B,
  output wire [    W - 1 : 0] o_A,
  output wire [    W - 1 : 0] o_B,
  //output wire [    W - 1 : 0] o_C
  output reg  [    W - 1 : 0] o_C
);

  //wire mode;
  //assign mode = 1;

  wire sync_load;
  assign o_A = i_A_buffered;
  assign o_B = i_B_buffered;
  assign sync_load = i_rst | ~i_en;

  wire [W - 1 : 0] i_A_buffered;
  wire [W - 1 : 0] i_B_buffered;

  reg  [15 : 0] accu;
  wire [15 : 0] mac_out;

  // Buffered in MAC
  delay2 #(.WIDTH(W), .DEPTH(1)) delayA(.clk(i_clk), .reset(i_rst), .data_in(i_A), .data_out(i_A_buffered));
  delay2 #(.WIDTH(W), .DEPTH(1)) delayB(.clk(i_clk), .reset(i_rst), .data_in(i_B), .data_out(i_B_buffered));

  always @(posedge i_clk) begin
    if (sync_load) begin
      accu <= 0;
      o_C  <= 0;
    end
    else begin
      accu <= mac_out;
      o_C  <= mac_out;
    end
  end

  // Optional: making it clocked
  mac_unit u0_mac(
    .in_a    (i_A_buffered),
    .in_b    (i_B_buffered),
    .in_c    (accu),
    .mode    (i_mode),
    .mac_out (mac_out)
  );

endmodule

`default_nettype wire
`endif
`ifndef DELAY_2_V_
`define DELAY_2_V_

`timescale 1ns / 1ps
`default_nettype none

module delay2 #(
  parameter WIDTH = 16,
  parameter DEPTH = 3
) (
  input  wire                 clk,
  input  wire                 reset,
  input  wire [WIDTH - 1 : 0] data_in,
  output wire [WIDTH - 1 : 0] data_out
);

  wire [WIDTH - 1 : 0] connect_wire [DEPTH : 0];

  assign data_out        = connect_wire[DEPTH];
  assign connect_wire[0] = data_in;

  genvar i;
  generate
    for (i = 1; i <= DEPTH; i = i + 1) begin
      dff #(.WIDTH(WIDTH)) DFF(
        .clk(clk),
        .rst(reset),
        .inp(connect_wire[i-1]),
        .outp(connect_wire[i]));
    end
  endgenerate
endmodule

// D flip-flop with synchronous reset
module dff#(
    parameter WIDTH = 1
  ) (
    input wire clk,
    input wire rst,

    input wire [WIDTH-1:0] inp,
    output reg [WIDTH-1:0] outp
  );

  always @(posedge clk) begin
    outp <= rst ? 0 : inp;
  end

endmodule

`default_nettype wire
`endif
`ifndef MUL_2x2_V_
`define MUL_2x2_V_

`timescale 1ns / 1ps

module mul2x2(
  input  [1:0] a,
  input  [1:0] b,
  output [3:0] c
);

  wire [3:0] tmp;

  assign tmp[0] = a[0] & b[0];
  assign tmp[1] = (a[1]&b[0]) ^ (a[0]&b[1]);
  assign tmp[2] = (a[0]&b[1]) & (a[1]&b[0]) ^ (a[1]&b[1]);
  assign tmp[3] = (a[0]&b[1]) & (a[1]&b[0]) & (a[1]&b[1]);
  assign c 	= {tmp[3],tmp[2],tmp[1],tmp[0]};

endmodule
`endif
`ifndef MUL_4x4_V_
`define MUL_4x4_V_

`timescale 1ns / 1ps

module mul4x4(
  input  [3:0] a,
  input  [3:0] b,
  output [7:0] c
);

  wire [15:0] tmp1;
  wire [ 5:0] result1;
  wire [ 5:0] result2;
  wire 	      co1,co2,co3;

  mul2x2 u1(a[3:2],b[3:2],tmp1[15:12]);
  mul2x2 u2(a[1:0],b[3:2],tmp1[11:8]);
  mul2x2 u3(a[3:2],b[1:0],tmp1[7:4]);
  mul2x2 u4(a[1:0],b[1:0],tmp1[3:0]);

  cla_nbit #(.n(6)) u5({tmp1[15:12],2'b0},{2'b0,tmp1[11:8]},1'b0	,result1	,co1);
  cla_nbit #(.n(6)) u6({2'b0,tmp1[7:4]}  ,{4'b0,tmp1[3:2]} ,co1 	,result2	,co2);
  cla_nbit #(.n(6)) u7(result1           ,result2	   ,co2 	,c[7:2] 	,co3);

  assign c[1:0] = tmp1[1:0];

endmodule
`endif
`ifndef MUL_8x8_V_
`define MUL_8x8_V_

`timescale 1ns / 1ps

module mul8x8(
  input  [ 7:0] a,
  input  [ 7:0] b,
  output [15:0] c
);

  wire [31:0] tmp1;
  wire [11:0] result1;
  wire [11:0] result2;
  wire        co1,co2,co3;

  mul4x4 u1(a[7:4],b[7:4],tmp1[31:24]);
  mul4x4 u2(a[3:0],b[7:4],tmp1[23:16]);
  mul4x4 u3(a[7:4],b[3:0],tmp1[15:8]);
  mul4x4 u4(a[3:0],b[3:0],tmp1[7:0]);

  cla_nbit #(.n(12)) u5({tmp1[31:24],4'b0} ,{4'b0,tmp1[23:16]} ,1'b0 ,result1 ,co1);
  cla_nbit #(.n(12)) u6({4'b0,tmp1[15:8]}  ,{8'b0,tmp1[7:4]}   ,co1  ,result2 ,co2);
  cla_nbit #(.n(12)) u7(result1		   ,result2	       ,co2  ,c[15:4] ,co3);

  assign c[3:0] = tmp1[3:0];

endmodule
`endif
`ifndef MUL_16x16_V_
`define MUL_16x16_V_

`timescale 1ns / 1ps

module mul16x16(
`ifdef PIPLINE
  input clk,
  input rst_n,
`endif
  input  [15:0] a,
  input  [15:0] b,
  output [31:0] c);

  wire [63:0] tmp1,tmp2;
  wire [23:0] result1;
  wire [23:0] result2;
  wire co1,co2,co3;

`ifdef PIPLINE
  // one stage pipline
  reg [63:0] tmp1_reg;
  always @ (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tmp1_reg <= 64'b0;
    end else begin
      tmp1_reg <= tmp1;
    end
  end
  assign tmp2 = tmp1_reg;

`else 
  assign tmp2 = tmp1;

`endif

  mul8x8 u1(a[15:8],b[15:8],tmp1[63:48]);
  mul8x8 u2(a[7:0] ,b[15:8],tmp1[47:32]);
  mul8x8 u3(a[15:8],b[ 7:0],tmp1[31:16]);
  mul8x8 u4(a[7:0] ,b[ 7:0],tmp1[15:0]);

  cla_nbit #(.n(24)) u5({tmp2[63:48],8'b0} ,{8'b0,tmp2[47:32]} ,1'b0 ,result1 ,co1);
  cla_nbit #(.n(24)) u6({8'b0,tmp2[31:16]} ,{16'b0,tmp2[15:8]} ,co1  ,result2 ,co2);
  cla_nbit #(.n(24)) u7(result1            ,result2            ,co2  ,c[31:8] ,co3);

  assign c[7:0] = tmp2[7:0];

endmodule
`endif
`ifndef ALIGNMENT_V_
`define ALIGNMENT_V_

`timescale 1ns / 1ps

module alignment (
  input  [14:0] bigger, 
  input  [14:0] smaller,
  output [10:0] aligned_small
);

  wire c1;
  wire [4:0] bigger_exponent, smaller_exponent, shift_bits;

  assign bigger_exponent  = bigger  [14:10];
  assign smaller_exponent = smaller [14:10];
  assign aligned_small    = ({1'b1,smaller[9:0]} >> shift_bits);

  cla_nbit #(.n(5)) u1(bigger_exponent,~smaller_exponent+1'b1,1'b0,shift_bits,c1);

endmodule
`endif
`ifndef SYSTOLIC_V_
`define SYSTOLIC_V_


`timescale 1ns / 1ps
`default_nettype none

// row
// 3 x 3
module systolic #(
  parameter W = 16,
  parameter N = 3
) (
  input  wire                         i_clk,
  input  wire                         i_rst,
  input  wire                         i_en,
  input  wire                         i_mode,
  input  wire [        W * N - 1 : 0] i_A,
  input  wire [        W * N - 1 : 0] i_B,
  output wire [    W * N * N - 1 : 0] o_C,

  // debug
  output wire [    W * N * 2 - 1 : 0] debug_pe_a,
  output wire [    W * N * 2 - 1 : 0] debug_pe_b
);

  //localparam O_VEC_WIDTH = 2 * W;
  localparam O_VEC_WIDTH = W;

  wire [W - 1 : 0] a00, a01, a02, b00, b01, b02;
  wire [W - 1 : 0] pe_a_00_01, pe_a_01_02, pe_a_10_11, pe_a_11_12, pe_a_20_21, pe_a_21_22;
  wire [W - 1 : 0] pe_b_00_10, pe_b_01_11, pe_b_02_12, pe_b_10_20, pe_b_11_21, pe_b_12_22;

  wire [O_VEC_WIDTH - 1 : 0] c00, c01, c02, c10, c11, c12, c20, c21, c22;

  assign a00 = i_A[0 * W +: W];
  assign a01 = i_A[1 * W +: W];
  assign a02 = i_A[2 * W +: W];

  assign b00 = i_B[0 * W +: W];
  assign b01 = i_B[1 * W +: W];
  assign b02 = i_B[2 * W +: W];

  PE #(.W(W)) PE00(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(a00),        .i_B(b00), .o_A(pe_a_00_01),.o_B(pe_b_00_10),.o_C(c00));
  PE #(.W(W)) PE01(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_00_01), .i_B(b01), .o_A(pe_a_01_02),.o_B(pe_b_01_11),.o_C(c01));
  PE #(.W(W)) PE02(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_01_02), .i_B(b02), .o_A(),          .o_B(pe_b_02_12),.o_C(c02));

  PE #(.W(W)) PE10(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(a01),       .i_B(pe_b_00_10),.o_A(pe_a_10_11),.o_B(pe_b_10_20),.o_C(c10));
  PE #(.W(W)) PE11(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_10_11),.i_B(pe_b_01_11),.o_A(pe_a_11_12),.o_B(pe_b_11_21),.o_C(c11));
  PE #(.W(W)) PE12(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_11_12),.i_B(pe_b_02_12),.o_A(),          .o_B(pe_b_12_22),.o_C(c12));

  PE #(.W(W)) PE20(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(a02),       .i_B(pe_b_10_20),.o_A(pe_a_20_21),.o_B(),.o_C(c20));
  PE #(.W(W)) PE21(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_20_21),.i_B(pe_b_11_21),.o_A(pe_a_21_22),.o_B(),.o_C(c21));
  PE #(.W(W)) PE22(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_21_22),.i_B(pe_b_12_22),.o_A(),          .o_B(),.o_C(c22));

  
  // https://stackoverflow.com/questions/18067571/indexing-vectors-and-arrays-with
  // https://standards.ieee.org/ieee/1800/6700/
  // a_vect[ 0 +: 8] // == a_vect[ 7 : 0]
  //assign o_C[1 * O_VEC_WIDTH - 1 -: O_VEC_WIDTH] = c00;
  
  assign o_C[0 * O_VEC_WIDTH +: O_VEC_WIDTH] = c00;
  assign o_C[1 * O_VEC_WIDTH +: O_VEC_WIDTH] = c01;
  assign o_C[2 * O_VEC_WIDTH +: O_VEC_WIDTH] = c02;
  assign o_C[3 * O_VEC_WIDTH +: O_VEC_WIDTH] = c10;
  assign o_C[4 * O_VEC_WIDTH +: O_VEC_WIDTH] = c11;
  assign o_C[5 * O_VEC_WIDTH +: O_VEC_WIDTH] = c12;
  assign o_C[6 * O_VEC_WIDTH +: O_VEC_WIDTH] = c20;
  assign o_C[7 * O_VEC_WIDTH +: O_VEC_WIDTH] = c21;
  assign o_C[8 * O_VEC_WIDTH +: O_VEC_WIDTH] = c22;

  assign debug_pe_a = {pe_a_00_01, pe_a_01_02, pe_a_10_11, pe_a_11_12, pe_a_20_21, pe_a_21_22};
  assign debug_pe_b = {pe_b_00_10, pe_b_01_11, pe_b_02_12, pe_b_10_20, pe_b_11_21, pe_b_12_22};

endmodule
`endif
`ifndef CLA_NBIT_V_
`define CLA_NBIT_V_

`timescale 1ns / 1ps

// Carry Look-ahead adder (CLA)
module cla_nbit #(
  parameter n = 4
) (
  input   [n-1:0] a,
  input   [n-1:0] b,
  input           ci,
  output  [n-1:0] s,
  output          co
);

  wire [n-1:0] g;
  wire [n-1:0] p;
  wire [  n:0] c;

  assign c[0] = ci;
  assign co   = c[n];

  genvar i;  /* i - generate index variable */

  generate
    for (i = 0; i < n; i = i + 1) begin : addbit
      assign s[i] = a[i] ^ b[i] ^ c[i];
      assign g[i] = a[i] & b[i];
      assign p[i] = a[i] | b[i];
      assign c[i + 1] = g[i] | (p[i] & c[i]);
    end
  endgenerate
  
endmodule
`endif
`ifndef INT_FP_ADD_V_
`define INT_FP_ADD_V_


`timescale 1ns / 1ps

module int_fp_add (
`ifdef PIPELINE
  input         clk,
  input         rst_n,
`endif
  input         mode,
  input  [15:0] a,
  input  [15:0] b,
  output [15:0] c
);

  wire [10:0] adder_input_1,adder_input_2,aligned_small,adder_output;
  wire if_sub,a_sign, b_sign, c_sign,c1, c2;
  wire [15:0] normalized_out;

  // only used in INT8 MAC mode
  wire [4:0] higher_add,higher_a,higher_b;

  wire [15:0] result;
  reg [14:0] bigger, smaller;
  reg a_larger_b;

`ifdef PIPELINE
  reg [14:0] bigger_reg, smaller_reg;
  reg [10:0] adder_output_reg;
  wire [14:0] bigger_tmp, smaller_tmp;
  wire [10:0] adder_output_tmp;
`endif  


  assign a_sign        = a[15];
  assign b_sign        = b[15];
  assign if_sub        = (a_sign == b_sign) ? 1'b0 : 1'b1;
  assign c_sign        = a_larger_b ? a_sign : b_sign;
  assign higher_a      = (mode == 1'b0) ? a[15:11] : 5'b0;
  assign higher_b      = (mode == 1'b0) ? b[15:11] : 5'b0;
  assign adder_input_1 = (mode==1'b0) ? a[10:0] :{1'b1,bigger[9:0]};
  assign adder_input_2 = (mode==1'b0) ? b[10:0] : (if_sub ? ~aligned_small + 1'b1 : aligned_small);
  assign c             = (mode == 1'b0) ? {higher_add,adder_output} : result;

  //compare two number regardless sign
  always @(*) begin
    if (a[14:0] > b[14:0]) begin
      bigger = a[14:0];
      smaller = b[14:0];
      a_larger_b = 1'b1;
    end else begin 
      bigger = b[14:0];
      smaller = a[14:0];
      a_larger_b = 1'b0;
    end 
  end

`ifdef PIPELINE 
    always @ (posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        bigger_reg <= 15'b0;
        smaller_reg <= 15'b0;
        adder_output_reg <= 11'b0;
      end else begin
        bigger_reg <= bigger;
        smaller_reg <= smaller;
        adder_output_reg <= adder_output;
      end
    end
    assign bigger_tmp = bigger_reg[14:0];
    assign smaller_tmp = smaller_reg[14:0];
    assign adder_output_tmp = adder_output_reg[10:0];
`endif

`ifdef PIPELINE
  // align small number
  alignment u1(bigger_tmp,smaller_tmp,aligned_small);
`else 
  // align small number
  alignment u1(bigger,smaller,aligned_small);
`endif

  cla_nbit #(.n(11)) u2(adder_input_1,adder_input_2,1'b0,adder_output,c1);

  // This 5 bit adder only used in INT8 MAC mode
  cla_nbit #(.n(5)) u3(higher_a,higher_b,c1,higher_add,c2);

`ifdef PIPELINE
  add_normalizer u4(c_sign,bigger[14:10],adder_output_tmp,result,c1,if_sub);
`else 
  add_normalizer u4(c_sign,bigger[14:10],adder_output,result,c1,if_sub);
`endif

endmodule
`endif
`ifndef INT_FP_MUL_V_
`define INT_FP_MUL_V_


module int_fp_mul (
`ifdef PIPELINE
  input         clk,
  input         rst_n,
`endif
  input         mode,
  input  [15:0] a,
  input  [15:0] b,
  output [15:0] c,
  output        error // valid in fp16 mode 
);

  wire [15:0] c_tmp;
  wire        c_sign,a_zero,b_zero;
  wire [ 4:0] sum_exponent, biased_sum_exponent;
  wire [15:0] multiplier_input1,multiplier_input2;

  wire [31:0] multiplier_output;
  wire [14:0] normalized_out;
  wire [21:0] mantissa_prod;
  wire c1,c2,underflow,overflow;

  assign overflow = (c1 && c2 && ~biased_sum_exponent[4]) ? 1'b1 :1'b0;
  assign underflow = (~c1 && ~c2 && biased_sum_exponent[4]) ? 1'b1:1'b0;

  assign a_zero = ~(|a);
  assign b_zero = ~(|b);
  assign c_sign = a[15] ^ b[15];
  assign multiplier_input1 = mode ? {5'b0,1'b1,a[9:0]} : ((a[7]==1'b0) ? {9'b0,a[6:0]} : {9'b0,~a[6:0]+1'b1});
  assign multiplier_input2 = mode ? {5'b0,1'b1,b[9:0]} : ((b[7]==1'b0) ? {9'b0,b[6:0]} : {9'b0,~b[6:0]+1'b1});

  assign c = mode ? ((a_zero | b_zero) ? 16'b0 : c_tmp) : ((a[7]^b[7] == 1'b0) ? multiplier_output[15:0] : {1'b1,~multiplier_output[14:0]+1'b1});
  //error detect
  assign c_tmp = (~error) ? {c_sign,normalized_out} : (underflow ? {c_sign,15'b0000_0000_0000_000} : {c_sign,5'b1111_1,10'b0000_0000_00});

  assign error = overflow | underflow; 

    
`ifdef PIPELINE

  reg [31:0] multiplier_output_tmp;

  always @ (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      multiplier_output_tmp <= 32'b0;
    end else begin
      multiplier_output_tmp <= multiplier_output;
    end
  end

  assign mantissa_prod = multiplier_output_tmp[21:0];
  mul16x16 u1(clk,rst_n,multiplier_input1,multiplier_input2,multiplier_output);

`else 

  assign mantissa_prod = multiplier_output[21:0];
  mul16x16 u1(multiplier_input1,multiplier_input2,multiplier_output);

`endif
    
  cla_nbit #(.n(5)) u2(a[14:10],b[14:10],1'b0,sum_exponent,c1); // add exponent
  cla_nbit #(.n(5)) u3(sum_exponent, 5'b10001,1'b0,biased_sum_exponent,c2); // minus bias
  mul_normalizer u4(biased_sum_exponent,mantissa_prod,normalized_out);

endmodule
`endif
`ifndef MUL_NORMALIZER_V_
`define MUL_NORMALIZER_V_

`timescale 1ns / 1ps

module mul_normalizer (
  input  [ 4:0] exponent,
  input  [21:0] mantissa_prod,
  output [14:0] result
);

  wire [4:0] result_exponent;
  wire [9:0] result_mantissa;

  assign result_exponent = (mantissa_prod[21]) ? (exponent + 1'b1): exponent;
  assign result_mantissa = (mantissa_prod[21]) ? mantissa_prod[20:11]:mantissa_prod[19:10];
  assign result          = {result_exponent,result_mantissa};

// No rounding and No overflow/underflow detection

endmodule
`endif
`ifndef ADD_NORMALIZER_V_
`define ADD_NORMALIZER_V_

`timescale 1ns / 1ps

module add_normalizer (
  input             sign,
  input      [ 4:0] exponent,
  input      [10:0] mantissa_add,
  output reg [15:0] result,
  input             if_carray,
  input             if_sub
);

  reg [4:0] number_of_zero_lead;
  reg [10:0] norm_mantissa_add;
  reg [9:0] mantissa_tmp;

  wire [4:0] shift_left_exp;
  wire c1;

  always @ (*) begin
    if (mantissa_add[10:4] == 7'b0000_001) begin
      number_of_zero_lead = 5'd6;
      norm_mantissa_add   = (mantissa_add << 4'd6);
    end else if (mantissa_add[10:5] == 6'b0000_01) begin 
      number_of_zero_lead = 5'd5;
      norm_mantissa_add   = (mantissa_add << 4'd5);
    end else if (mantissa_add[10:6] == 5'b0000_1) begin
      number_of_zero_lead = 5'd4;
      norm_mantissa_add   = (mantissa_add << 4'd4);
    end else if (mantissa_add[10:7] == 4'b0001) begin
      number_of_zero_lead = 5'd3;
      norm_mantissa_add   = (mantissa_add << 4'd3);
    end else if (mantissa_add[10:8] == 3'b001) begin
      number_of_zero_lead = 5'd2;
      norm_mantissa_add   = (mantissa_add << 4'd2);
    end else if (mantissa_add[10:9] == 2'b01) begin
      number_of_zero_lead = 5'd1;
      norm_mantissa_add   = (mantissa_add << 4'd1);
    end else begin 
      number_of_zero_lead = 5'd0;
      norm_mantissa_add   = mantissa_add[10:0];
    end 
  end

  always @(*) begin
    result[15]      = sign;
    if (!if_sub) begin 
      result[14:10] = if_carray ? exponent + 1'b1 : exponent;
      result[9:0]   = if_carray ? mantissa_add[10:1] : mantissa_add[9:0];
    end else begin 
      result[14:10] = shift_left_exp;
      result[9:0]   = norm_mantissa_add[9:0];
    end 
  end

  cla_nbit #(.n(5)) u1(exponent,~number_of_zero_lead+1'b1,1'b0,shift_left_exp,c1);

endmodule

`endif
`ifndef CONTROL_V_
`define CONTROL_V_


`timescale 1ns / 1ps
`default_nettype none

// Really 3x3, a done output??


// 6 logic cycles + 2 (buffered?) delay cycles
module control #(
  parameter W = 16,
  parameter N = 3
) (
  input  wire                         i_clk,
  input  wire                         i_rst,
  input  wire                         i_en,
  input  wire                         i_mode,
  input  wire [    W * N * N - 1 : 0] i_A,
  input  wire [    W * N * N - 1 : 0] i_B,
  output wire [    W * N * N - 1 : 0] o_C,
  output wire                         o_done,
  output wire [    W * N     - 1 : 0] A_in,
  output wire [    W * N     - 1 : 0] B_in,

  // debug
  output wire [    W * N * 2 - 1 : 0] debug_pe_a,
  output wire [    W * N * 2 - 1 : 0] debug_pe_b
);
  
  reg [3 : 0] states, next_states;

  reg  [W - 1 : 0] a00, a01, a02;
  wire [W - 1 : 0] a01_q, a02_q;

  reg  [W - 1 : 0] b00, b01, b02;
  wire [W - 1 : 0] b01_q, b02_q;

  assign A_in = {a02_q, a01_q, a00};
  assign B_in = {b02_q, b01_q, b00};  

  // 0000 is the idle / standby state
  always @(posedge i_clk) begin
    if (i_rst | ~i_en) begin
      states <= 4'b0000;
    end
    else if (i_en) begin
      states <= next_states;
    end
  end
  
  always @(*) begin
    if (next_states == 4'b1001) begin
      // Done: Force waiting
      // This can also be done by switching off input
      next_states = 4'b1001;
    end else begin
      next_states = states + 4'b0001;
    end
  end

  assign o_done = (states == 4'b1001);	
  
  always @(*) begin
    case (states)
      4'b0001: begin
        a00 = i_A[    W - 1 : 0 * W];
        a01 = i_A[2 * W - 1 : 1 * W];
        a02 = i_A[3 * W - 1 : 2 * W];
       
        
        b00 = i_B[    W - 1 : 0 * W];
        b01 = i_B[2 * W - 1 : 1 * W];
        b02 = i_B[3 * W - 1 : 2 * W];
      end
      4'b0010: begin
        a00 = i_A[4 * W - 1 : 3 * W];
        a01 = i_A[5 * W - 1 : 4 * W];
        a02 = i_A[6 * W - 1 : 5 * W];
  
        b00 = i_B[4 * W - 1 : 3 * W];
        b01 = i_B[5 * W - 1 : 4 * W];
        b02 = i_B[6 * W - 1 : 5 * W];
      end
      4'b0011: begin
        a00 = i_A[7 * W - 1 : 6 * W];
        a01 = i_A[8 * W - 1 : 7 * W];
        a02 = i_A[9 * W - 1 : 8 * W];
        
        b00 = i_B[7 * W - 1 : 6 * W];
        b01 = i_B[8 * W - 1 : 7 * W];
        b02 = i_B[9 * W - 1 : 8 * W];
      end
      default: begin
	a00 = 0;
	a01 = 0;
	a02 = 0;
	      
	b00 = 0;
	b01 = 0;
	b02 = 0;
      end
    endcase
  end

  systolic #(.W(W), .N(N)) sys(.i_clk(i_clk), .i_rst(i_rst), .i_en(i_en), .i_mode(i_mode), .i_A(A_in), .i_B(B_in), .o_C(o_C), .debug_pe_a(debug_pe_a), .debug_pe_b(debug_pe_b));

  delay2 #(.WIDTH(W), .DEPTH(1)) delayA1(.clk(i_clk), .reset(i_rst), .data_in(a01), .data_out(a01_q));
  delay2 #(.WIDTH(W), .DEPTH(2)) delayA2(.clk(i_clk), .reset(i_rst), .data_in(a02), .data_out(a02_q));

  delay2 #(.WIDTH(W), .DEPTH(1)) delayB1(.clk(i_clk), .reset(i_rst), .data_in(b01), .data_out(b01_q));
  delay2 #(.WIDTH(W), .DEPTH(2)) delayB2(.clk(i_clk), .reset(i_rst), .data_in(b02), .data_out(b02_q));


endmodule
`default_nettype wire
`endif
// No pipelined/piplined MAC
// Version: 1.0

// Description:

// Function : mac_out = in_a * in_b + in_c.  Both work for INT8 and FP16 mode. Default INT8 and FP16 are signed number
// Exception : error detection for overflow and underflow in FP16 mode
`ifndef MAC_UNIT_V_
`define MAC_UNIT_V_


`timescale 1ns / 1ps

module mac_unit
(
`ifdef PIPELINE
  input            clk,
  input            rst_n,
`endif
  input     [15:0] in_a, // multiplier input1
  input     [15:0] in_b, // multiplier input2
  input     [15:0] in_c, // adder input2 ; adder input1 = in_a*in_b
  input 	   mode,
  //output    [15:0] mac_out,
  output    [15:0] mac_out,
  output 	   error
);

  wire [15:0] mul_out;

  int_fp_add add(
  `ifdef PIPELINE
    .clk   (clk    ),
    .rst_n (rst_n  ),
  `endif 
    .mode  (mode   ),
    .a     (mul_out),
    .b     (in_c   ),
    .c     (mac_out)
  );

  int_fp_mul mul(
  `ifdef PIPELINE
    .clk   (clk    ),
    .rst_n (rst_n  ),
  `endif 
    .mode  (mode   ),
    .a     (in_a   ),
    .b     (in_b   ),
    .c     (mul_out),
    .error (error  )
  );

endmodule
`endif
`ifndef PE_2_V_
`define PE_2_V_


`timescale 1ns / 1ps
`default_nettype none

module PE #(
  //parameter W = 32
  parameter W = 16
) (
  input  wire                 i_clk,
  input  wire                 i_rst,
  input  wire                 i_en,
  input  wire                 i_mode,
  input  wire [    W - 1 : 0] i_A,
  input  wire [    W - 1 : 0] i_B,
  output wire [    W - 1 : 0] o_A,
  output wire [    W - 1 : 0] o_B,
  //output wire [    W - 1 : 0] o_C
  output reg  [    W - 1 : 0] o_C
);

  //wire mode;
  //assign mode = 1;

  wire sync_load;
  assign o_A = i_A_buffered;
  assign o_B = i_B_buffered;
  assign sync_load = i_rst | ~i_en;

  wire [W - 1 : 0] i_A_buffered;
  wire [W - 1 : 0] i_B_buffered;

  reg  [15 : 0] accu;
  wire [15 : 0] mac_out;

  // Buffered in MAC
  delay2 #(.WIDTH(W), .DEPTH(1)) delayA(.clk(i_clk), .reset(i_rst), .data_in(i_A), .data_out(i_A_buffered));
  delay2 #(.WIDTH(W), .DEPTH(1)) delayB(.clk(i_clk), .reset(i_rst), .data_in(i_B), .data_out(i_B_buffered));

  always @(posedge i_clk) begin
    if (sync_load) begin
      accu <= 0;
      o_C  <= 0;
    end
    else begin
      accu <= mac_out;
      o_C  <= mac_out;
    end
  end

  // Optional: making it clocked
  mac_unit u0_mac(
    .in_a    (i_A_buffered),
    .in_b    (i_B_buffered),
    .in_c    (accu),
    .mode    (i_mode),
    .mac_out (mac_out)
  );

endmodule

`default_nettype wire
`endif
`ifndef DELAY_2_V_
`define DELAY_2_V_

`timescale 1ns / 1ps
`default_nettype none

module delay2 #(
  parameter WIDTH = 16,
  parameter DEPTH = 3
) (
  input  wire                 clk,
  input  wire                 reset,
  input  wire [WIDTH - 1 : 0] data_in,
  output wire [WIDTH - 1 : 0] data_out
);

  wire [WIDTH - 1 : 0] connect_wire [DEPTH : 0];

  assign data_out        = connect_wire[DEPTH];
  assign connect_wire[0] = data_in;

  genvar i;
  generate
    for (i = 1; i <= DEPTH; i = i + 1) begin
      dff #(.WIDTH(WIDTH)) DFF(
        .clk(clk),
        .rst(reset),
        .inp(connect_wire[i-1]),
        .outp(connect_wire[i]));
    end
  endgenerate
endmodule

// D flip-flop with synchronous reset
module dff#(
    parameter WIDTH = 1
  ) (
    input wire clk,
    input wire rst,

    input wire [WIDTH-1:0] inp,
    output reg [WIDTH-1:0] outp
  );

  always @(posedge clk) begin
    outp <= rst ? 0 : inp;
  end

endmodule

`default_nettype wire
`endif
`ifndef MUL_2x2_V_
`define MUL_2x2_V_

`timescale 1ns / 1ps

module mul2x2(
  input  [1:0] a,
  input  [1:0] b,
  output [3:0] c
);

  wire [3:0] tmp;

  assign tmp[0] = a[0] & b[0];
  assign tmp[1] = (a[1]&b[0]) ^ (a[0]&b[1]);
  assign tmp[2] = (a[0]&b[1]) & (a[1]&b[0]) ^ (a[1]&b[1]);
  assign tmp[3] = (a[0]&b[1]) & (a[1]&b[0]) & (a[1]&b[1]);
  assign c 	= {tmp[3],tmp[2],tmp[1],tmp[0]};

endmodule
`endif
`ifndef MUL_4x4_V_
`define MUL_4x4_V_

`timescale 1ns / 1ps

module mul4x4(
  input  [3:0] a,
  input  [3:0] b,
  output [7:0] c
);

  wire [15:0] tmp1;
  wire [ 5:0] result1;
  wire [ 5:0] result2;
  wire 	      co1,co2,co3;

  mul2x2 u1(a[3:2],b[3:2],tmp1[15:12]);
  mul2x2 u2(a[1:0],b[3:2],tmp1[11:8]);
  mul2x2 u3(a[3:2],b[1:0],tmp1[7:4]);
  mul2x2 u4(a[1:0],b[1:0],tmp1[3:0]);

  cla_nbit #(.n(6)) u5({tmp1[15:12],2'b0},{2'b0,tmp1[11:8]},1'b0	,result1	,co1);
  cla_nbit #(.n(6)) u6({2'b0,tmp1[7:4]}  ,{4'b0,tmp1[3:2]} ,co1 	,result2	,co2);
  cla_nbit #(.n(6)) u7(result1           ,result2	   ,co2 	,c[7:2] 	,co3);

  assign c[1:0] = tmp1[1:0];

endmodule
`endif
`ifndef MUL_8x8_V_
`define MUL_8x8_V_

`timescale 1ns / 1ps

module mul8x8(
  input  [ 7:0] a,
  input  [ 7:0] b,
  output [15:0] c
);

  wire [31:0] tmp1;
  wire [11:0] result1;
  wire [11:0] result2;
  wire        co1,co2,co3;

  mul4x4 u1(a[7:4],b[7:4],tmp1[31:24]);
  mul4x4 u2(a[3:0],b[7:4],tmp1[23:16]);
  mul4x4 u3(a[7:4],b[3:0],tmp1[15:8]);
  mul4x4 u4(a[3:0],b[3:0],tmp1[7:0]);

  cla_nbit #(.n(12)) u5({tmp1[31:24],4'b0} ,{4'b0,tmp1[23:16]} ,1'b0 ,result1 ,co1);
  cla_nbit #(.n(12)) u6({4'b0,tmp1[15:8]}  ,{8'b0,tmp1[7:4]}   ,co1  ,result2 ,co2);
  cla_nbit #(.n(12)) u7(result1		   ,result2	       ,co2  ,c[15:4] ,co3);

  assign c[3:0] = tmp1[3:0];

endmodule
`endif
`ifndef MUL_16x16_V_
`define MUL_16x16_V_

`timescale 1ns / 1ps

module mul16x16(
`ifdef PIPLINE
  input clk,
  input rst_n,
`endif
  input  [15:0] a,
  input  [15:0] b,
  output [31:0] c);

  wire [63:0] tmp1,tmp2;
  wire [23:0] result1;
  wire [23:0] result2;
  wire co1,co2,co3;

`ifdef PIPLINE
  // one stage pipline
  reg [63:0] tmp1_reg;
  always @ (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tmp1_reg <= 64'b0;
    end else begin
      tmp1_reg <= tmp1;
    end
  end
  assign tmp2 = tmp1_reg;

`else 
  assign tmp2 = tmp1;

`endif

  mul8x8 u1(a[15:8],b[15:8],tmp1[63:48]);
  mul8x8 u2(a[7:0] ,b[15:8],tmp1[47:32]);
  mul8x8 u3(a[15:8],b[ 7:0],tmp1[31:16]);
  mul8x8 u4(a[7:0] ,b[ 7:0],tmp1[15:0]);

  cla_nbit #(.n(24)) u5({tmp2[63:48],8'b0} ,{8'b0,tmp2[47:32]} ,1'b0 ,result1 ,co1);
  cla_nbit #(.n(24)) u6({8'b0,tmp2[31:16]} ,{16'b0,tmp2[15:8]} ,co1  ,result2 ,co2);
  cla_nbit #(.n(24)) u7(result1            ,result2            ,co2  ,c[31:8] ,co3);

  assign c[7:0] = tmp2[7:0];

endmodule
`endif
`ifndef ALIGNMENT_V_
`define ALIGNMENT_V_

`timescale 1ns / 1ps

module alignment (
  input  [14:0] bigger, 
  input  [14:0] smaller,
  output [10:0] aligned_small
);

  wire c1;
  wire [4:0] bigger_exponent, smaller_exponent, shift_bits;

  assign bigger_exponent  = bigger  [14:10];
  assign smaller_exponent = smaller [14:10];
  assign aligned_small    = ({1'b1,smaller[9:0]} >> shift_bits);

  cla_nbit #(.n(5)) u1(bigger_exponent,~smaller_exponent+1'b1,1'b0,shift_bits,c1);

endmodule
`endif
`ifndef SYSTOLIC_V_
`define SYSTOLIC_V_


`timescale 1ns / 1ps
`default_nettype none

// row
// 3 x 3
module systolic #(
  parameter W = 16,
  parameter N = 3
) (
  input  wire                         i_clk,
  input  wire                         i_rst,
  input  wire                         i_en,
  input  wire                         i_mode,
  input  wire [        W * N - 1 : 0] i_A,
  input  wire [        W * N - 1 : 0] i_B,
  output wire [    W * N * N - 1 : 0] o_C,

  // debug
  output wire [    W * N * 2 - 1 : 0] debug_pe_a,
  output wire [    W * N * 2 - 1 : 0] debug_pe_b
);

  //localparam O_VEC_WIDTH = 2 * W;
  localparam O_VEC_WIDTH = W;

  wire [W - 1 : 0] a00, a01, a02, b00, b01, b02;
  wire [W - 1 : 0] pe_a_00_01, pe_a_01_02, pe_a_10_11, pe_a_11_12, pe_a_20_21, pe_a_21_22;
  wire [W - 1 : 0] pe_b_00_10, pe_b_01_11, pe_b_02_12, pe_b_10_20, pe_b_11_21, pe_b_12_22;

  wire [O_VEC_WIDTH - 1 : 0] c00, c01, c02, c10, c11, c12, c20, c21, c22;

  assign a00 = i_A[0 * W +: W];
  assign a01 = i_A[1 * W +: W];
  assign a02 = i_A[2 * W +: W];

  assign b00 = i_B[0 * W +: W];
  assign b01 = i_B[1 * W +: W];
  assign b02 = i_B[2 * W +: W];

  PE #(.W(W)) PE00(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(a00),        .i_B(b00), .o_A(pe_a_00_01),.o_B(pe_b_00_10),.o_C(c00));
  PE #(.W(W)) PE01(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_00_01), .i_B(b01), .o_A(pe_a_01_02),.o_B(pe_b_01_11),.o_C(c01));
  PE #(.W(W)) PE02(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_01_02), .i_B(b02), .o_A(),          .o_B(pe_b_02_12),.o_C(c02));

  PE #(.W(W)) PE10(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(a01),       .i_B(pe_b_00_10),.o_A(pe_a_10_11),.o_B(pe_b_10_20),.o_C(c10));
  PE #(.W(W)) PE11(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_10_11),.i_B(pe_b_01_11),.o_A(pe_a_11_12),.o_B(pe_b_11_21),.o_C(c11));
  PE #(.W(W)) PE12(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_11_12),.i_B(pe_b_02_12),.o_A(),          .o_B(pe_b_12_22),.o_C(c12));

  PE #(.W(W)) PE20(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(a02),       .i_B(pe_b_10_20),.o_A(pe_a_20_21),.o_B(),.o_C(c20));
  PE #(.W(W)) PE21(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_20_21),.i_B(pe_b_11_21),.o_A(pe_a_21_22),.o_B(),.o_C(c21));
  PE #(.W(W)) PE22(.i_clk(i_clk),.i_rst(i_rst),.i_en(i_en), .i_mode(i_mode), .i_A(pe_a_21_22),.i_B(pe_b_12_22),.o_A(),          .o_B(),.o_C(c22));

  
  // https://stackoverflow.com/questions/18067571/indexing-vectors-and-arrays-with
  // https://standards.ieee.org/ieee/1800/6700/
  // a_vect[ 0 +: 8] // == a_vect[ 7 : 0]
  //assign o_C[1 * O_VEC_WIDTH - 1 -: O_VEC_WIDTH] = c00;
  
  assign o_C[0 * O_VEC_WIDTH +: O_VEC_WIDTH] = c00;
  assign o_C[1 * O_VEC_WIDTH +: O_VEC_WIDTH] = c01;
  assign o_C[2 * O_VEC_WIDTH +: O_VEC_WIDTH] = c02;
  assign o_C[3 * O_VEC_WIDTH +: O_VEC_WIDTH] = c10;
  assign o_C[4 * O_VEC_WIDTH +: O_VEC_WIDTH] = c11;
  assign o_C[5 * O_VEC_WIDTH +: O_VEC_WIDTH] = c12;
  assign o_C[6 * O_VEC_WIDTH +: O_VEC_WIDTH] = c20;
  assign o_C[7 * O_VEC_WIDTH +: O_VEC_WIDTH] = c21;
  assign o_C[8 * O_VEC_WIDTH +: O_VEC_WIDTH] = c22;

  assign debug_pe_a = {pe_a_00_01, pe_a_01_02, pe_a_10_11, pe_a_11_12, pe_a_20_21, pe_a_21_22};
  assign debug_pe_b = {pe_b_00_10, pe_b_01_11, pe_b_02_12, pe_b_10_20, pe_b_11_21, pe_b_12_22};

endmodule
`endif
`ifndef CLA_NBIT_V_
`define CLA_NBIT_V_

`timescale 1ns / 1ps

// Carry Look-ahead adder (CLA)
module cla_nbit #(
  parameter n = 4
) (
  input   [n-1:0] a,
  input   [n-1:0] b,
  input           ci,
  output  [n-1:0] s,
  output          co
);

  wire [n-1:0] g;
  wire [n-1:0] p;
  wire [  n:0] c;

  assign c[0] = ci;
  assign co   = c[n];

  genvar i;  /* i - generate index variable */

  generate
    for (i = 0; i < n; i = i + 1) begin : addbit
      assign s[i] = a[i] ^ b[i] ^ c[i];
      assign g[i] = a[i] & b[i];
      assign p[i] = a[i] | b[i];
      assign c[i + 1] = g[i] | (p[i] & c[i]);
    end
  endgenerate
  
endmodule
`endif
`ifndef INT_FP_ADD_V_
`define INT_FP_ADD_V_


`timescale 1ns / 1ps

module int_fp_add (
`ifdef PIPELINE
  input         clk,
  input         rst_n,
`endif
  input         mode,
  input  [15:0] a,
  input  [15:0] b,
  output [15:0] c
);

  wire [10:0] adder_input_1,adder_input_2,aligned_small,adder_output;
  wire if_sub,a_sign, b_sign, c_sign,c1, c2;
  wire [15:0] normalized_out;

  // only used in INT8 MAC mode
  wire [4:0] higher_add,higher_a,higher_b;

  wire [15:0] result;
  reg [14:0] bigger, smaller;
  reg a_larger_b;

`ifdef PIPELINE
  reg [14:0] bigger_reg, smaller_reg;
  reg [10:0] adder_output_reg;
  wire [14:0] bigger_tmp, smaller_tmp;
  wire [10:0] adder_output_tmp;
`endif  


  assign a_sign        = a[15];
  assign b_sign        = b[15];
  assign if_sub        = (a_sign == b_sign) ? 1'b0 : 1'b1;
  assign c_sign        = a_larger_b ? a_sign : b_sign;
  assign higher_a      = (mode == 1'b0) ? a[15:11] : 5'b0;
  assign higher_b      = (mode == 1'b0) ? b[15:11] : 5'b0;
  assign adder_input_1 = (mode==1'b0) ? a[10:0] :{1'b1,bigger[9:0]};
  assign adder_input_2 = (mode==1'b0) ? b[10:0] : (if_sub ? ~aligned_small + 1'b1 : aligned_small);
  assign c             = (mode == 1'b0) ? {higher_add,adder_output} : result;

  //compare two number regardless sign
  always @(*) begin
    if (a[14:0] > b[14:0]) begin
      bigger = a[14:0];
      smaller = b[14:0];
      a_larger_b = 1'b1;
    end else begin 
      bigger = b[14:0];
      smaller = a[14:0];
      a_larger_b = 1'b0;
    end 
  end

`ifdef PIPELINE 
    always @ (posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        bigger_reg <= 15'b0;
        smaller_reg <= 15'b0;
        adder_output_reg <= 11'b0;
      end else begin
        bigger_reg <= bigger;
        smaller_reg <= smaller;
        adder_output_reg <= adder_output;
      end
    end
    assign bigger_tmp = bigger_reg[14:0];
    assign smaller_tmp = smaller_reg[14:0];
    assign adder_output_tmp = adder_output_reg[10:0];
`endif

`ifdef PIPELINE
  // align small number
  alignment u1(bigger_tmp,smaller_tmp,aligned_small);
`else 
  // align small number
  alignment u1(bigger,smaller,aligned_small);
`endif

  cla_nbit #(.n(11)) u2(adder_input_1,adder_input_2,1'b0,adder_output,c1);

  // This 5 bit adder only used in INT8 MAC mode
  cla_nbit #(.n(5)) u3(higher_a,higher_b,c1,higher_add,c2);

`ifdef PIPELINE
  add_normalizer u4(c_sign,bigger[14:10],adder_output_tmp,result,c1,if_sub);
`else 
  add_normalizer u4(c_sign,bigger[14:10],adder_output,result,c1,if_sub);
`endif

endmodule
`endif
`ifndef INT_FP_MUL_V_
`define INT_FP_MUL_V_


module int_fp_mul (
`ifdef PIPELINE
  input         clk,
  input         rst_n,
`endif
  input         mode,
  input  [15:0] a,
  input  [15:0] b,
  output [15:0] c,
  output        error // valid in fp16 mode 
);

  wire [15:0] c_tmp;
  wire        c_sign,a_zero,b_zero;
  wire [ 4:0] sum_exponent, biased_sum_exponent;
  wire [15:0] multiplier_input1,multiplier_input2;

  wire [31:0] multiplier_output;
  wire [14:0] normalized_out;
  wire [21:0] mantissa_prod;
  wire c1,c2,underflow,overflow;

  assign overflow = (c1 && c2 && ~biased_sum_exponent[4]) ? 1'b1 :1'b0;
  assign underflow = (~c1 && ~c2 && biased_sum_exponent[4]) ? 1'b1:1'b0;

  assign a_zero = ~(|a);
  assign b_zero = ~(|b);
  assign c_sign = a[15] ^ b[15];
  assign multiplier_input1 = mode ? {5'b0,1'b1,a[9:0]} : ((a[7]==1'b0) ? {9'b0,a[6:0]} : {9'b0,~a[6:0]+1'b1});
  assign multiplier_input2 = mode ? {5'b0,1'b1,b[9:0]} : ((b[7]==1'b0) ? {9'b0,b[6:0]} : {9'b0,~b[6:0]+1'b1});

  assign c = mode ? ((a_zero | b_zero) ? 16'b0 : c_tmp) : ((a[7]^b[7] == 1'b0) ? multiplier_output[15:0] : {1'b1,~multiplier_output[14:0]+1'b1});
  //error detect
  assign c_tmp = (~error) ? {c_sign,normalized_out} : (underflow ? {c_sign,15'b0000_0000_0000_000} : {c_sign,5'b1111_1,10'b0000_0000_00});

  assign error = overflow | underflow; 

    
`ifdef PIPELINE

  reg [31:0] multiplier_output_tmp;

  always @ (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      multiplier_output_tmp <= 32'b0;
    end else begin
      multiplier_output_tmp <= multiplier_output;
    end
  end

  assign mantissa_prod = multiplier_output_tmp[21:0];
  mul16x16 u1(clk,rst_n,multiplier_input1,multiplier_input2,multiplier_output);

`else 

  assign mantissa_prod = multiplier_output[21:0];
  mul16x16 u1(multiplier_input1,multiplier_input2,multiplier_output);

`endif
    
  cla_nbit #(.n(5)) u2(a[14:10],b[14:10],1'b0,sum_exponent,c1); // add exponent
  cla_nbit #(.n(5)) u3(sum_exponent, 5'b10001,1'b0,biased_sum_exponent,c2); // minus bias
  mul_normalizer u4(biased_sum_exponent,mantissa_prod,normalized_out);

endmodule
`endif
`ifndef MUL_NORMALIZER_V_
`define MUL_NORMALIZER_V_

`timescale 1ns / 1ps

module mul_normalizer (
  input  [ 4:0] exponent,
  input  [21:0] mantissa_prod,
  output [14:0] result
);

  wire [4:0] result_exponent;
  wire [9:0] result_mantissa;

  assign result_exponent = (mantissa_prod[21]) ? (exponent + 1'b1): exponent;
  assign result_mantissa = (mantissa_prod[21]) ? mantissa_prod[20:11]:mantissa_prod[19:10];
  assign result          = {result_exponent,result_mantissa};

// No rounding and No overflow/underflow detection

endmodule
`endif
// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [31:0] rdata; 
    wire [31:0] wdata;
    wire [BITS-1:0] count;

    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;

    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = rdata;
    assign wdata = wbs_dat_i;

    // IO
    assign io_out = count;
    assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

    // IRQ
    assign irq = 3'b000;	// Unused

    // LA
    //assign la_data_out = {{(127-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register  
    assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;

    wire cs;
    // Assuming LA probes [66] are for controlling cs (data ready)
    assign cs = (~la_oenb[66]) ? la_data_in[66] : 0;

    wire [31:0] bank2;
    wire done_o_net;
    assign bank2 = {{(24){1'b0}}, done_o_net, {(7){1'b0}}};
    assign la_data_out = {{(BITS){1'b0}}, bank2, {(BITS){1'b0}}, count};

    /*
    counter #(
        .BITS(BITS)
    ) counter(
        .clk(clk),
        .reset(rst),
        .ready(wbs_ack_o),
        .valid(valid),
        .rdata(rdata),
        .wdata(wbs_dat_i),
        .wstrb(wstrb),
        .la_write(la_write),
        .la_input(la_data_in[63:32]),
        .count(count)
    );
    */

    interface_top interface_inst(
      .clk(clk),
      .rst(rst),
      .cs(cs),
      .readout_addr(la_data_in[70:67]),
      .data_in(la_data_in[63:32]),
      .data_out(count),
      .done_o(done_o_net)
    );

endmodule

module counter #(
    parameter BITS = 32
)(
    input clk,
    input reset,
    input valid,
    input [3:0] wstrb,
    input [BITS-1:0] wdata,
    input [BITS-1:0] la_write,
    input [BITS-1:0] la_input,
    output ready,
    output [BITS-1:0] rdata,
    output [BITS-1:0] count
);
    reg ready;
    reg [BITS-1:0] count;
    reg [BITS-1:0] rdata;

    always @(posedge clk) begin
        if (reset) begin
            count <= 0;
            ready <= 0;
        end else begin
            ready <= 1'b0;
            if (~|la_write) begin
                count <= count + 1;
            end
            if (valid && !ready) begin
                ready <= 1'b1;
                rdata <= count;
                if (wstrb[0]) count[7:0]   <= wdata[7:0];
                if (wstrb[1]) count[15:8]  <= wdata[15:8];
                if (wstrb[2]) count[23:16] <= wdata[23:16];
                if (wstrb[3]) count[31:24] <= wdata[31:24];
            end else if (|la_write) begin
                count <= la_write & la_input;
            end
        end
    end

endmodule
`default_nettype wire

`default_nettype none

// Looks like we need our own version of transactional memory definition
// SPI alike
// 3 x 3, 3 x 3 -- 18 in
// 3 x 3 9 out
// GEMM

module interface_top (
  input  wire        clk,
  input  wire        rst,
  input  wire        cs,  // data ready
  input  wire [ 3:0] readout_addr,
  input  wire [31:0] data_in,
  output reg  [31:0] data_out,
  output wire        done_o
);

  localparam W = 16;
  localparam N = 3;

  //wire done_o;
  //assign data_out = 1;

  // scratch pad
  reg  [2 * W * N * N - 1 : 0] input_registers;
  wire [    W * N * N - 1 : 0] C_mat;

  // 16bit - 8
  // 32bit - 9
  // clog2 function
  reg  [7 : 0] addr_ptr;

  wire [W * N * N - 1 : 0] A_mat;
  wire [W * N * N - 1 : 0] B_mat;

  assign A_mat = input_registers[        0 +: W * N * N];
  assign B_mat = input_registers[W * N * N +: W * N * N];

  // mode selection
  // First try on FP16

  reg [2:0]      state;
  reg [2:0] next_state;

  wire [2:0] IDLE    = 3'b000;
  wire [2:0] LOAD    = 3'b001;
  wire [2:0] PROCESS = 3'b011;

  // Refactor: Moving them inside control
  wire [W * N - 1 : 0] A_in;
  wire [W * N - 1 : 0] B_in;

  control #(.W(W), .N(N)) control_inst(
    .i_clk(clk),
    .i_rst(rst),
    .i_en(mat_en),
    .i_mode(1'b1),
    .i_A(A_mat),
    .i_B(B_mat),
    .o_C(C_mat),
    .o_done(done_o)
  );

  // memory counter
  reg [4:0]      addr_cnter;
  reg [4:0] next_addr_cnter;

  // TODO: might be redundant
  reg mat_en;

  always @(posedge clk) begin
    if (rst) begin
      state           <= IDLE;
      next_state      <= IDLE;
      addr_cnter      <= 5'b0;
      next_addr_cnter <= 5'b0;
      data_out        <= 32'b0;
    end
    else begin
      state      <= next_state;
      addr_cnter <= next_addr_cnter;
    end
  end

  always @(*) begin
    case (state)
      IDLE: begin
	if (cs) begin
	  next_state = LOAD;
	end
	mat_en = 1'b0;
      end
      LOAD: begin
        input_registers[addr_cnter * W +: W] = data_in[W - 1 : 0];

	if (addr_cnter >= 2 * N * N - 1) begin
	  mat_en = 1'b1;
	  next_state = PROCESS;
	  next_addr_cnter = 0;
	end
	else begin
          next_addr_cnter = addr_cnter + 1;
	end
      end
      PROCESS: begin
	next_state = done_o ? IDLE : PROCESS;
      end
    endcase
  end

  // Readout addr
  always @(*) begin
    case (readout_addr)
      4'b0000: begin
        data_out[W - 1 : 0] = C_mat[4'b0000 * W +: W];
      end
      4'b0001: begin
        data_out[W - 1 : 0] = C_mat[4'b0001 * W +: W];
      end
      4'b0010: begin
        data_out[W - 1 : 0] = C_mat[4'b0010 * W +: W];
      end
      4'b0011: begin
        data_out[W - 1 : 0] = C_mat[4'b0011 * W +: W];
      end
      4'b0100: begin
        data_out[W - 1 : 0] = C_mat[4'b0100 * W +: W];
      end
      4'b0101: begin
        data_out[W - 1 : 0] = C_mat[4'b0101 * W +: W];
      end
      4'b0110: begin
        data_out[W - 1 : 0] = C_mat[4'b0110 * W +: W];
      end
      4'b0111: begin
        data_out[W - 1 : 0] = C_mat[4'b0111 * W +: W];
      end
      4'b1000: begin
        data_out[W - 1 : 0] = C_mat[4'b1000 * W +: W];
      end
    endcase
  end

endmodule

`default_nettype wire
