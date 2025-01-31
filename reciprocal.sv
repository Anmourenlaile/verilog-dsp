module reciprocal #(
    parameter REG=0
)(
    input clk,
    input rst_n,

    input [23:0] a,  // signed, Q23
    input [6:0] a_quant_bit,  // signed, -64 ~ 63
    output [23:0] o,  // (1, 2] or (-2, -1], Q21
    output reg [6:0] quant_bit,  // signed, -64 ~ 63
    output reg zero_in
);

reg signed [10:0] num_0;  // signed, Q8
wire [9:0] a2 = a[23:14];  // signed, Q8

wire signed [32:0] den_prod_0_sign = $signed(a) * num_0;  // unsigned, Q31
wire [32:0] den_prod_0 = $unsigned(den_prod_0_sign);  // unsigned, Q31

wire [25:0] den_0 = den_prod_0[32:7];  // unsigned, Q24

// 2022/01/06 Jianlin Liang: Add pipeline
reg signed [10:0] num_0_d;
reg [25:0] den_0_d;

if (REG)
begin
    always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        num_0_d <= #`FFD 11'h0;
        den_0_d <= #`FFD 26'h0;
    end else
    begin
        num_0_d <= #`FFD num_0;
        den_0_d <= #`FFD den_0;
    end
end 
else
begin
    always @(*)
    begin
        num_0_d = num_0;
        den_0_d = den_0;
    end
end
wire [10:0] m = den_0_d[18:8]; // signed, Q16

wire signed [21:0] num_prod_1_sign = num_0_d * $signed(m); // signed, Q24

wire [21:0] num_prod_1 = $unsigned(num_prod_1_sign); // unsigned, Q24

wire [26:0] num_1_tmp = {num_0_d, 16'h0} - {{5{num_prod_1[21]}}, num_prod_1[21:0]}; // signed, Q24

wire signed [35:0] den_prod_1_sign = $signed(den_0_d) * $signed(m); // signed, Q40

wire [35:0] den_prod_1 = $unsigned(den_prod_1_sign); // unsigned, Q40

wire [25:0] den_1_tmp = den_0_d - {{6{den_prod_1[35]}}, den_prod_1[35:16]}; // unsigned, Q24

reg [26:0] num_1;
reg [25:0] den_1;

reg [6:0] quant_bit_tmp1, quant_bit_tmp2;
if (REG)
begin
    always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        num_1 <= #`FFD 27'h0;
        den_1 <= #`FFD 26'h0;
        quant_bit <= #`FFD 7'h0;
        quant_bit_tmp1 <= #`FFD 7'h0;
        quant_bit_tmp2 <= #`FFD 7'h0;
        zero_in <= #`FFD 0;
    end 
    else
    begin
        num_1 <= #`FFD num_1_tmp;
        den_1 <= #`FFD den_1_tmp;
        quant_bit_tmp1 <= #`FFD -a_quant_bit + 2;
        quant_bit_tmp2 <= #`FFD quant_bit_tmp1;
        quant_bit <= #`FFD quant_bit_tmp2;
        zero_in <= #`FFD (a == 0);
    end
end 
else
begin
    always @(*)
    begin
        num_1 = num_1_tmp;
        den_1 = den_1_tmp;
        quant_bit = -a_quant_bit + 2;
        zero_in = (a == 0);
    end
end

wire [8:0] m1 = den_1[8:0]; // signed, Q24
wire signed [34:0] num_prod_2_sign = $signed(num_1[26:0]) * $signed(m1); // signed, Q48
wire [34:0] num_prod_2 = $unsigned(num_prod_2_sign); // unsigned, Q48
// 2022/01/06 Jianlin Liang: Add pipeline

reg [26:0] num_1_d;
reg [34:0] num_prod_2_d;
reg [25:0] den_1_d;
wire [8:0] m1_d = den_1_d[8:0]; // signed, Q24

if (REG)
begin
    always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        num_1_d <= #`FFD 27'h0;
        num_prod_2_d <= #`FFD 35'h0;
        den_1_d <= #`FFD 26'h0;
    end 
    else
    begin
        num_1_d <= #`FFD num_1;
        num_prod_2_d <= #`FFD num_prod_2;
        den_1_d <= #`FFD den_1;
    end
end else
begin
    always @(*)
    begin
        num_1_d = num_1;
        num_prod_2_d = num_prod_2;
        den_1_d = den_1;
    end
end

//--------------------------------------------------------

reg [26:0] num_2;
always @(*)
    num_2 = num_1_d[26:0] - {{16{num_prod_2_d[34]}}, num_prod_2_d[34:24]}; // signed, Q24

wire signed [33:0] den_prod_2_sign = $signed(den_1_d) * $signed(m1_d); // signed, Q48
wire [33:0] den_prod_2 = $unsigned(den_prod_2_sign); // unsigned, Q48

// synopsys translate_off
reg [25:0] den_2;
always @(*)
    den_2 = den_1 - {{16{den_prod_2[33]}}, den_prod_2[33:24]}; // unsigned, Q24
// synopsys translate_on

assign o = num_2[26:3];

//need gpt
always_comb begin
    num_0 = 512;
end
//need gpt
endmodule
