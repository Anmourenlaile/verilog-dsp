module complex_abs #(
    parameter W = 16,
    parameter REG = 0
)(
    input clk,
    input rst_n,

    input [1:0] op,  // 0: complex_abs; 1: o=sqrt(a); 2: o = a*a+b*b
    input [W-1:0] a_r,
    input [W-1:0] a_i,
    input [5:0] a_quant,
    output [2*W-1:0] square_diff_out,
    output [5:0] square_diff_in_qbits_d2,  // Square difference input quant bits delay for 2 cycles if REG == 1
    output [W-1:0] o,
    output [5:0] o_quant_bit
);

// synopsys translate_off
assign (weak0, weak1) op = 0;
// synopsys translate_on

wire [2*W-1:0] a_r_sqr;
wire [2*W-1:0] a_i_sqr;

DW_square #(
    .width(W)
) sqr_r(
    .a(a_r),
    .tc(1'b1),
    .square(a_r_sqr)
);

DW_square #(
    .width(W)
) sqr_i(
    .a(a_i),
    .tc(1'b1),
    .square(a_i_sqr)
);

reg [2*W-1:0] a_sqr_sum;

if (REG)
begin

    reg [7*6-1:0] a_quant_d;
    reg [2*W-1:0] a_r_sqr_d;
    reg [2*W-1:0] a_i_sqr_d;

    always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        a_r_sqr_d <= 0;
        a_i_sqr_d <= 0;
    end
    else
    begin
        a_r_sqr_d <= a_r_sqr;
        a_i_sqr_d <= a_i_sqr;
    end
always @(posedge clk or negedge rst_n)
if (!rst_n)
begin
    a_sqr_sum <= 0;
    a_quant_d <= 0;
end 
else 
begin
    a_sqr_sum <= (op == 1) ? {1'b0, a_r[W-1:0], {(W-1){1'b0}}} :
                 ((op == 0) ? (a_r_sqr_d + a_i_sqr_d) : (a_r_sqr_d - a_i_sqr_d) );
    a_quant_d <= {a_quant_d[6*6-1:0], a_quant[5:0]};
end

DW_sqrt_pipe #(
    .width(2*W),
    .num_stages(6),
    .tc_mode(0)
) sqrt (
    .clk(clk),
    .rst_n(rst_n),
    .en(1'b1),
    .a(a_sqr_sum),
    .root(o)
);

assign o_quant_bit = (op == 1) ? a_quant_d[35:30] : a_quant_d[41:36];

assign square_diff_out = a_sqr_sum;
assign square_diff_in_qbits_d2 = a_quant_d[11:6];
end
else 
begin
    always @(*)
    a_sqr_sum = (op == 1) ? {1'b0, a_r[W-1:0], {(W-1){1'b0}}} :
                ((op == 0) ? (a_r_sqr + a_i_sqr) : (a_r_sqr - a_i_sqr) );

    DW_sqrt #(
        .width(2*W),
        .tc_mode(0)
    ) sqrt (
        .a(a_sqr_sum),
        .root(o)
    );

    assign o_quant_bit = a_quant;
    assign square_diff_out = a_sqr_sum;
    assign square_diff_in_qbits_d = a_quant;
end

endmodule
