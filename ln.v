module ln #(
    parameter W = 16,
    parameter LOG_PREC = 10,
    parameter OUT_QUANT = 12,
    parameter REG = 1
)(
    input clk,
    input rst_n,
    input [W-1:0] a,
    input signed [7:0] a_quant,
    output [W-1:0] z
);

wire [W-1:0] norm_out;
wire signed [7:0] norm_quant_out;
wire norm_no_detect;
reg norm_no_detect_reg, norm_no_detect_reg_2;

DW_norm #(
    .a_width(W),
    .srch_wind(W),
    .exp_width(8),
    .exp_ctr(1)
) norm (
    .a(a[W-1:0]),
    .exp_offset(a_quant),
    .no_detect(norm_no_detect),
    .ovfl(),
    .b(norm_out[W-1:0]),
    .exp_adj(norm_quant_out)
);

reg [W-1:0] norm_out_reg;
reg signed [7:0] norm_quant_out_reg, norm_quant_out_reg_2;

if (REG)
begin
    always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        norm_out_reg <= {W{1'b0}};
        norm_quant_out_reg <= 8'h0;
        norm_quant_out_reg_2 <= 8'h0;
        norm_no_detect_reg <= 1'b0;
        norm_no_detect_reg_2 <= 1'b0;
    end else
    begin
        norm_out_reg <= norm_out;
        norm_quant_out_reg <= norm_quant_out;
        norm_quant_out_reg_2 <= norm_quant_out_reg;
        norm_no_detect_reg <= norm_no_detect;
        norm_no_detect_reg_2 <= norm_no_detect_reg;
    end
end
begin

    always @(*)
    begin
        norm_out_reg = norm_out;
        norm_quant_out_reg = norm_quant_out;
        norm_quant_out_reg_2 = norm_quant_out_reg;
        norm_no_detect_reg = norm_no_detect;
        norm_no_detect_reg_2 = norm_no_detect;
    end

end

wire [LOG_PREC-1:0] ln_out;
reg [LOG_PREC-1:0] ln_out_d;

DW_ln #(
    .op_width(LOG_PREC),
    .arch(0),
    .err_range(1)
) ln (
    .a(norm_out_reg[W-1:W-LOG_PREC]),
    .z(ln_out[LOG_PREC-1:0])
);

// Pipeline ln output to speed up
if (REG)
begin
    always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        ln_out_d <= {LOG_PREC{1'b0}};
    end
    else begin
        ln_out_d <= ln_out;
    end
end 
else
begin
    always @(*)
        ln_out_d = ln_out;
end

wire [W-1:0] _ln_2 = 24'd11629080 >> (24-OUT_QUANT); // 0.69314718 * $unsigned(2**OUT_QUANT)

wire signed [6:0] tmp = norm_quant_out_reg_2 + W - 1;

wire signed [W:0] z_sign;

assign z_sign = $signed({{(W-LOG_PREC){1'b0}}, ln_out_d} << (OUT_QUANT-LOG_PREC)) + tmp * $signed(_ln_2);

assign z = norm_no_detect_reg_2 ? 24'h800000 : (z_sign[W:W-1] == 2'b10 ? 24'h800000 : $unsigned(z_sign[W-1:0]));

endmodule

