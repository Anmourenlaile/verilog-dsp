module signed_norm #(
    parameter W=24
)(
    input [W-1:0] a,
    output reg [W-1:0] o,
    output reg [5:0] quant_bit
);

reg [W-1:0] is_leading_sign;
genvar i;

assign is_leading_sign[W-1] = 1;

for (i=W-2; i>=0; i=i-1)
    assign is_leading_sign[i] = is_leading_sign[i+1] ? (a[i] == a[i+1]) : 1'b0;

reg [5:0] nbits_leading_sign;

always @(*)
begin
    int i;
    nbits_leading_sign = 0;
    for (i=W-1; i>0; i=i-1)
        nbits_leading_sign = nbits_leading_sign + is_leading_sign[i];

    if (a != 0)
    begin
        quant_bit = nbits_leading_sign - 1;
        o = a << quant_bit;
    end else
    begin
        quant_bit = 0;
        o = 0;
    end
end

endmodule
