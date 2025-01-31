module rescale #(
    parameter W=16
)(
    input [2*W-1:0] din,
    output [W-1:0] dout,
    output reg [5:0] nbit_scaled
);

wire sign_bit = din[2*W-1];
genvar i;

wire [W:0] is_leading_sign;
assign is_leading_sign[0] = 1;

for (i=1; i<W; i=i+1)
begin
    assign is_leading_sign[i] = is_leading_sign[i-1] && (din[2*W-i] == din[2*W-i-1]);
end

always @(is_leading_sign)
begin
    int n;
    reg [5:0] sum;
    sum = 0;
    for (n=0; n<W; n=n+1)
        sum = sum + is_leading_sign[n];

    nbit_scaled = $unsigned(W+1) - sum;
end

wire [2*W-1:0] din_sft = din >> nbit_scaled;
assign dout = din_sft[W-1:0];

endmodule
