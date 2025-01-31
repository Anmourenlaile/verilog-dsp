module complex_sub #(
    parameter W =24
) (
    input  [W-1:0] a_r, 
    input  [W-1:0] a_i, 
    input  [W-1:0] b_r, 
    input  [W-1:0] b_i,

    output [W-1:0] o_r,
    output [W-1:0] o_i 
);

assign o_r = a_r - b_r;
assign o_i = a_i - b_i;
endmodule