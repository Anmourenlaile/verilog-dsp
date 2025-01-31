// *********************************************************************************
// Project Name : zkx2024
// Author       : Jlan
// Email        : 15533610762@163.com
// Create Time  : 2024-06-26
// File Name    : butterfly.sv
// Module Name  : butterfly
// Called By    : jlan
// Abstract     : butterfly
//
// 
// *********************************************************************************
// Modification History:
// Date         By              Version                 Change Description
// -----------------------------------------------------------------------
// 2024-06-26    Macro           1.0                     Original
//  
// *********************************************************************************

module butterfly#(
    parameter W = 24,
    parameter REG = 1
) (
    input clk,
    input rst_n,
    input start,

    input [W-1:0] a_r,
    input [W-1:0] a_i,
    input [W-1:0] b_r,
    input [W-1:0] b_i,

    input [W:0] w_r, //signed
    input [W:0] w_i,
    output logic [W-1:0] p_r,
    output logic [W-1:0] p_i,
    output logic [W-1:0] q_r, 
    output logic [W-1:0] q_i,

    output quant_full,

    input complex_mul_en,
    input complex_mul_start,

    input [W-1:0] complex_mul_a_r,
    input [W-1:0] complex_mul_a_i,
    input [W-1:0] complex_mul_b_r,
    input [W-1:0] complex_mul_b_i,

    output [2*W-1:0] complex_mul_o_r,
    output [2*W-1:0] complex_mul_o_i
);
//*************************************************************************************//
//butterfly: all operation is complex calculation
//          p=a+b
//          q=(a-b)*w
//*************************************************************************************//

logic [W:0] diff_r_tmp, diff_i_tmp;
logic [W:0] diff_r , diff_i;
logic p_r_msb;
logic p_i_msb;
logic [W-1:0] p_r_tmp,p_i_tmp;
logic [W-1:0] p_r_tmp2,p_i_tmp2;

logic add_almost_overflow,add_almost_underflow;
logic sub_almost_overflow,sub_almost_underflow;

complex_add #(
    .W(W+1)
) add(
    .a_r({a_r[W-1],a_r[W-1:0]}),
    .a_r({a_i[W-1],a_i[W-1:0]}),
    .b_r(b_r[W-1],b_r[W-1:0]),
    .b_r(b_i[W-1],b_i[W-1:0]),
    .o_r({p_r_msb,p_r_tmp[W-1:0]}),
    .o_r({p_i_msb,p_i_tmp[W-1:0]})
);

logic [2:0] add_msb_r = {p_r_msb,p_r_tmp[W-1:W-2]};
logic [2:0] add_msb_i = {p_i_msb,p_i_tmp[W-1:W-2]};

logic add_almost_overflow_tmp = (add_msb_r = 3'b001) || (add_msb_r = 3'b010) || (add_msb_r = 3'b011) ||
                                (add_msb_i = 3'b001) || (add_msb_i = 3'b010) || (add_msb_i = 3'b011);
logic add_almost_underflow_tmp = (add_msb_r = 3'b110) || (add_msb_r = 3'b101) || (add_msb_r = 3'b100) ||
                                (add_msb_i = 3'b110) || (add_msb_i = 3'b101) || (add_msb_i = 3'b100);

assign  p_r_tmp2 = (add_msb_r == 3'b010) || (add_msb_r  == 3'b011) ? {1'b0,{(W-1){1'b1}}} :
                (add_msb_r == 3'b101) || (add_msb_r = 3'b110) ? {1'b1,{(W-1){1'b0}}} :
                p_r_tmp;
assign  p_i_tmp2 = (add_msb_i == 3'b010) || (add_msb_i  == 3'b011) ? {1'b0,{(W-1){1'b1}}} :
                (add_msb_i == 3'b101) || (add_msb_i = 3'b110) ? {1'b1,{(W-1){1'b0}}} :
                p_i_tmp;

complex_sub #(
    .W(W+1)
) sub(
    .a_r({a_r[W-1],a_r[W-1:0]}),
    .a_r({a_i[W-1],a_i[W-1:0]}),
    .b_r(b_r[W-1],b_r[W-1:0]),
    .b_r(b_i[W-1],b_i[W-1:0]),
    .o_r(diff_r_tmp[W:0]),
    .o_i(diff_i_tmp[W:0])
);

logic sub_almost_overflow_tmp = (diff_r_tmp[W:W-2] == 3'b001) || (diff_r_tmp[W:W-2] == 3'b010) || (diff_r_tmp[W:W-2] == 3'b011) ||
                                (diff_i_tmp[W:W-2] == 3'b001) || (diff_i_tmp[W:W-2] == 3'b010) || (diff_i_tmp[W:W-2] == 3'b011);
logic sub_almost_underflow_tmp = (diff_r_tmp[W:W-2] == 3'b110) || (diff_r_tmp[W:W-2] == 3'b101) || (diff_r_tmp[W:W-2] == 3'b100) ||
                                (diff_i_tmp[W:W-2] == 3'b110) || (diff_i_tmp[W:W-2] == 3'b101) || (diff_i_tmp[W:W-2] == 3'b100);

if(REG)begin
    always_ff @( posedge clk or negedge rst_n ) begin 
        if(!rst_n)begin
            p_r <= {W{1'b0}};
            p_i <= {W{1'b0}};
            add_almost_overflow  <= 1'b0;
            add_almost_underflow <= 1'b0;
            sub_almost_overflow  <= 1'b0;
            sub_almost_underflow <= 1'b0;
        end

        else begin
            p_r <= p_r_tmp2;
            p_i <= p_i_tmp2;
            add_almost_overflow  <= add_almost_overflow_tmp;
            add_almost_underflow <= add_almost_underflow_tmp;
            sub_almost_overflow  <= sub_almost_overflow_tmp;
            sub_almost_underflow <= sub_almost_underflow_tmp;
        end
    end
end else 
begin
    always_comb begin
        p_r = p_r_tmp2;
        p_i = p_i_tmp2;
        add_almost_overflow  = add_almost_overflow_tmp;
        add_almost_underflow = add_almost_underflow_tmp;
        sub_almost_overflow  = sub_almost_overflow_tmp;
        sub_almost_underflow = sub_almost_underflow_tmp;
    end
end
//-----------------------------------------------------------------------------------------------------------------------
assign diff_r = (diff_r_tmp[W:W-2] == 3'b010) || (diff_r_tmp[W:W-2] == 3'b011) ? {1'b0,{(W-1){1'b1}}} :
    (diff_r_tmp[W:W-2] == 3'b101) || (diff_r_tmp[W:W-2] == 3'b100) ? {1'b1,{(W-1){1'b0}}}:
    diff_r_tmp;
assign diff_i = (diff_i_tmp[W:W-2] == 3'b010) || (diff_i_tmp[W:W-2] == 3'b011) ? {1'b0,{(W-1){1'b1}}} :
    (diff_i_tmp[W:W-2] == 3'b101) || (diff_i_tmp[W:W-2] == 3'b100) ? {1'b1,{(W-1){1'b0}}}:
    diff_i_tmp;

if(REG)begin
    logic start_d;
    logic [W-1:0] diff_r_d;
    logic [W-1:0] diff_i_d;
    logic [W:0] w_r_d;
    logic [W:0] w_i_d;

    always_ff@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            start_d  <= 1'b0;
            diff_r_d <= '0;
            diff_i_d <= '0;
            w_r_d <= '0;
            w_i_d <= '0;
        end
        else begin
            start_d <= start;
            diff_r_d    <= diff_r;
            diff_i_d    <= diff_i;
            w_r_d    <= w_r;
            w_i_d    <= w_i;
        end
    end

    complex_mul #(
        .W(W)
    ) mul(
        .clk(clk),
        .rst_n(rst_n),
        .start(complex_mul_en ? complex_mul_start : start_d),


        .a_r(complex_mul_en ? complex_mul_a_r : diff_r_d[W-1:0]),
        .a_i(complex_mul_en ? complex_mul_a_i : diff_i_d[W-1:0]),
        .b_r(complex_mul_en ? {complex_mul_b_r[W-1],complex_mul_b_r[W-1:0]} : w_r_d[W:0]),
        .b_i(complex_mul_en ? {complex_mul_b_i[W-1],complex_mul_b_i[W-1:0]} : w_i_d[W:0]),
        .o_r(q_r[W-1:0]),
        .o_i(q_i[W-1:0]),
        .prod_r(complex_mul_o_r[2*W-1:0]),
        .prod_i(complex_mul_o_i[2*W-1:0])
    );
end else
begin

    complex_mul #(
        .W(W)
    ) mul(
        .clk(clk),
        .rst_n(rst_n),
        .start(complex_mul_en ? complex_mul_start : start_d),


        .a_r(complex_mul_en ? complex_mul_a_r : diff_r_d[W-1:0]),
        .a_i(complex_mul_en ? complex_mul_a_i : diff_i_d[W-1:0]),
        .b_r(complex_mul_en ? {complex_mul_b_r[W-1],complex_mul_b_r[W-1:0]} : w_r_d[W:0]),
        .b_i(complex_mul_en ? {complex_mul_b_i[W-1],complex_mul_b_i[W-1:0]} : w_i_d[W:0]),
        .o_r(q_r[W-1:0]),
        .o_i(q_i[W-1:0]),
        .prod_r(complex_mul_o_r[2*W-1:0]),
        .prod_i(complex_mul_o_i[2*W-1:0])
    );
end

logic mul_almost_overflow = (q_r[W-1:W-3] == 3'b001) || (q_r[W-1:W-3] == 3'b010) ||(q_r[W-1:W-3] == 3'b011) || 
                            (q_i[W-1:W-3] == 3'b001) || (q_i[W-1:W-3] == 3'b010) ||(q_i[W-1:W-3] == 3'b011);
logic mul_almost_underflow = (q_r[W-1:W-3] == 3'b110) || (q_r[W-1:W-3] == 3'b101) ||(q_r[W-1:W-3] == 3'b100) || 
                             (q_i[W-1:W-3] == 3'b110) || (q_i[W-1:W-3] == 3'b101) ||(q_i[W-1:W-3] == 3'b100);

assign quant_full = add_almost_overflow || add_almost_underflow || sub_almost_overflow || sub_almost_underflow || mul_almost_overflow || mul_almost_underflow;
endmodule