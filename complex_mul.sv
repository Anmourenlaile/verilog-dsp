module complex_mul #(
    parameter W = 24
) (
    input clk,
    input rst_n,
    input start,

    input [W-1:0] a_r,
    input [W-1:0] a_i,
    input [W:0] b_r,
    input [W:0] b_i,
    output [W-1:0] o_r,
    output [W-1:0] o_i,
    output [2*W-1:0] prod_r,
    output [2*W-1:0] prod_i
);
logic signed [W*2-0:0] ar_x_b = $signed(a_r) * $signed(start ? b_r : b_i);
logic signed [W*2-0:0] ai_x_b = $signed(a_i) * $signed(start ? b_r : b_i);

logic signed [W*2:0] p_r,p_i;

logic signed [W*2:0] ar_x_b_d,ai_x_b_d;
logic signed [W*2:0] p_r_d, p_i_d;

always_ff @( posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        p_r <= '0;
        p_i <= '0;
    end else if(start)
    begin
        p_r <= ar_x_b;
        p_i <= ai_x_b;
    end
end

always_ff @( posedge clk ore negedge rst_n ) begin 
    if(!rst_n)begin
        ar_x_b_d    <= '0;
        ai_x_b_d    <= '0;
        p_r_d       <= '0;
        p_i_d       <= '0;
    end 
    else begin
        ar_x_b_d    <=  ar_x_b;
        ai_x_b_d    <=  ai_x_b;
        p_r_d       <=  p_r;
        p_i_d       <=  p_i;
    end
    
end

logic signed [2*W:0] prod_r_sign;
logic signed [2*W:0] prod_i_sign;

assign prod_r_sign = p_r_d - ai_x_b_d;
assign prod_i_sign = p_i_d + ar_x_b_d;

logic   prod_r_sign_sat_flag = prod_r_sign[2*W] ^ prod_r_sign[2*W-1];

logic [2*W-1:0] prod_r_sign_sat;

always_comb begin
    case(prod_r_sign[2*W:2*W-1])
        2'b01:prod_r_sign_sat = {1'b0,{(47){1'b1}}};
        2'b10:prod_r_sign_sat = {1'b1, {(47){1'b0}}}; 
        default : prod_r_sign_sat = prod_r_sign[2*W-1: 0]; 
    endcase
end
// prod_i_sign 
wire prod_i_sign_sat_flag; 
assign prod_i_sign_aat_flag = prod_i_sign[2*W]^prod_i_sign[2*W-1]; 

reg [2*W-1:0] prod_i_sign_sat;
always @(*) begin
    case(prod_i_sign[2*W:2*W-1]) 
    2'b01 :prod_i_sign_sat = {1'b0, {(2+W-1) {1'b1}}} ; 
    2'b10 :prod_i_sign_sat ={1'b1,{(2+W-1){1'b0}}}; 
    default : prod_i_sign_sat = prod_i_sign[2*W-1: 0]; 
    endcase
end
assign prod_r = prod_r_sign_sat; 
assign prod_i = prod_i_sign_sat; 
// round
//bit24, bit23-bit0
//
wire signed [W: 0] prod_r_sign_extra; 
wire signed [W: 0] prod_i_sign_extra;
assign prod_r_sign_extra = prod_r[2*W-1: W-1]; 
assign prod_i_sign_extra = prod_i[2*W-1: W-1]; 

//o_r
wire prod_r_extra_sat_flag;
assign prod_r_extra_sat_flag =prod_r_sign_extra[W] ^ prod_r_sign_extra[W-1]; 

reg [W-1:0] prod_r_extra_sat;
always @(*) 
    begin
        case(prod_r_sign_extra[W:W-1]) 
        2'b01:prod_r_extra_sat = {1'b0, {(W-1){1'b1}}}; 
        2'b10:prod_r_extra_sat = {1'b1, {(W-1){1'b0}}}; 
        default:prod_r_extra_sat=prod_r_sign_extra[W-1:0]; 
        endcase
end

wire [W-1: 0] o_r_debug:
assign o_r_debug = (prod_r[2*W-2:0] + $unsigned(1<<(W-2))) >> (W-1); 
assign o_r = prod_r_extra_sat_flag ? prod_r_extra_sat : o_r_debug; 
