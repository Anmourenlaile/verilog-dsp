`ifndef FFD
`define FFD 1
`endif

// Used to calculate linear interpolation

// Assume y1, y2, k_pos. k_pos is the relative location where the interpolation value to be calculated.
// Here k_pos is a value between [0,1) represented by a 24-bit fix point value.
// The sign bit (highest bit, which is bit 23) of k_pos should be fixed at 0.
// In real use, bit 23 of k_pos is ignored and a 0 will be put at that bit before mul.
// The interpolated value y is y = y1 + k_pos * (y2 - y1).
// For a complex number, real part and image part will perform the same calculation above and results sent out separately.

module linear_interpolation #(
    parameter W=24
)(
    input clk,
    input rst_n,

    input start_v_intp,
    input [W-1:0] k_pos,

    input op_from_fifo_vld,
    input is_real,
    input [W-1:0] op_from_fifo,

    output result_vld,
    output [W-1:0] result,

    output mul_en,
    output [W-1:0] mul_a,
    output reg [W:0] mul_b,
    input [2*W:0] mul_prod
);

reg real_init_done, image_init_done;
wire set_real, set_image;

wire real_cal_start, image_cal_start;
reg [1:0] real_cal_start_d, image_cal_start_d;

reg [W-1:0] in1_cur_real, in1_cur_image, in1_next_real, in1_next_image;
wire update_in1_cur_real, update_in1_cur_image;

wire signed [W-1:0] sub_op_1;
wire signed [W-1:0] sub_op_2;
wire signed [W:0] sub_result;

wire signed [W-1:0] add_op_1;
wire signed [W-1:0] add_op_2;
wire signed [W:0] add_result;

//---------------------------------------------

assign set_real = op_from_fifo_vld && is_real;
assign set_image = op_from_fifo_vld && (!is_real);

assign real_cal_start = set_real && real_init_done;
assign image_cal_start = set_image && image_init_done;


always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        real_cal_start_d <= #`FFD 2'b0;
        image_cal_start_d <= #`FFD 2'b0;
    end else
    begin
        real_cal_start_d <= #`FFD {real_cal_start_d[0], real_cal_start};
        image_cal_start_d <= #`FFD {image_cal_start_d[0], image_cal_start};
    end

assign update_in1_cur_real = real_cal_start_d[1];
assign update_in1_cur_image = image_cal_start_d[1];

//-----------------------------------------------------

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        real_init_done <= #`FFD 1'b0;
    end else if (start_v_intp)
    begin
        real_init_done <= #`FFD 1'b0;
    end else if (set_real)
    begin
        real_init_done <= #`FFD 1'b1;
    end

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        image_init_done <= #`FFD 1'b0;
    end else if (start_v_intp)
    begin
        image_init_done <= #`FFD 1'b0;
    end else if (set_image)
    begin
        image_init_done <= #`FFD 1'b1;
    end
// Save the inputs
//-----------------------------------------------------
always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        in1_cur_real <= #`FFD {W{1'b0}};
    end else if (set_real && (!real_init_done))
    begin
        in1_cur_real <= #`FFD op_from_fifo;
    end else if (update_in1_cur_real)
    begin
        in1_cur_real <= #`FFD in1_next_real;
    end

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        in1_cur_image <= #`FFD {W{1'b0}};
    end else if (set_image && (!image_init_done))
    begin
        in1_cur_image <= #`FFD op_from_fifo;
    end else if (update_in1_cur_image)
    begin
        in1_cur_image <= #`FFD in1_next_image;
    end

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        in1_next_real <= #`FFD {W{1'b0}};
    end else if (real_cal_start)
    begin
        in1_next_real <= #`FFD op_from_fifo;
    end

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        in1_next_image <= #`FFD {W{1'b0}};
    end else if (image_cal_start)
    begin
        in1_next_image <= #`FFD op_from_fifo;
    end

// Stage 1: sub
//-----------------------------------------------------
assign sub_op_1[W-1:0] = real_cal_start ? in1_cur_real[W-1:0] : in1_cur_image[W-1:0];
assign sub_op_2 = op_from_fifo[W-1:0];

assign sub_result = sub_op_2 - sub_op_1;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        mul_b <= #`FFD {(W+1){1'b0}};
    end else
    begin
        mul_b <= #`FFD sub_result;
    end

// k_pos is between [0,1], thus highest bit(the sign bit) is always set to 0 when used.
assign mul_a = {1'b0, k_pos[W-2:0]};

//-----------------------------------------------------
// Stage 2: mul
//-----------------------------------------------------
assign mul_en = real_cal_start_d[0] | image_cal_start_d[0];

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        add_op_2 <= #`FFD {(W+1){1'b0}};
    end else
    begin
        add_op_2 <= #`FFD mul_prod[2*W-1: W-1];
    end

//-----------------------------------------------------
// Stage 3: add and output
//-----------------------------------------------------
assign add_op_1[W-1:0] = real_cal_start_d[1] ? in1_cur_real[W-1:0] : in1_cur_image[W-1:0];
assign add_result = add_op_1 + add_op_2;

// Interpolation result is always between y1 and y2, thus only lower W bits of add_result are enough.
assign result = add_result[W-1:0];

assign result_vld = real_cal_start_d[1] | image_cal_start_d[1];

endmodule
