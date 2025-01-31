`ifndef FFD
`define FFD 1
`endif

// Input data separate to three part: aaa_aaaa.bbbb_bbbb_cccc_cccc_cccc_ccc
//     n = a + b + c
// quantize to integer number:
//     n = A + B/(1<<8) + C/(1<<23)
// Here, A is 7-bit signed number, B is 8-bit unsigned number, C is 15-bit unsigned number
// result will be exp(a) * exp(b) * exp(c)

// First stage, use an LUT to get exp(a), range of A is from -38 to 21,
// If A out of this range, result will be overflow or underflow.
// in RTL, exp(a) result is intd/(1<<(W-1))*2^int_q
// use an LUT to get exp(b), in RTL, result is f0_d/(1<<(W-1))*2^f0_q

// Second stage, calculate exp(n) = exp(a) * exp(b);
//     calculate taylor expansion of exp(c) = 1 + c + c*c/2

// Third stage, calculate exp(n) = exp(n) * exp(c)
module exp #(
    parameter W=16
) (
    input clk,
    input rst_n,

    input start,
    output reg busy,
    input [W-1:0] in,
    input [5:0] in_quant_bits,
    output reg [W-1:0] out,
    output reg [5:0] out_quant_bits,
    output reg out_valid,

    output reg mul_en,
    output reg [W-1:0] mul_a,
    output reg [W-1:0] mul_b,
    input [2*W:0] mul_prod
);

localparam IDLE = 3'h0;
localparam EXP_MUL0_S1 = 3'h1;
localparam EXP_MUL0_S2 = 3'h2;
localparam EXP_MUL1_S1 = 3'h3;
localparam EXP_MUL1_S2 = 3'h4;

reg [2:0] state, state_next;
reg busy_next;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        state <= #`FFD IDLE;
        busy <= #`FFD 0;
    end else
    begin
        state <= #`FFD state_next;
        busy <= #`FFD busy_next;
    end


always @(*)
begin
    state_next = state;
    busy_next = busy;
    case (state)
        IDLE:
            if (start)
            begin
                state_next = EXP_MUL0_S1;
                busy_next = 1'b1;
            end
        EXP_MUL0_S1:
            state_next = EXP_MUL0_S2;
        EXP_MUL0_S2:
            state_next = EXP_MUL1_S1;
        EXP_MUL1_S1:
            state_next = EXP_MUL1_S2;
        EXP_MUL1_S2:
        begin
            state_next = IDLE;
            busy_next = 1'b0;
        end
    endcase
end

reg [W-1+96:0] in_shift;
reg [2*W:0] mul_prod_d;

always @(*)
    if (in_quant_bits[5])
        in_shift = {{64{in[W-1]}}, in[W-1:0], 32'h0} >> -in_quant_bits;
    else
        in_shift = {{64{in[W-1]}}, in[W-1:0], 32'h0} << in_quant_bits;

// --------------------------------------------------

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        mul_prod_d <= #`FFD {2*W{1'b0}};
    end else
    begin
        mul_prod_d <= #`FFD mul_prod;
    end


wire [W+8:0] in_shift_int = in_shift[W-1+64:W+31];
wire [7:0] in_shift_frac_0 = in_shift[W+30:W+23];
wire [14:0] in_shift_frac_1 = in_shift[W+22:W+8];    // Q23

wire overflow = $signed(in_shift_int) > 21;
wire underflow = $signed(in_shift_int) < -38;

reg [W-1:0] int_d;  // 0.5~1.0 Q23
reg signed [5:0] int_q;

reg [W-1:0] f0_d;
reg signed [5:0] f0_q;

reg [6:0] qbit_reg;
reg [W-1:0] out_reg;

//wire [2*W:0] mul_prod_1 = ((int_d[W-1] ? f0_d[W-1:0] : 0)<<(W-1)) + mul_prod;
wire [2*W:0] mul_prod_1 = ((int_d[W-1] ? f0_d[W-1:0] : 0)<<(W-1)) + mul_prod_d;


always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        mul_a <= #`FFD 0;
        mul_b <= #`FFD 0;
        mul_en <= #`FFD 0;
        out_reg <= #`FFD 0;
        qbit_reg <= #`FFD 0;
        out <= #`FFD 0;
        out_quant_bits <= #`FFD 0;
        out_valid <= #`FFD 0;
    end
    else if (start)
    begin
        mul_a <= #`FFD {1'b0, int_d[W-2:0]};
        mul_b <= #`FFD {1'b0, f0_d[W-1:0]};
        out_valid <= #`FFD 1'b1;
        mul_en <= #`FFD 1'b0;
    end
    else if (state == EXP_MUL0_S1)
    begin: exp_mul0_s1_blk
        mul_en <= #`FFD 0;
        out_valid <= #`FFD 0;
    end
    else if (state == EXP_MUL0_S2)
    begin: exp_mul0_s2_blk
        reg [W-1:0] out_reg_tmp;
        reg signed [6:0] qbit_reg_sign;

        out_valid <= #`FFD 0;

        qbit_reg_sign = int_q + f0_q;
        if (mul_prod_1[2*W-1:2*W-3] == 3'b000)
            qbit_reg_sign = qbit_reg_sign - 1;
        qbit_reg <= #`FFD $unsigned(qbit_reg_sign);

        out_reg_tmp = (mul_prod_1[2*W-1:2*W-3] == 3'b000) ? (mul_prod_1 >> (W-2)) : (mul_prod_1 >> (W-1));
        out_reg <= #`FFD out_reg_tmp[W-1:0];

        mul_a <= #`FFD in_shift_frac_1[14:0] + (in_shift_frac_1[14:7]*in_shift_frac_1[14:7] >> 10);
        mul_b <= #`FFD (1'b0, out_reg_tmp[W-1:0]);
        mul_en <= #`FFD 1'b1;
    end
    else if (state == EXP_MUL1_S1)
    begin
        out_valid <= #`FFD 0;
        mul_en <= #`FFD 0;
    end
    else if (state == EXP_MUL1_S2)
    begin
        reg [W-1:0] out_reg_tmp//, t;
        reg [6:0] qbit_reg_tmp;

        mul_en <= #`FFD 0;
        //t = (mul_prod_1 >> (W-1)); // Q(W-1+23) >> 23
        out_reg_tmp = out_reg + (mul_prod_d >> (W-1));
        qbit_reg_tmp = qbit_reg;
        if (out_reg_tmp[W-1])
        begin
            out_reg_tmp = out_reg_tmp >> 1;
            qbit_reg_tmp = qbit_reg_tmp + 1;
        end
        if (overflow || ($signed(qbit_reg_tmp) > 31) || ($signed(qbit_reg_tmp) == 31) && out_reg_tmp[W-1])
        begin
            out <= #`FFD (32'b1 <<(W-1)) - 1;
            out_quant_bit <= #`FFD 31;
    end else if (underflow)
begin
    out <= #`FFD 0;
    out_quant_bits <= #`FFD 0;
end else
begin
    if ($signed(qbit_reg_tmp) < -32)
    begin
        out <= #`FFD out_reg_tmp >> (-32 - $signed(qbit_reg_tmp));
        out_quant_bits <= #`FFD $unsigned(-32);
    end else
    begin
        out <= #`FFD out_reg_tmp;
        out_quant_bits <= #`FFD qbit_reg_tmp[5:0];
    end
end
    out_valid <= #`FFD 1;
end else
    out_valid <= #`FFD 0;
//need gpt to exect the code
always_comb begin
    int_d = 24'h1;
    int_q = -32;
end
always_comb begin
    f0_d = 24'h1;
    f0_q = 1;
end

//need gpt to exect the code
endmodule