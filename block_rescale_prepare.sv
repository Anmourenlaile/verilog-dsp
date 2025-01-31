module block_rescale_prepare #(
    parameter W=16
)
(
    input clk,
    input rst_n,

    input stage_0_start,
    input stage_0_enable,
    input set_inf,
    input [W-1:0] in,
    input [6:0] in_quant_bit,
    output reg [5:0] quant_bits_max,
    output reg [W-1:0] out_unaligned,  // Used by Dot product intermediate stage
    output reg [5:0] out_quant_bit_unaligned,  // Used by Dot product intermediate stage
    output reg [W-1:0] out,
    output reg [5:0] out_quant_bit
);

/// inf, overflow and underflow handling
reg [W-1:0] result_0;
reg [5:0] result_quant_bit_0;
always @(*)
begin : inf_ov_ud_handling_block
    reg overflow, underflow;
    reg [W-1:0] tmp_result;
    reg [6:0] tmp_result_quant_bit;

    overflow = ($signed(in_quant_bit) > 31);
    underflow = ($signed(in_quant_bit) < -55);

    if (set_inf)
    begin
        tmp_result_quant_bit = 6'd31;
        tmp_result = $unsigned((1<<(W-1))-1);
    end 
    else if (overflow)
    begin
        tmp_result_quant_bit = 6'd31;
        tmp_result = $unsigned(in[W-1] ? (1<<(W-1)) : (1<<(W-1))-1);
    end 
    else if (underflow)
    begin
        tmp_result_quant_bit = -6'd32;
        tmp_result = in[W-1] ? {W{1'b1}} : 1;
    end 
    else
    begin
        tmp_result_quant_bit = in_quant_bit;
        tmp_result = in;
    end

    if ($signed(tmp_result_quant_bit) < -32)
    begin
        result_0 = $signed(tmp_result) >>> (-32 - $signed(tmp_result_quant_bit));
        result_quant_bit_0 = -32;
    end 
    else
    begin
        result_0 = tmp_result;
        result_quant_bit_0 = result_0 != 0 ? tmp_result_quant_bit[5:0] : -32;
    end
end

reg [9:0] rescale_cnt;
always @(posedge clk or negedge rst_n)
if (!rst_n)
begin
    rescale_cnt <= #`FFD 10'b0;
    quant_bits_max <= #`FFD 6'h0;
end 
else if (stage_0_start)
begin
    rescale_cnt <= #`FFD 10'b0;
    quant_bits_max <= #`FFD 6'h0;
end 
else if (stage_0_enable)
begin
    if (rescale_cnt == 0)
    begin
        if (!set_inf)
            quant_bits_max <= #`FFD result_quant_bit_0;
        else
            quant_bits_max <= #`FFD $unsigned(-32);
    end 
    else if (($signed(result_quant_bit_0) > $signed(quant_bits_max)) && !set_inf)
    begin
        quant_bits_max <= #`FFD result_quant_bit_0[5:0];
    end
    
    rescale_cnt <= #`FFD rescale_cnt + 10'h1;
end

always @(*)
begin
    if ((rescale_cnt != 0) && ($signed(result_quant_bit_0) < $signed(quant_bits_max)))
    begin
        reg [W+31:0] t;
        t = {{32{result_0[W-1]}}, result_0[W-1:0]} >> ($signed(quant_bits_max) - $signed(result_quant_bit_0));
        out = t[W-1:0];
        out_quant_bit = quant_bits_max;
    end 
    else
    begin
        out = result_0[W-1:0];
        out_quant_bit = result_quant_bit_0;
    end
end

// Used by Dot product intermediate steps, not aligned based on quant_bits_max
assign out_unaligned = result_0[W-1:0];
assign out_quant_bit_unaligned = result_quant_bit_0;

endmodule  // block_rescale_prepare
