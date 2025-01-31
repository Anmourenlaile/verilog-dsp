`ifndef FFD
`define FFD 1
`endif

module vector_core #(
    parameter W = 16,      // width of buf_a / buf_b data with exp, which is W + 6 (6 is for exponent part)
    parameter W_BUF = 22,   // length of buffer
    parameter LOG_PREC = 12,
    parameter LN_INT_QUANT = 12,
    parameter REG = 0
)(
    input clk,
    input rst_n,
    output reg busy,
    input in_buf_sel_mode,   // 0: use previous result; 1: set by in_buf_sel
    input prev_in_buf_sel,  // 0: from buffer A; 1: from buffer B
    input [5:0] prev_in_quant_bits,
    input in_buf_sel,
    input [5:0] in_quant_bits,

    input [4:0] op, // 0: hamming window;
                     // 2: complex abs
                     // 3: real abs, complex abs + real map + ln
                     // 0/10: set op2
    input [9:0] tab_offset,
                     // op1or0: window offset in tab memory
                     // op3: mel map coeff offset in tab memory
    input tab_type,  // 0: table is even symmetric like (1 2 3 3 2 1 0);
                     // 1: table is odd symmetric like (1 2 3 4 3 2 1);

    input [12:0] pre_emp_k, // k is Q13
    input [W-1:0] han_in_prev_sample, // 1: last sample of previous hamming window operation
    input use_prev_sample, // 0: set previous sample as han_win_prev_sample[w-1:0]; 1: use last sample of previous hamming window op
    input [3:0] ham_win_prec_keep,
    input [3:0] mfcc_out_prec,   
    input [8:0] vector_len, // vector_length-1
    input [8:0] in_vector_start,
    input [8:0] out_vector_start,

    input scalar_op 
    input [W-1:0]scalar_dat,
    input [5:0] scalar_qbit,
    input complex_op,
    input [W-1:0] complex_image_dat,  // This is also used as v_intp_k_pos (the position where to interpolate) in linear interpolation

    output [9:0] tab_addr,
    output tab_en,
    input  [2*W-1:0] tab_rdata,

    output [8:0] buf_addr_a,
    output buf_en_a,
    output buf_we_a,
    output [1:0] buf_wbe_a,
    output [2*W_BUF-1:0] buf_wdata_a_full,
    input  [2*W_BUF-1:0] buf_rdata_a_full,
    
    output [8:0] buf_addr_b,
    output buf_en_b,
    output buf_we_b,
    output [1:0] buf_wbe_b,
    output [2*W_BUF-1:0] buf_wdata_b_full,
    input  [2*W_BUF-1:0] buf_rdata_b_full,


    output reg [5:0] out_quant_bits,
    output result_in,  // 0: in buffer A; 1: in buffer B
    
    output complex_mul_en,
    output complex_mul_start,

    output [W-1:0] complex_mul_a_r,
    output [W-1:0] complex_mul_a_i,
    output [W-1:0] complex_mul_b_r,
    output [W-1:0] complex_mul_b_i,
    input  [2*W-1:0] complex_mul_o_r,
    input  [2*W-1:0] complex_mul_o_i,

    input reg_2ch_handle,
    input reg_dsp_done
);
// synopsys translate_off
// assign (weak1, weak1) scalar_op = 0;
// assign (weak0, weak1) scalar_dat = 0;
// assign (weak0, weak1) scalar_qbit = 0;
// assign (weak0, weak1) complex_mul_start = 0;
// assign (weak0, weak1) complex_mul_a_r = 0;
// assign (weak0, weak1) complex_mul_i_r = 0;
// assign (weak0, weak1) complex_mul_a_i = 0;
// assign (weak0, weak1) complex_mul_b_r = 0;
// assign (weak0, weak1) complex_mul_b_i = 0;
// synopsys translate_on

reg [8:0] loop_d;
reg [8:0] loop;

localparam VEC_IDLE = 3'h0;
localparam HW_CALC = 3'h1;
localparam VEC_WAIT_LAST_WR = 3'h2;
localparam MEL_MAP = 3'h3;
localparam VEC_OP = 3'h4;
localparam VEC_OP2 = 3'h5;
localparam VEC_RESCALE = 3'h6;  // 2021/02/23 Jianlin Liang: VEC_RESCALE state is no longer needed.
// It's kept here just for completion purpose.

localparam OP_HAM_WIN = 5'h0;
localparam OP_HAM_WIN_1 = 5'h1;
localparam OP_ABS = 5'h2;
localparam OP_LN = 5'h3;
localparam OP_MFCC = 5'h4;
localparam OP_SET_OP2 = 5'h10;
localparam OP_V_ADD = 5'h11;
localparam OP_V_SUB = 5'h12;
localparam OP_V_MUL = 5'h13;
localparam OP_V_RECI = 5'h14;
localparam OP_V_SQR = 5'h15;
localparam OP_V_SQRT = 5'h16;
localparam OP_V_LN = 5'h17;
localparam OP_V_EXP = 5'h18;
localparam OP_V_INTP = 5'h19;
localparam OP_V_SQR_DIFF = 5'h1a;
localparam OP_V_DOT = 5'h1b;

wire [2*W-1:0] buf_rdata_a;  // Internal buf_rdata_a without exp part
wire [2*W-1:0] buf_rdata_b;  // Internal buf_rdata_b without exp part

reg [2*W-1:0] buf_rdata_a_d, buf_rdata_b_d;

reg [2*W_BUF-1:0] buf_rdata_a_full_d, buf_rdata_b_full_d;

wire [2*W_BUF-1:0] buf_rdata_full_d;
reg [2*W_BUF-1:0] buf_rdata_full_d2, buf_rdata_full_d3;

wire [2*W-1:0] buf_rdata_d, buf_rdata_d2, buf_rdata_d3;
reg  [2*W-1:0] tab_rdata_d;

reg read_from_a;

assign buf_rdata_a = {buf_rdata_a_full[W_BUF+W-1:W_BUF],buf_rdata_a_full[W-1:0]};
assign buf_rdata_b = {buf_rdata_b_full[W_BUF+W-1:W_BUF],buf_rdata_b_full[W-1:0]};

reg buf_a_rdata_valid, buf_b_rdata_valid;
wire tab_rdata_valid;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        buf_a_rdata_valid <= #`FFD 1'b0;
        buf_b_rdata_valid <= #`FFD 1'b0;
    end else
    begin
        buf_a_rdata_valid <= #`FFD buf_en_a && (!buf_we_a);
        buf_b_rdata_valid <= #`FFD buf_en_b && (!buf_we_b);
    end


// Inputs from data are pipelined one cycle before using them
always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        buf_rdata_a_full_d <= #`FFD {2*W_BUF{1'b0}};
        buf_rdata_b_full_d <= #`FFD {2*W_BUF{1'b0}};
        
        buf_rdata_full_d2 <= #`FFD {2*W_BUF{1'b0}};
        buf_rdata_full_d3 <= #`FFD {2*W_BUF{1'b0}};
        tab_rdata_d       <= #`FFD {2*W{1'b0}};
    end
    else
    begin
        // Only save the data when they are valid read from buffer in previous clk cycle
        if(buf_a_rdata_valid)
            buf_rdata_a_full_d <= #`FFD buf_rdata_a_full;
        if(buf_b_rdata_valid)
            buf_rdata_b_full_d <= #`FFD buf_rdata_b_full;
        if(tab_rdata_valid)
            tab_rdata_d        <= #`FFD tab_rdata;

        buf_rdata_full_d2 <= #`FFD buf_rdata_full_d;
        buf_rdata_full_d3 <= #`FFD buf_rdata_full_d2;
    end


assign buf_rdata_a_d = {buf_rdata_a_full_d[W_BUF+W-1:W_BUF], buf_rdata_a_full_d[W-1:0]};
assign buf_rdata_b_d = {buf_rdata_b_full_d[W_BUF+W-1:W_BUF], buf_rdata_b_full_d[W-1:0]};

assign buf_rdata_full_d = read_from_a ? buf_rdata_a_full_d : buf_rdata_b_full_d;
assign buf_rdata_d = {buf_rdata_full_d[W_BUF+W-1:W_BUF], buf_rdata_full_d[W-1:0]};
assign buf_rdata_d2 = {buf_rdata_full_d2[W_BUF+W-1:W_BUF], buf_rdata_full_d2[W-1:0]};
assign buf_rdata_d3 = {buf_rdata_full_d3[W_BUF+W-1:W_BUF], buf_rdata_full_d3[W-1:0]};

//---------------------------------------------------------------------

// wire [5:0] in_quant_bits_mux = in_buf_sel_mode ? in_quant_bits : prev_in_quant_bits;
wire [5:0] in_quant_bits_mux_0 = in_buf_sel_mode ? in_quant_bits : prev_in_quant_bits;
wire [5:0] in_quant_bits_mux = pure_vector_op ? op1_qbit_from_fifo : in_quant_bits_mux_0;

reg [8:0] op2_in_vector_start;
reg op2_in_buf_sel;
// reg [5:0] op2_quant_bits;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        // op2_quant_bits, op2_in_buf_sel, op2_in_vector_start <= #`FFD 10'h0;
        {op2_in_buf_sel, op2_in_vector_start} <= #`FFD 10'h0;
    end else if (start && (op == OP_SET_OP2))
    begin
        // op2_quant_bits, op2_in_buf_sel, op2_in_vector_start <= #`FFD {in_quant_bits, in_buf_sel, in_vector_start};
        {op2_in_buf_sel,op2_in_vector_start} <= #`FFD {in_buf_sel, in_vector_start} ;
    end

reg [2:0] vector_core_state, vector_core_state_next, vector_core_state_d;
wire start_hamming_win;
assign start_hamming_win = start && ((op == OP_HAM_WIN) || (op == OP_HAM_WIN_1));

wire hamming_win_end = loop == vector_len;
wire mel_map_end;
wire mel_map_end_of_write;
reg pre_for_mel_map_last_write;

wire start_vector_op = start && (
    (op == OP_LN) || (op == OP_ABS) ||
    (op == OP_V_ADD) || (op == OP_V_SUB) ||
    (op == OP_V_MUL) || (op == OP_V_RECI) ||
    (op == OP_V_SQR) || (op == OP_V_SQRT) ||
    (op == OP_V_LN) || (op == OP_V_EXP) ||
    (op == OP_V_DOT) || (op == OP_V_SQR_DIFF) ||
    (op == OP_V_INTP)
);

wire vector_op_end = loop == vector_len;
reg write_en;
wire eoag0;
wire eoag1;
//wire [9:0] rescale_pos;
//reg rescale_start;
reg busy_next;
//reg rescale_start_next;
wire same_buf;

// same_buf: 1 means save the real part of the complex mul result to the real part of accumulator.
// 0 means save the image part of the complex mul result to the image part of accumulator.
wire mac_add_start;
wire [1:0] mac_add_in_save_en;  // mac_add_in_save_en[0]: Save the real part of complex mul out
                                 // mac_add_in_save_en[1]: Save the image part of complex mul out
reg [W-1:0] mac_add_inl_real, mac_add_inl_image;
reg [5:0] mac_add_inl_real_qbit, mac_add_inl_image_qbit;

reg [W-1:0] mac_result_real, mac_result_image;
reg [5:0] mac_result_real_qbit, mac_result_image_qbit;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        vector_core_state <= #`FFD VEC_IDLE;
        busy <= #`FFD 1'b0;
        //rescale_start <= #`FFD 1'b0;
    end else 
    begin
        vector_core_state <= #`FFD vector_core_state_next;
        busy <= #`FFD busy_next;
        //rescale_start <= #`FFD rescale_start_next;
    end

always @(*)
begin
    vector_core_state_next = vector_core_state;
    busy_next = busy;
    case (vector_core_state)
        VEC_IDLE: begin
            if (start_hamming_win)
            begin
                vector_core_state_next = HW_CALC;
                busy_next = 1'b1;
            end else if (start_mel_map)
                vector_core_state_next = MEL_MAP;
                busy_next = 1'b1;
            end else if (start_vector_op)
                begin
                    vector_core_state_next = VEC_OP;
                    busy_next = 1'b1;
                end
                
        HW_CALC:
            if (hamming_win_end)
                vector_core_state_next = VEC_WAIT_LAST_WR;
        
        VEC_WAIT_LAST_WR:
            if (eoag1)
            begin
                //-- if (rescale_pos != 0)
                //-- begin
                //--    vector_core_state_next = VEC_RESCALE;
                //--    rescale_start_next = 1'b1;
                //-- end else
                //-- begin
                vector_core_state_next = VEC_IDLE;
                busy_next = 1'b0;
                //-- end
            end
        
        MEL_MAP:
            if (mel_map_end_of_write)
            begin
                vector_core_state_next = VEC_IDLE;
                busy_next = 1'b0;
            end
        
        VEC_OP:
            if (eoag0)
                vector_core_state_next = VEC_WAIT_LAST_WR;
            //-- VEC_RESCALE:
            //-- begin
            //--    rescale_start_next = 1'b0;
            //--    if (eoq1)
            //--    begin
            //--        vector_core_state_next = VEC_IDLE;
            //--        busy_next = 1'b0;
            //--    end
            //-- end
        endcase
always @(posedge clk or negedge rst_n)
    if (!rst_n)
        loop <= #`FFD 0;
    else if (start_hamming_win || start_mel_map || start_vector_op)
        loop <= #`FFD 0;
    else if ((vector_core_state == HW_CALC) || (vector_core_state == MEL_MAP))
        loop <= #`FFD loop + 1;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        loop_d <= #`FFD 0;
        vector_core_state_d <= #`FFD VEC_IDLE;
    end else
    begin
        loop_d <= #`FFD loop;
        vector_core_state_d <= #`FFD vector_core_state;
    end

reg addr_gen_0_start;
wireG[8:0] read_addr;
wire read_en;
reg [8:0] vector_len_ag0;
reg [8:0] vector_len_ag1;
reg [8:0] vector_len_ag2;                          
reg [3:0] addr_incr_interval_ag0;
reg [3:0] addr_incr_interval_ag1;
reg [3:0] addr_incr_interval_ag2;
//-- wire [8:0] ag0_addr_start = (vector_core_state == VEC_RESCALE) ? out_vector_start : in_vector_start;
wire [8:0]ag0_addr_start = in_vector_start;

addr_gen #(
    .AW(9)
) addr_gen_0 (
    .clk(clk),
    .rst_n(rst_n),
    .start(addr_gen_0_start),
    .addr_type(2'b0),
    .addr_start(ag0_addr_start[8:0]),
    .addr_incr(9'h1),
    .addr_incr_interval(addr_incr_interval_ag0),
    .vector_len(vector_len_ag0[8:0]),
    .addr_en(read_en),
    .addr(read_addr),
    .end_of_addr_gen(eoag0)
);

// address generation for output vector
reg [12:0] start_d;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        start_d <= #`FFD 13'h0;
    else
        start_d <= #`FFD {start_d[11:0], start};
// Add the part for DOT PRODUCT
//-----------------------------------
reg eoag0_d;
wire dot_prod_addr_gen_1_start;
reg dot_prod_addr_gen_1_start_d;
wire dot_prod_result_fifo_din_vld;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        eoag0_d <= #`FFD 1'b0;
    else if (start)
        eoag0_d <= #`FFD 1'b0;
    else
        eoag0_d <= #`FFD eoag0_d;

assign dot_prod_addr_gen_1_start = (!same_buf) ? eoag0 : eoag0_d;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        dot_prod_addr_gen_1_start_d <= #`FFD 1'b0;
    else if (start)
        dot_prod_addr_gen_1_start_d <= #`FFD 1'b0;
    else
        dot_prod_addr_gen_1_start_d <= #`FFD dot_prod_addr_gen_1_start;

assign dot_prod_result_fifo_din_vld = dot_prod_addr_gen_1_start_d || dot_prod_addr_gen_1_start;

//--------------------------------------------------------
reg addr_gen_1_start;
wire addr_en_ag1;
wire [8:0] addr_ag1;
wire [8:0] ag1_addr_start = out_vector_start;

addr_gen #(
    .AW(9)
) addr_gen_1 (
    .clk(clk),
    .rst_n(rst_n),
    .start(addr_gen_1_start),
    .addr_type(2'b0),
    .addr_start(ag1_addr_start[8:0]),
    .addr_incr(9'h1),
    .addr_incr_interval(addr_incr_interval_ag1),
    .vector_len(vector_len_ag1[8:0]),
    .addr_en(addr_en_ag1),
    .addr(addr_ag1),
    .end_of_addr_gen(eoag1)
);

// address generation for OP2 vector
// reg [4:0] op2_start_d;

// always @(posedge clk or negedge rst_n)
//     if (!rst_n)
//         op2_start_d <= #`FFD 4'h0;
//     else
//         op2_start_d <= #`FFD {op2_start_d[3:0], addr_gen_0_start};

reg addr_gen_2_start;
wire addr_en_op2;
wire [8:0] addr_op2;
wire eoag2;

addr_gen #(
    .AW(9)
) addr_gen_2 (
    .clk(clk),
    .rst_n(rst_n),
    .start(addr_gen_2_start),
    .addr_type(2'b0),
    .addr_start(op2_in_vector_start[8:0]),
    .addr_incr(9'h1),
    .addr_incr_interval(addr_incr_interval_ag2),
    .vector_len(vector_len_ag2[8:0]),
    .addr_en(addr_en_op2),
    .addr(addr_op2),
    .end_of_addr_gen(eoag2)
);

reg addr_gen_tab_start;
wire [8:0] vector_len_tab;
wire [3:0] addr_incr_interval_tab;
wire [1:0] addr_type_tab;
wire [9:0] addr_tab;
wire addr_en_tab;
wire eoag_tab;
wire [1:0] stage_tab;
reg [1:0] stage_tab_d;
reg tab_rdata_valid_d;
if (REG)
begin
    always @(posedge clk or negedge rst_n)
        if (!rst_n)
        begin
            stage_tab_d <= #`FFD 2'b0;
            tab_rdata_valid_d <= #`FFD 1'b0;
        end else
        begin
            stage_tab_d <= #`FFD stage_tab;
            tab_rdata_valid_d <= #`FFD tab_rdata_valid;
        end
end else
begin
    always @(*)
    begin
        stage_tab_d = stage_tab;
        tab_rdata_valid_d = tab_rdata_valid;
    end
end

addr_gen #(
    .AW(10)
) addr_gen_tab (
    .clk(clk),
    .rst_n(rst_n),
    .addr_type(addr_type_tab),
    .start(addr_gen_tab_start),
    .addr_start(tab_offset[9:0]),
    .addr_incr(10'h1),
    .addr_incr_interval(addr_incr_interval_tab),
    .vector_len(vector_len_tab[8:0]),
    .addr_en(addr_en_tab),
    .addr(addr_tab),
    .addr_en_d(tab_rdata_valid),
    .end_of_addr_gen(eoag_tab),
    .stage(stage_tab[1:0])
);
assign same_buf = op2_in_buf_sel == (in_buf_sel_mode ? in_buf_sel : prev_in_buf_sel);
reg same_buf_d;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        same_buf_d <= #`FFD 1'b0;
    else if (start)
        same_buf_d <= #`FFD same_buf;

wire rd_same_buf = start ? same_buf : same_buf_d;
always @(*)
begin
    addr_gen_0_start = 0;
    addr_gen_2_start = 0;
    addr_gen_1_start = 0;
    vector_len_ag0 = vector_len;
    vector_len_ag2 = vector_len;
    vector_len_ag1 = vector_len;
    addr_incr_interval_ag0 = 0;
    addr_incr_interval_ag1 = 0;
    addr_incr_interval_ag2 = 0;
    addr_gen_tab_start = 0;
    vector_len_tab = 0;
    addr_incr_interval_tab = 0;
    addr_type_tab = 0;
    if (op != OP_SET_OP2)
    begin
        case (op)
            OP_HAM_WIN:
            begin
                addr_gen_0_start = tab_type == 1 ? start_d[1] : start;
                addr_gen_1_start = tab_type == 1 ? 
                    (REG ? start_d[5] : start_d[2]) : 
                    (REG ? start_d[3] : start_d[0]);
                addr_gen_tab_start = REG ? start_d[0] : start;
                vector_len_tab = vector_len >> 2;
                addr_incr_interval_tab = 1;
                addr_type_tab = $unsigned(tab_type ? 2 : 1);
            end
            OP_ABS:
            begin
                addr_gen_0_start = start;
                addr_gen_1_start = REG ? start_d[8] : start_d[1];
            end
            OP_MFCC:
            begin
                addr_gen_0_start = start;
                addr_gen_1_start = 0;
                addr_gen_tab_start = REG ? start_d[6] : start;
                vector_len_tab = vector_len;
                addr_incr_interval_tab = 0;
                addr_type_tab = 0;
            end
            
            OP_V_ADD, OP_V_SUB, OP_V_MUL, OP_V_SQR:
            begin
                addr_gen_0_start = start;
                if (!scalar_op && (op != OP_V_SQR))
                    addr_gen_2_start = rd_same_buf ? start_d[0] : start;
                
                if (((op == OP_V_MUL) && complex_op))
                    if (scalar_op)
                        addr_gen_1_start = start_d[6];
                    else if (!same_buf)
                        addr_gen_1_start = start_d[6];
                    else
                        addr_gen_1_start = start_d[7];
                else if (!scalar_op)
                    if (op == OP_V_SQR)
                        addr_gen_1_start = start_d[4];
                    else
                        addr_gen_1_start = rd_same_buf ? start_d[5] : start_d[4];
                else
                    addr_gen_1_start = start_d[4];
                
                vector_len_ag0 = complex_op ? vector_len : vector_len >> 1;
                vector_len_ag2 = complex_op ? vector_len : vector_len >> 1;
                vector_len_ag1 = complex_op ? vector_len : vector_len >> 1;
                addr_incr_interval_ag0 = 1;
                addr_incr_interval_ag1 = 1;
                addr_incr_interval_ag2 = 1;
            end
            OP_V_DOT:
            begin
                addr_gen_0_start = start;
                addr_gen_2_start = rd_same_buf ? start_d[0] : start;
                addr_gen_1_start = dot_prod_addr_gen_1_start;
            
                vector_len_ag0 = vector_len;
                vector_len_ag2 = vector_len;
                vector_len_ag1 = 0; // for dot product only needs to output one final result. Thus vector_len_ag1 = 1-1 = 0
                addr_incr_interval_ag0 = 9;
                addr_incr_interval_ag1 = 1;
                addr_incr_interval_ag2 = 9;
            end
            
            OP_V_RECI:
            begin
                addr_gen_0_start = start;
                addr_gen_1_start = REG ? start_d[7] : start_d[2];
                vector_len_ag0 = vector_len >> 1;
                vector_len_ag1 = vector_len >> 1;
                addr_incr_interval_ag0 = 1;
                addr_incr_interval_ag1 = 1;
            end
            OP_V_SQRT:
            begin
                addr_gen_0_start = start;
                addr_gen_1_start = REG ? start_d[10] : start_d[2];
                vector_len_ag0 = vector_len >> 1;
                vector_len_ag1 = vector_len >> 1;
                addr_incr_interval_ag0 = 1;
                addr_incr_interval_ag1 = 1;
            end
            
            OP_V_LN:
            begin
                addr_gen_0_start = start;
                addr_gen_1_start = REG ? start_d[6] : start_d[2];
                vector_len_ag0 = vector_len >> 1;
                vector_len_ag1 = vector_len >> 1;
                addr_incr_interval_ag0 = 1;
                addr_incr_interval_ag1 = 1;
            end
            
            OP_V_EXP:
            begin
                addr_gen_0_start = start;
                addr_gen_1_start = start_d[12];
                vector_len_ag0 = vector_len >> 1;
                vector_len_ag1 = vector_len >> 1;
                addr_incr_interval_ag0 = 11;
                addr_incr_interval_ag1 = 11;
            end
            
            OP_V_SQR_DIFF:
            begin
                addr_gen_0_start = start;
                addr_gen_2_start = rd_same_buf ? start_d[0] : start;
                addr_gen_1_start = rd_same_buf ? start_d[7] : start_d[6];
            
                vector_len_ag0 = vector_len >> 1;
                vector_len_ag2 = vector_len >> 1;
                vector_len_ag1 = vector_len >> 1;
            
                addr_incr_interval_ag0 = 1;
                addr_incr_interval_ag1 = 1;
                addr_incr_interval_ag2 = 1;
            end
            OP_V_INTP :
            begin
                addr_gen_0_start = start;
                addr_gen_1_start = start_d[8];
                vector_len_ag0 = vector_len - 1;
                addr_incr_interval_ag0 = 3;
                addr_incr_interval_ag1 = 3;
            end
            endcase
        end
    end

/// buffer read logic
// reg rvalid_from_a;
wire write_to_b = read_from_a;  //rvalid_from_a;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        read_from_a <= #`FFD 1;
    else if (start)
        read_from_a <= #`FFD in_buf_sel_mode ? ~in_buf_sel : ~prev_in_buf_sel;
    // always @(posedge clk or negedge rst_n)
    //     if (!rst_n)
    //         rvalid_from_a <= #`FFD 1'b1;
    //     else
    //         rvalid_from_a <= #`FFD read_from_a;

reg rdata_valid, rdata_valid_d;

wire [2*W_BUF-1:0] op2_buf_rdata_full = op2_in_buf_sel ? buf_rdata_b_full_d : buf_rdata_a_full_d;
wire [2*W-1:0] op2_buf_rdata = op2_in_buf_sel ? buf_rdata_b_d : buf_rdata_a_d;
reg [1:0] addr_en_op2_d;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        addr_en_op2_d <= #`FFD 2'b0;
    end else
    begin
        addr_en_op2_d <= #`FFD {addr_en_op2_d[0], addr_en_op2};
    end
// op_group_0, for real add/sub/mul/sqr
wire op_group_0 = (op == OP_V_ADD) || (op == OP_V_SUB) || (op == OP_V_MUL) && !complex_op || (op == OP_V_SQR);
// op_group_1, for complex mul
wire op_group_1 = (op == OP_V_MUL) && complex_op;
// op_group_2, for reciprocal
wire op_group_2 = (op == OP_V_RECI);
// op_group_3, for complex dot product
wire op_group_3 = (op == OP_V_DOT);
// op_group_4, for square difference
wire op_group_4 = (op == OP_V_SQR_DIFF);
// op_group_5, for linear interpolation
wire op_group_5 = (op == OP_V_INTP);

//wire two_buf_op_group = (((op == OP_V_ADD) || (op == OP_V_SUB) || (op == OP_V_MUL)) && !scalar_op) || (op == OP_V_DOT);
wire two_buf_op_group = (((op == OP_V_ADD) || (op == OP_V_SUB) || (op == OP_V_MUL)) && !scalar_op) || op_group_3 || op_group_4;

wire start_v_intp = start && op_group_5;

// wire rescaling = vector_core_state == VEC_RESCALE;
// wire op1_din_vld = rdata_valid && (op_group_0 || op_group_1 || 
//     (op == OP_V_SQRT) || (op == OP_V_LN) || (op == OP_V_EXP)) && !rescaling;
// wire op1_din_vld = rdata_valid && (op_group_0 || op_group_1 || op_group_4 || op_group_5 || (op == OP_V_SQRT) || (op == OP_V_LN) || (op == OP_V_EXP));
wire op1_din_vld = rdata_valid && (op_group_0 || op_group_1 || op_group_2|| op_group_3|| op_group_4 || op_group_5 || (op == OP_V_SQRT) || (op == OP_V_LN) || (op == OP_V_EXP));
wire op2_din_vld = addr_en_op2_d[1] & two_buf_op_group;

reg op1_din_vld_d;
wire [W-1:0] op1_from_fifo;
wire op1_from_fifo_vld;
wire op1_from_fifo_rdy;
wire op1_from_fifo_vld_rdy = op1_from_fifo_vld && op1_from_fifo_rdy;
reg [9:0] op1_from_fifo_vld_rdy_d;

always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        op1_from_fifo_vld_rdy_d <= #`FFD 10'h0;
        op1_din_vld_d <= #`FFD 1'b0;
    end else begin
        op1_from_fifo_vld_rdy_d <= #`FFD {op1_from_fifo_vld_rdy_d[8:0], op1_from_fifo_vld_rdy};
        op1_din_vld_d <= #`FFD op1_din_vld;
    end

wire is_real;

reg [2*W_BUF+1:0] op1_fifo_din;
reg op1_fifo_din_vld;

always @(*) begin
    if ((( (op == OP_V_ADD) || (op == OP_V_SUB) || (op == OP_V_MUL)) && (!scalar_op)) || op_group_3 || op_group_4)
    begin
        op1_fifo_din_vld = rd_same_buf? op1_din_vld_d : op1_din_vld;
        op1_fifo_din = rd_same_buf? {1'b0, buf_rdata_full_d2[2*W_BUF-1:W_BUF], 1'b1, buf_rdata_full_d2[W_BUF-1:0]} : 
                        {1'b0, buf_rdata_full_d[2*W_BUF-1:W_BUF], 1'b1, buf_rdata_full_d[W_BUF-1:0]};
    end
    else begin
        op1_fifo_din_vld = op1_din_vld;
        op1_fifo_din = {1'b0, buf_rdata_full_d[2*W_BUF-1:W_BUF], 1'b1, buf_rdata_full_d[W_BUF-1:0]};
    end
end

fifo_2to1 #(
    .W(W_BUF+1)
) op1_fifo (
    .clk(clk),
    .rst_n(rst_n),

    .flush(1'b0),
    .din(op1_fifo_din),
    .din_vld(op1_fifo_din_vld),
    .din_rdy(),
    .dout({is_real, op1_qbit_from_fifo[5:0], op1_from_fifo[W-1:0]}),
    .dout_vld(op1_from_fifo_vld),
    .dout_rdy(op1_from_fifo_rdy)
);

wire [W-1:0] op2_from_fifo;
wire op2_from_fifo_vld;
wire op2_from_fifo_rdy;

fifo_2to1 #(
    .W(W_BUF)
) op2_fifo (
    .clk(clk),
    .rst_n(rst_n),

    .flush(1'b0),
    .din(op2_buf_rdata_full[2*W_BUF-1:0]),
    .din_vld(op2_din_vld),
    .din_rdy(),

    .dout({op2_qbit_from_fifo[5:0], op2_from_fifo[W-1:0]}),
    .dout_vld(op2_from_fifo_vld),
    .dout_rdy(op2_from_fifo_rdy)
);
/////////////////////////////////////////////
/////////////////////////////////////////////
//ADD/SUB/MUL/ Unit
/////////////////////////////////////////////
/////////////////////////////////////////////
reg [W-1:0] operand_1;
reg [W-1:0] operand_2;
wire [W-1:0] op1_norm;
wire [5:0] op1_norm_quant_bit;
reg [W-1:0] asm_result;
wire op_group_0_op1_en_d = op1_from_fifo_vld_rdy_d[0] && op_group_0;

reg [2*W-1:0] mul_result;
wire [W-1:0] mul_result_rescaled;
wire [5:0] rescale_result_nbit;
reg [W-1:0] quantized_reci_result;
reg [5:0] reci_result_quant_bit;

always @(*)
begin
    if (op_group_3) // Dot Product
    begin
        operand_1 = mac_add_start[0] ? mac_add_in1_real :
            (mac_add_start[1] ? mac_add_in1_image : {W{1'b0}});

        operand_2 = mac_add_start[0] ? mac_result_real : 
            (mac_add_start[1] ? mac_result_image : {W{1'b0}});
    end
    else if (op_group_4) begin // Square Difference
        operand_1 = op1_from_fifo[W-1:0];
        operand_2 = op2_from_fifo[W-1:0];
    end
    else begin
        operand_1 = op1_from_fifo[W-1:0];
        operand_2 = scalar_op ? ((!complex_op || is_real) ? scalar_dat : complex_image_dat) : op2_from_fifo[W-1:0];
    end
end

reg [4:0] op1_rshift;
reg [4:0] op2_rshift;

reg [5:0] operand_1_quant_bits, operand_2_quant_bits;

always @(*)
begin
    if (op_group_3) begin // Dot Product
        operand_1_quant_bits = mac_add_start[0] ? mac_add_in1_real_qbit :
                                (mac_add_start[1] ? mac_add_in1_image_qbit : {6{1'b0}});
        operand_2_quant_bits = mac_add_start[0] ? mac_result_real_qbit :
                                (mac_add_start[1] ? mac_result_image_qbit : {6{1'b0}});
    end
    else if (op_group_4) begin // Square Difference
        operand_1_quant_bits = in_quant_bits_mux;
        operand_2_quant_bits = op2_qbit_from_fifo;
    end
    else begin
        operand_1_quant_bits = in_quant_bits_mux;
        operand_2_quant_bits = scalar_op ? scalar_qbit : op2_qbit_from_fifo;
    end
end

reg [5:0] add_sub_quant_bits;
reg [5:0] add_sub_quant_bits_d;

always @(*)
begin
    reg signed [6:0] q_diff_sign;
    reg [6:0] q_diff;

    if ($signed(operand_1_quant_bits) > $signed(operand_2_quant_bits))
    begin
        q_diff_sign = $signed(operand_1_quant_bits) - $signed(operand_2_quant_bits);
        q_diff = $unsigned(q_diff_sign);
        op1_rshift = 0;
        op2_rshift = q_diff < 7'd32 ? q_diff : 5'd31;
        add_sub_quant_bits = operand_1_quant_bits;
    end else begin
        q_diff_sign = $signed(operand_2_quant_bits) - $signed(operand_1_quant_bits);
        q_diff = $unsigned(q_diff_sign);
        op1_rshift = q_diff<7'd32 ? q_diff : 5'd31;
        op2_rshift = 0;
        add_sub_quant_bits = operand_2_quant_bits;
    end
end

wire signed [W-1:0] operand1_sft = $signed(operand_1) >>> op1_rshift;
wire signed [W-1:0] operand2_sft = $signed(operand_2) >>> op2_rshift;
reg signed [W:0] add_sub_result;

always @(*)
begin : add_sub_blk
    if ((op == OP_V_ADD) || (op == OP_V_DOT))
        add_sub_result = operand_1_sft + operand_2_sft;
    else
        add_sub_result = operand_1_sft - operand_2_sft;
end

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        add_sub_quant_bits_d <= #`FFD 6'b0;
    end else begin
        add_sub_quant_bits_d <= #`FFD add_sub_quant_bits;
    end

//--------------------------------------------------------------------------
// Prepare input data for 2nd calculation cycle of Square Difference
// These data will be sent to abs module
//--------------------------------------------------------------------------

reg signed [W-1:0] square_diff_in1, square_diff_in2;
reg [5:0] square_diff_in_quant_bits;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        square_diff_in1 <= #`FFD {W{1'b0}};
        square_diff_in2 <= #`FFD {W{1'b0}};
        // square_diff_in_quant_bits <= #`FFD 6'b0;
    end else begin
        square_diff_in1 <= #`FFD operand_1_sft;
        square_diff_in2 <= #`FFD operand_2_sft;
        //square_diff_in_quant_bits <= #`FFD add_sub_quant_bits;
    end

always @(*) begin
    square_diff_in_quant_bits = add_sub_quant_bits_d;
end

//--------------------------------------------------------------------------
// MUL complex
//--------------------------------------------------------------------------

// assign complex_mul_en = op_group_1;
// wire op_group_1_rdata_valid = rdata_valid & op_group_1 && (vector_core_state == VEC_OP);

assign complex_mul_en = op_group_1 || op_group_3;
wire op_group_1_rdata_valid = rdata_valid & (op_group_1 || op_group_3) && (vector_core_state == VEC_OP);
reg [1:0] op_group_1_rdata_valid_d;

assign complex_mul_a_r = ((scalar_op && op_group_1) || !same_buf) ? buf_rdata_d[W-1:0] :
                         (op_group_1_rdata_valid_d[1] ? buf_rdata_d2[W-1:0] : buf_rdata_d3[W-1:0]);

assign complex_mul_a_i = ((scalar_op && op_group_1) || !same_buf) ? buf_rdata_d[2*W-1:W] :
                         (op_group_1_rdata_valid_d[1] ? buf_rdata_d2[2*W-1:W] : buf_rdata_d3[2*W-1:W]);

assign complex_mul_b_r = (scalar_op && op_group_1) ? scalar_dat :
                         (same_buf ? (op_group_1_rdata_valid_d[1] ? buf_rdata_d[W-1:0] : buf_rdata_d2[W-1:0]) : op2_buf_rdata[W-1:0]);

assign complex_mul_b_i = (scalar_op && op_group_1) ? complex_image_dat :
                         (same_buf ? (op_group_1_rdata_valid_d[1] ? buf_rdata_d[2*W-1:W] : buf_rdata_d2[2*W-1:W]) : op2_buf_rdata[2*W-1:W]);

   
reg [2*W-1:0] complex_mul_o_r_d;
reg [2*W-1:0] complex_mul_o_i_d, complex_mul_o_i_d2;
reg [7:0]     complex_mul_start_d;

wire complex_mul_result_valid_d = complex_mul_start_d[2] | complex_mul_start_d[3];
reg  complex_mul_result_valid_d2;
            
always @(posedge clk or negedge rst_n)
if (!rst_n)
begin
    op_group_1_rdata_valid_d <= #`FFD 2'b0;
    complex_mul_o_r_d        <= #`FFD {2*W{1'b0}};
    complex_mul_o_i_d        <= #`FFD {2*W{1'b0}};
    complex_mul_o_i_d2       <= #`FFD {2*W{1'b0}};
    complex_mul_start_d      <= #`FFD 8'b0;
    complex_mul_result_valid_d2 <= #`FFD 1'b0;
    ///{buf_rdata_full_d2, buf_rdata_full_d} <= #`FFD {4*W_BUF{1'b0}};
end else
begin
    op_group_1_rdata_valid_d <= #`FFD {op_group_1_rdata_valid_d[0], op_group_1_rdata_valid};
    complex_mul_o_r_d        <= #`FFD complex_mul_o_r;
    complex_mul_o_i_d        <= #`FFD complex_mul_o_i;
    complex_mul_o_i_d2       <= #`FFD complex_mul_o_i_d;
    complex_mul_start_d      <= #`FFD {complex_mul_start_d[6:0], complex_mul_start};
    complex_mul_result_valid_d2 <= #`FFD complex_mul_result_valid_d;
    ///{buf_rdata_d2, buf_rdata_d} <= #`FFD {buf_rdata_d, buf_rdata};
    ///{buf_rdata_full_d2, buf_rdata_full_d} <= #`FFD {buf_rdata_full_d, buf_rdata_full};
end
assign complex_mul_start = op_group_1 ? ((scalar_op || !same_buf) ? op_group_1_rdata_valid_d[0] : op_group_1_rdata_valid_d[1]) :
                            ((!same_buf) ? op_group_1_rdata_valid_d[0] : op_group_1_rdata_valid_d[1]);  // This is for

//////////////////////////////////////////////////////
/// rescale ADD/SUB/MUL/Square Diff result
//////////////////////////////////////////////////////
//wire [2*W-1:0] rescale_in =
//       op_group_1? (complex_mul_start_d[0] ? complex_mul_o_r : complex_mul_o_i_d) :
//       ((op == OP_V_MUL) || (op == OP_V_SQR) ? mul_result[2*W-1:0] :
//       {{W-1{add_sub_result[W]}}, add_sub_result});

wire [2*W-1:0] square_diff_out;
wire [5:0]     square_diff_in_qbits_d2;

reg  [2*W-1:0] rescale_in;
reg  [2*W-1:0] rescale_in_d, rescale_in_d_tmp;

always @(*)
begin
    if(op_group_3) begin
        rescale_in = complex_mul_start_d[1] ? complex_mul_o_r :
                     complex_mul_start_d[2] ? complex_mul_o_i_d : {{W-1{add_sub_result[W]}}, add_sub_result};
    end
end
    else begin
        rescale_in = op_group_1? (complex_mul_start_d[1] ? complex_mul_o_r : complex_mul_o_i_d) :
                     (op == OP_V_MUL) || (op == OP_V_SQR) ? mul_result[2*W-1:0] :
                     {{W-1{add_sub_result[W]}}, add_sub_result};
    end
end

// Pipeline rescale input for 1 cycle
if(REG)
begin
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            rescale_in_d_tmp <= #`FFD {2*W{1'b0}};
        end
        else begin
            rescale_in_d_tmp <= #`FFD rescale_in;
        end
end else
begin
    always @(*)
        rescale_in_d_tmp = rescale_in;
end

assign rescale_in_d = op_group_4? square_diff_out : rescale_in_d_tmp;

wire [W-1:0] rescale_out;

rescale #(
    .W(W)
) rescale (
    .din(rescale_in_d[2*W-1:0]),
    .dout(rescale_out[W-1:0]),
    .nbit_scaled(rescale_result_nbit[5:0])
);
//--------------------------------------------------------------------------------------
// 2021/11/29. Complex mul takes 3 cycles, plus the pipeline, thus quant bits for it needs to pipeline 3 cycles as well.
// This part may not be useful now. It's just kept here for potential future use.
//--------------------------------------------------------------------------------------
reg signed [5:0] in_quant_bits_mux_d, in_quant_bits_mux_d2, in_quant_bits_mux_d3;
reg signed [5:0] op2_qbit_from_fifo_d, op2_qbit_from_fifo_d2, op2_qbit_from_fifo_d3;

if(REG)
begin
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            in_quant_bits_mux_d  <= #`FFD 6'b0;
            in_quant_bits_mux_d2 <= #`FFD 6'b0;
            in_quant_bits_mux_d3 <= #`FFD 6'b0;
            
            op2_qbit_from_fifo_d  <= #`FFD 6'b0;
            op2_qbit_from_fifo_d2 <= #`FFD 6'b0;
            op2_qbit_from_fifo_d3 <= #`FFD 6'b0;
        end
        else begin
        in_quant_bits_mux_d   <= #`FFD in_quant_bits_mux[5:0];
        in_quant_bits_mux_d2  <= #`FFD in_quant_bits_mux_d;
        in_quant_bits_mux_d3  <= #`FFD in_quant_bits_mux_d2;

        op2_qbit_from_fifo_d  <= #`FFD op2_qbit_from_fifo[5:0];
        op2_qbit_from_fifo_d2 <= #`FFD op2_qbit_from_fifo_d;
        op2_qbit_from_fifo_d3 <= #`FFD op2_qbit_from_fifo_d2;
    end
end else
begin
    always @(*)
    begin
        in_quant_bits_mux_d   = in_quant_bits_mux[5:0];
        in_quant_bits_mux_d2  = in_quant_bits_mux[5:0];
        in_quant_bits_mux_d3  = in_quant_bits_mux[5:0];

        op2_qbit_from_fifo_d  = op2_qbit_from_fifo[5:0];
        op2_qbit_from_fifo_d2 = op2_qbit_from_fifo[5:0];
        op2_qbit_from_fifo_d3 = op2_qbit_from_fifo[5:0];
    end
end
reg signed [5:0] tmp_in_quant_bits_mux, tmp_op2_qbit_from_fifo;

if(REG)
    always @(*)
    begin
        tmp_in_quant_bits_mux  = complex_op? in_quant_bits_mux_d3  : in_quant_bits_mux_d;
        tmp_op2_qbit_from_fifo = complex_op? op2_qbit_from_fifo_d3 : op2_qbit_from_fifo_d;
    end
else
    always @(*)
    begin
        tmp_in_quant_bits_mux  = in_quant_bits_mux[5:0];
        tmp_op2_qbit_from_fifo = op2_qbit_from_fifo[5:0];
    end


reg signed [7:0] op_group_0_1_result_quant_bits;
always @(*)
begin
    op_group_0_1_result_quant_bits = 0;
    case (op)
        OP_V_ADD, OP_V_SUB:
            op_group_0_1_result_quant_bits = $signed(rescale_result_nbit[5:0]) + $signed(add_sub_quant_bits_d);
        OP_V_MUL:
            op_group_0_1_result_quant_bits = $signed(rescale_result_nbit[5:0]) + $signed(tmp_in_quant_bits_mux[5:0]) + $signed(scalar_op ? scalar_qbit[5:0] : tmp_op2_qbit_from_fifo[5:0]) - (W-1);
            //op_group_0_1_result_quant_bits = $signed(rescale_result_nbit[5:0]) + $signed(in_quant_bits_mux[5:0]) + $signed(scalar_op ? scalar_qbit[5:0] : op2_qbit_from_fifo[5:0]) - (W-1);
        OP_V_DOT:
            if(complex_mul_start_d[2] || complex_mul_start_d[3])  // rescale for the complex mul result
                op_group_0_1_result_quant_bits = $signed(rescale_result_nbit[5:0]) + $signed(tmp_in_quant_bits_mux[5:0]) + $signed(tmp_op2_qbit_from_fifo[5:0]) -(W-1);
            else  // rescale for the add result of MAC
                op_group_0_1_result_quant_bits = $signed(rescale_result_nbit[5:0]) + $signed(add_sub_quant_bits_d);
        OP_V_SQR:
            op_group_0_1_result_quant_bits = $signed(rescale_result_nbit[5:0]) + ($signed(in_quant_bits_mux_d) <<1) - (W-1);
        OP_V_SQR_DIFF:
            op_group_0_1_result_quant_bits = $signed(rescale_result_nbit[5:0]) + ($signed(square_diff_in_qbits_d2) <<1) - (W-1);
    endcase
    if (op_group_0_1_result_quant_bits < -55)
        op_group_0_1_result_quant_bits = -56;
end

//////////////////////////////////////////////////////
/// vector block rescale quantization
//////////////////////////////////////////////////////

wire [6:0] reci_out_quant_bits;
reg  [W-1:0] result_rescale_data_in;
reg  [6:0] result_rescale_quant_bits_in;
wire zero_in;
reg  set_inf;
wire [W-1:0] abs_result;
wire [5:0] complex_abs_o_quant;
wire [W-1:0] reci_result;
wire signed [W-1:0] ln_out;
wire [5:0] ln_out_quant_bit;
wire [W-1:0] exp_out;
wire [5:0] exp_out_quant_bits;

always @(*)
begin
    result_rescale_data_in  = 0;
    result_rescale_quant_bits_in = 0;
    set_inf = 0;
    
    if (op_group_0 || op_group_1 || op_group_3 || op_group_4)
    begin
        result_rescale_data_in  = rescale_out[W-1:0];
        result_rescale_quant_bits_in = op_group_0_1_result_quant_bits[6:0];
    end else if (op == OP_V_RECI)
    begin
        result_rescale_data_in  = reci_result;
        result_rescale_quant_bits_in = reci_out_quant_bits;
        set_inf = zero_in;
    end else if (op == OP_V_SQRT)
    begin
        result_rescale_data_in  = abs_result[W-1:0];
        result_rescale_quant_bits_in = {complex_abs_o_quant[5], complex_abs_o_quant[5:0]};
    end else if (op == OP_V_LN)
    begin
        result_rescale_data_in  = ln_out; //abs_result[W-1:0];
        result_rescale_quant_bits_in = {ln_out_quant_bit[5], ln_out_quant_bit};
    end else if (op == OP_V_EXP)
    begin
        result_rescale_data_in  = exp_out; //abs_result[W-1:0];
        result_rescale_quant_bits_in = {exp_out_quant_bits[5], exp_out_quant_bits};
    end
end
wire        sqrt_result_valid;
wire [2*W-1:0] rescaled_data;

wire [W-1:0] s0_out;
wire  [5:0]  s0_out_quant_bit;
wire  [5:0] quant_bits_max;
wire        reci_result_valid;
wire        exp_result_valid;
wire        ln_result_valid;

reg         sqrt_result_valid_d, reci_result_valid_d, exp_result_valid_d, ln_result_valid_d;

if(REG)
begin
    always @(posedge clk or negedge rst_n)
    begin
        if (!rst_n) begin
            sqrt_result_valid_d <= #`FFD 1'b0;
            reci_result_valid_d <= #`FFD 1'b0;
            exp_result_valid_d  <= #`FFD 1'b0;
            ln_result_valid_d   <= #`FFD 1'b0;
        end
        else begin
            sqrt_result_valid_d <= #`FFD sqrt_result_valid;
            reci_result_valid_d <= #`FFD reci_result_valid;
            exp_result_valid_d  <= #`FFD exp_result_valid;
            ln_result_valid_d   <= #`FFD ln_result_valid;
        end
    end
end else
begin
    always @(*) begin
        sqrt_result_valid_d = sqrt_result_valid;
        reci_result_valid_d = reci_result_valid;
        exp_result_valid_d  = exp_result_valid;
        ln_result_valid_d   = ln_result_valid;
    end
end

wire op_group_1_result_valid_d = op1_from_fifo_vld_rdy_d[2] && op_group_1;

// 2022/10/10 jiliang: For dot product this signal needs to be calculated carefully.
//wire op_group_3_result_valid = dot_prod_result_fifo_din_vld && op_group_3;
wire op_group_3_result_valid_d = dot_prod_result_fifo_din_vld && op_group_3;

wire op_group_4_result_valid_d = op1_from_fifo_vld_rdy_d[2] && op_group_4;
//wire op_group_4_result_valid_d = op1_from_fifo_vld_rdy_d[1] && op_group_4;

//wire block_rescale_s0_en = op_group_0_op1_en_d || op_group_1_result_valid_d || reci_result_valid ||
//                           sqrt_result_valid || exp_result_valid || op_group_3_result_valid || op_group_4_result_valid_d;

wire block_rescale_s0_en = op_group_0_op1_en_d || op_group_1_result_valid_d || reci_result_valid ||
                           sqrt_result_valid || exp_result_valid || op_group_4_result_valid_d;

// Used by Dot Product
wire [W-1:0] mac_internal_out;
wire  [5:0]  mac_internal_out_quant_bit;

//------------------------------------------------------------
// Pipeline one more cycle before sending to block_rescale_prepare
reg  block_rescale_s0_en_d_tmp;
wire block_rescale_s0_en_d;
reg  set_inf_d;
reg  [W-1:0] result_rescale_data_in_d;
reg  [6:0] result_rescale_quant_bits_in_d;

if (REG)
begin
    always @(posedge clk or negedge rst_n)
    begin
        if (!rst_n) begin
            block_rescale_s0_en_d_tmp   <= #`FFD 1'b0;
            set_inf_d                   <= #`FFD 1'b0;
            result_rescale_data_in_d     <= #`FFD {W{1'b0}};
            result_rescale_quant_bits_in_d <= #`FFD 7'b0;
        end
        else begin
            block_rescale_s0_en_d_tmp   <= #`FFD block_rescale_s0_en;
            set_inf_d                   <= #`FFD set_inf;
            result_rescale_data_in_d     <= #`FFD result_rescale_data_in;
            result_rescale_quant_bits_in_d <= #`FFD result_rescale_quant_bits_in;
        end
    end

    // 2022/10/10 jiliang: For dot product needs to take care of separately
    assign block_rescale_s0_en_d = block_rescale_s0_en_d_tmp || op_group_3_result_valid_d;
end else
begin
    always @(*) begin
        block_rescale_s0_en_d_tmp   = block_rescale_s0_en;
        set_inf_d                   = set_inf;
        result_rescale_data_in_d     = result_rescale_data_in;
        result_rescale_quant_bits_in_d = result_rescale_quant_bits_in;
    end
end

assign block_rescale_s0_en_d = block_rescale_s0_en_d_tmp || op_group_3_result_valid_d;
// 2021/11/01 Jianlin Liang: block_rescale_prepare will be used to replace the original block_rescale
// block_rescale_prepare will finish the stage_0 of the original block_rescale while the rest part of
// block_rescale will be integrated in lp_aud_dma and hp_aud_dma.

block_rescale_prepare #(
    .W(W)
) block_rescale_prepare (
    .clk(clk),
    .rst_n(rst_n),

    .stage_0_start(start),
    .stage_0_enable(block_rescale_s0_en_d),
    .set_inf(set_inf_d),
    .in(result_rescale_data_in_d),
    .in_quant_bit(result_rescale_quant_bits_in_d),
    .quant_bits_max(quant_bits_max[5:0]),
    .out_unaligned(mac_internal_out),
    .out_quant_bit_unaligned(mac_internal_out_quant_bit),
    .out(s0_out[W-1:0]),
    .out_quant_bit(s0_out_quant_bit[5:0])
);
///////////////////////////////////////////////////////////////////////////
// Some part of DOT PRODUCT
// Other parts of DOT PRODUCT have been merged into complex mul and vector add
///////////////////////////////////////////////////////////////////////////

assign mac_add_start    = complex_mul_start_d[7:4];    // mac_add_start[0] is delayed 5 cycles compared to original complex_mul_start.
                                                      // 3 cycles for complex mul itself and 2 more cycles for rescale / block_rescale_prepare of mul result.
assign mac_add_in_save_en = complex_mul_start_d[4:3];


//-----------------------------------------------------------------
always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        mac_add_in1_real        <= #`FFD {W{1'b0}};
        mac_add_in1_real_qbit   <= #`FFD 6'b0;
    end
    else if (op_group_3 && mac_add_in_save_en[0]) begin
        mac_add_in1_real        <= #`FFD mac_internal_out;
        mac_add_in1_real_qbit   <= #`FFD mac_internal_out_quant_bit;
    end

//-----------------------------------------------------------------
always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        mac_add_in1_image       <= #`FFD {W{1'b0}};
        mac_add_in1_image_qbit  <= #`FFD 6'b0;
    end
    else if (op_group_3 && mac_add_in_save_en[1]) begin
        mac_add_in1_image       <= #`FFD mac_internal_out;
        mac_add_in1_image_qbit  <= #`FFD mac_internal_out_quant_bit;
    end

//-----------------------------------------------------------------
//-----------------------------------------------------------------
always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        mac_result_real        <= #`FFD {W{1'b0}};
        mac_result_real_qbit   <= #`FFD 6'b0;
    end
    else if (start) begin
        mac_result_real        <= #`FFD {W{1'b0}};
        mac_result_real_qbit   <= #`FFD 6'b0;
    end
    else if (op_group_3 && mac_add_start[2]) begin
        mac_result_real        <= #`FFD mac_internal_out;
        mac_result_real_qbit   <= #`FFD mac_internal_out_quant_bit;
    end

//-----------------------------------------------------------------
always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        mac_result_image       <= #`FFD {W{1'b0}};
        mac_result_image_qbit  <= #`FFD 6'b0;
    end
    else if (start) begin
        mac_result_image       <= #`FFD {W{1'b0}};
        mac_result_image_qbit  <= #`FFD 6'b0;
    end
    else if (op_group_3 && mac_add_start[3]) begin
        mac_result_image       <= #`FFD mac_internal_out;
        mac_result_image_qbit  <= #`FFD mac_internal_out_quant_bit;
    end

//-----------------------------------------------------------------
//-----------------------------------------------------------------
// output fifo
reg [W-1:0] result_fifo_din;
wire exp_busy;

wire v_intp_result_vld;
wire [W-1:0] v_intp_result;

always @(*)
begin
    //result_fifo_din = op_group_4 ? s0_out_d : s0_out;
    result_fifo_din = op_group_5 ? v_intp_result : s0_out;
    // case (op)
    //   OP_V_RECI, OP_V_LN, OP_V_EXP,
    //   OP_V_SQRT: result_fifo_din = s0_out;
    //   default: result_fifo_din = rescale_out;
    // endcase
end

wire [5:0] result_fifo_din_qbit;
//assign result_fifo_din_qbit = op_group_4 ? s0_out_quant_bit_d : s0_out_quant_bit;
assign result_fifo_din_qbit = op_group_5 ? in_quant_bits_mux : s0_out_quant_bit;

wire need_op2 = ((op == OP_V_ADD) || (op == OP_V_SUB) || (op == OP_V_MUL)) && !scalar_op;
wire asm_op_vld_rdy = op1_from_fifo_vld_rdy && (need_op2 ? (op2_from_fifo_vld && op2_from_fifo_rdy) : 1'b1);
reg [1:0] asm_op_vld_rdy_d;

wire result_fifo_din_vld =
    (op == OP_V_EXP) ? exp_result_valid_d :
    (op == OP_V_LN)  ? ln_result_valid_d :
    (op == OP_V_SQRT)? sqrt_result_valid_d :
    (op == OP_V_RECI)? reci_result_valid_d :
    op_group_3 ? dot_prod_result_fifo_din_vld :
    op_group_4 ? block_rescale_s0_en_d :
    op_group_5 ? v_intp_result_vld :
    op_group_1 ? complex_mul_result_valid_d2 : asm_op_vld_rdy_d[1];

wire result_fifo_din_rdy;
assign op1_from_fifo_rdy = result_fifo_din_rdy && ((op == OP_V_EXP) ? !exp_busy : 1);
assign op2_from_fifo_rdy = result_fifo_din_rdy && result_fifo_din_vld;

always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        asm_op_vld_rdy_d <= #`FFD 1'b0;
    end else begin
        asm_op_vld_rdy_d <= #`FFD {asm_op_vld_rdy_d[0], asm_op_vld_rdy};
    end

wire [2*W_BUF-1:0] result_from_out_fifo;
wire result_from_out_fifo_vld;
reg [2*W_BUF-1:0] result_from_out_fifo_d;

fifo_lto2 #(
    .W(W_BUF)
) result_fifo (
    .clk(clk),
    .rst_n(rst_n),
    .mode(1'b0),
    .din({{W_BUF{1'b0}}, result_fifo_din_qbit[5:0], result_fifo_din[W-1:0]}),
    .din_vld(result_fifo_din_vld),
    .din_rdy(result_fifo_din_rdy),
    .dout(result_from_out_fifo[2*W_BUF-1:0]),
    .dout_vld(result_from_out_fifo_vld),
    //-- dout_rdy(addr_en_ag1 && !rescaling)
    .dout_rdy(1'b1)
);

always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        result_from_out_fifo_d <= #`FFD {2*W_BUF{1'b0}};
    end else begin
        result_from_out_fifo_d <= #`FFD result_from_out_fifo;
    end
/// coefficient read logic
reg ham_win_tab_rdata_valid, ham_win_tab_rdata_valid_d;
if (REG)
begin
    // wire [7:0] ham_win_addr_0 = loop_d >> 1;
    // wire [6:0] ham_win_addr = ham_win_addr_0[7] ? 127 - ham_win_addr_0[6:0] : ham_win_addr_0[6:0];
    // assign tab_addr = ((vector_core_state_d == HW_CALC) ? ham_win_addr : loop) + tab_offset;
    // assign tab_en = (vector_core_state_d == HW_CALC) && !loop_d[0] || (vector_core_state_d == MEL_MAP);
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            ham_win_tab_rdata_valid <= #`FFD 1'b0;
        end else begin
            ham_win_tab_rdata_valid <= #`FFD (loop_d <= (vector_len>>1)) ? loop_d[0] : ~loop_d[0];
        end

end else begin
    // wire [7:0] ham_win_addr_0 = loop_d >> 1;
    // wire [6:0] ham_win_addr = ham_win_addr_0[7] ? 127 - ham_win_addr_0[6:0] : ham_win_addr_0[6:0];
    // assign tab_addr = ((vector_core_state_d == HW_CALC) ? ham_win_addr : loop) + tab_offset;
    // assign tab_en = (vector_core_state_d == HW_CALC) && !loop_d[0] || (vector_core_state_d == MEL_MAP);
    always_comb begin 
        ham_win_tab_rdata_valid =  (loop_d <= (vector_len>>1)) ? loop_d[0] : ~loop_d[0];        
    end
end

always @(posedge clk or negedge rst_n)
    if (!rst_n) begin
        ham_win_tab_rdata_valid_d <= #`FFD 1'b0;
    end else begin
        ham_win_tab_rdata_valid_d <= #`FFD ham_win_tab_rdata_valid;
    end

assign tab_en = addr_en_tab;
assign tab_addr = addr_tab;

///////////////////////////
///// pre-emphasis, y(n) = x(n) - k*x(n-1)
///////////////////////////
//////////////////////////
///// increased by jilan in 20231030
//////////////////////////

logic [W-1:0] prev_x_0;
logic [W-1:0] prev_x_1;
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        prev_x_0 <= #`FFD '0;
        prev_x_1 <= #`FFD '0;
    end else if (start_hamming_win && !use_prev_sample) begin
        prev_x_0 <= #`FFD ham_win_prev_sample[W-1:0];
        prev_x_1 <= #`FFD ham_win_prev_sample[W-1:0];
    end else if (rdata_valid_d && (op == OP_HAM_WIN)) begin
        if(reg_dsp_done) begin
            prev_x_1 <= #`FFD buf_rdata_d[W-1:0];
        end else begin
            prev_x_0 <= #`FFD buf_rdata_d[W-1:0];
        end
    end
end

logic signed [W+12:0] pre_emp_mul_result_sign;
logic [W+12:0] pre_emp_mul_result;
always_comb begin
    pre_emp_mul_result_sign = reg_dsp_done ? 
        $signed(prev_x_1) * $signed({1'b0, pre_emp_k}) : 
        $signed(prev_x_0) * $signed({1'b0, pre_emp_k}) ;
    pre_emp_mul_result  = $unsigned(pre_emp_mul_result_sign);
end

wire [W-1:0] pre_emp_out = buf_rdata_d[W-1:0] - pre_emp_mul_result[W+12:13]; // - {{W-1{1'b0}}, pre_emp_mul_result[12]};

reg [W-1:0] pre_emp_out_reg;
//reg __ham_win_en;
if (REG)
begin
    always @(posedge clk or negedge rst_n)
    if (!rst_n) 
        pre_emp_out_reg <= #`FFD {W{1'b0}};
    else 
        pre_emp_out_reg <= #`FFD pre_emp_out;
    //always @(posedge clk)
    //__ham_win_en <= #`FFD __pre_emp_en;
end else begin
    always @(*)
    begin
        pre_emp_out_reg = pre_emp_out;
        //always @(*)
        //__ham_win_en = __pre_emp_en;
    end
end

reg [2*W-1:0] odd_symm_tab_data;
reg [W-1:0] tab_type_1_data, tab_type_1_data_d;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        odd_symm_tab_data <= #`FFD {2*W{1'b0}};
    else if (tab_rdata_valid_d)
        odd_symm_tab_data <= #`FFD tab_rdata_d[2*W-1:0];

always @(*)
    if ((stage_tab_d == 0) || (stage_tab_d == 1))
        tab_type_1_data = tab_rdata_valid_d ? odd_symm_tab_data[2*W-1:W] : odd_symm_tab_data[W-1:0];
    else
        tab_type_1_data = tab_rdata_valid_d ? tab_rdata_d[2*W-1:W] : odd_symm_tab_data[W-1:0];

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        tab_type_1_data_d <= #`FFD {W{1'b0}};
    else
        tab_type_1_data_d <= #`FFD tab_type_1_data;

reg [W-1:0] mul_op_a;
reg [W:0] mul_op_b;
wire exp_mul_en;
wire [W-1:0] exp_mul_a;
wire [W:0] exp_mul_b;
wire [2*W:0] exp_mul_prod;
wire v_intp_mul_en;
wire [W-1:0] v_intp_mul_a;
wire [W:0] v_intp_mul_b;
wire [2*W:0] v_intp_mul_prod;
logic [W-1:0]abs_result_raw;
logic abs_result_overflow = abs_result[W-1];

always @(*)
begin
    mul_op_a = 0;
    mul_op_b = 0;
    abs_result_raw = 'd0;
    if (exp_mul_en)
    begin
        mul_op_a = exp_mul_a;
        mul_op_b = exp_mul_b;
    end else if (op == OP_HAM_WIN)
    begin
        mul_op_a = pre_emp_out_reg;
        mul_op_b = tab_type == 0 ? 
            (ham_win_tab_rdata_valid_d ? tab_rdata_d[2*W-1:W] : tab_rdata_d[W-1:0]) : 
            tab_type_1_data_d;
    end else if (op == OP_MFCC)
    begin
        abs_result_raw = abs_result;
        mul_op_a = abs_result_overflow ? {1'b0, abs_result[W-1:1]} : abs_result[W-1:0];
        mul_op_b = {1'b0, tab_rdata_d[2*W-1:W]};
    end else if (op == OP_V_MUL)
    begin
        mul_op_a = operand1;
        mul_op_b = {operand_2[W-1],operand_2};
    end else if (op == OP_V_SQR)
begin
    mul_op_a = operand1;
    mul_op_b = {operand1[W-1], operand1};
end else if (v_intp_mul_en)
begin
    mul_op_a = v_intp_mul_a;
    mul_op_b = v_intp_mul_b;
end
end
reg signed [2*W-1:0] mul_result_sign;

always @(*)
begin : mul_blk
    mul_result_sign = $signed(mul_op_a) * $signed(mul_op_b);
    mul_result = $unsigned(mul_result_sign);
end

assign v_intp_mul_prod = mul_result;
reg [2*W-1:0] mul_result_d;
if (REG)
begin
    always @(posedge clk or negedge rst_n)
    if (!rst_n) 
        mul_result_d <= #`FFD {(2*W){1'b0}};
    else 
        mul_result_d <= #`FFD mul_result;
end else begin
    always @(*)
        mul_result_d = mul_result;
end
wire [W-1:0] mul_result_quant = abs_result_overflow? mul_result[W+14:15] : mul_result[W+15:16];
reg [W-1:0] ham_win_result_quant;

always @(*)
begin
    // _ham_win_quant
    reg [2*W+9:0] t;
    // t = {{10{mul_result[2*W-1]}}, mul_result} >> (W-ham_win_prec_keep);
    t = {{10{mul_result_d[2*W-1]}}, mul_result_d} >> (W-ham_win_prec_keep);

    // if ((t[W+9]==0) && (t[W+8] != 'h0))  
    //     ham_win_result_quant = {1'b0, (W-1{'1'b1})};
    // else if ((t[W+9]==1) && (t[W+8] != 'h1ff)) 
    //     ham_win_result_quant = {1'b1, (W-1{'1'b1})};
    // else 
    //     ham_win_result_quant = {1'b1, (W-1{'1'b0})};

    //---------------------------------------------------------------------------
    // changed by jilan in 20231011
    if ((t[W+9]==0) && (t[W+8:W-1] != 'h0))
        ham_win_result_quant = {1'b0, (W-1{1'b1})};
    else if ((t[W+9]==1) && (t[W+8:W-1] != 10'h3ff))
        ham_win_result_quant = {1'b1, (W-1{1'b1})};
    else
        ham_win_result_quant = t[W-1:0];

end
assign exp_mul_prod = {mul_result[2*W-1], mul_result[2*W-1:0]};
// complex absolute
wire _complex_abs_en = (op == OP_ABS) || (op == OP_MFCC) && rdata_valid;
//wire _sqrt_en = (op == OP_V_SQRT) && op1_from_fifo_vld_rdy;
assign sqrt_result_valid = (op == OP_V_SQRT) ? 1 : (op == OP_V_SQR_DIFF) ? 2 : 0;

always @(posedge clk)
begin
    complex_abs_en_d <= {complex_abs_en_d[9:0], _complex_abs_en};
end

logic [1:0] abs_op = $unsigned((op == OP_V_SQRT) ? 1: (op == OP_V_SQR_DIFF) ? 2:0);
reg [W-1:0] sqrt_op1_norm_d;
reg [W-1:0] sqrt_op1_norm;
reg [5:0] sqrt_op1_norm_quant_bit;
reg [5:0] sqrt_op1_norm_quant_bit_d;
always @(*)
begin
    //sqrt_op1_adj
    reg [6:0] t;
    t = {in_quant_bits_mux[5], in_quant_bits_mux[5:0]} - {op1_norm_quant_bit[5], op1_norm_quant_bit};
    if (t[0])
    begin
        sqrt_op1_norm = {op1_norm[W-1], op1_norm[W-1:1]};
        t = t + 1;
    end else
    begin
        sqrt_op1_norm = op1_norm[W-1:0];
    end
    sqrt_op1_norm_quant_bit = t[6:1];
end

if (REG)
begin
    always @(posedge clk or negedge rst_n)
    if (!rst_n) 
    begin
        sqrt_op1_norm_d <= #`FFD {W{1'b0}};
        sqrt_op1_norm_quant_bit_d <= #`FFD 6'b0;
    end else 
    begin
        sqrt_op1_norm_d <= #`FFD sqrt_op1_norm;
        sqrt_op1_norm_quant_bit_d <= #`FFD sqrt_op1_norm_quant_bit;
    end
end else begin
    always @(*)
    begin
        sqrt_op1_norm_d = sqrt_op1_norm;
    end
end

wire [W-1:0] complex_abs_a_r = op == OP_V_SQRT ? sqrt_op1_norm_d : buf_rdata_d[W-1:0];
wire [W-1:0] complex_abs_a_i = op == OP_V_SQRT ? 0 : buf_rdata_d[2*W-1:W];
wire [5:0] complex_abs_a_quant = op == OP_V_SQRT ? sqrt_op1_norm_quant_bit_d : in_quant_bits_mux;

wire [W-1:0] abs_a_r = op_group_4 ? square_diff_in1 : complex_abs_a_r;
wire [W-1:0] abs_a_i = op_group_4 ? square_diff_in2 : complex_abs_a_i;
wire [5:0] abs_a_quant = op_group_4 ? square_diff_in_quant_bits : complex_abs_a_quant[5:0];

complex_abs #(
    .W(W),
    .REG(REG)
) abs (
    .clk(clk),
    .rst_n(rst_n),
    .op(abs_op),
    .a_r(abs_a_r),
    .a_i(abs_a_i),
    .square_diff_out(square_diff_out),         // output of square diff
    .square_diff_in_qbits_d2(square_diff_in_qbits_d2), // Square diff input quant bits delay for 2 cycles
    .o(abs_result[W-1:0]),
    .a_quant(abs_a_quant),
    .o_quant_bit(complex_abs_o_quant[5:0])
);
///////////////////////////////////////////////
// mel map
///////////////////////////////////////////////

wire [6:0] cur_fbank   = tab_rdata_d[6:0];
wire [6:0] next_fbank  = cur_fbank+1;
reg  [6:0] prev_cur_fbank;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        prev_cur_fbank <= #`FFD 7'h7e;
    else if (vector_core_state == VEC_IDLE)
        prev_cur_fbank <= #`FFD 7'h7e;
    else if (tab_rdata_valid_d)
        prev_cur_fbank <= #`FFD cur_fbank;

assign mel_map_end = eoag_tab;
wire new_fbank = rdata_valid && (cur_fbank != prev_cur_fbank);
//wire write_fbank = (new_fbank && (prev_cur_fbank > 0) && (prev_cur_fbank < 65) || mel_map_end_of_write);
wire write_fbank = (new_fbank && (prev_cur_fbank > 0) && (prev_cur_fbank < 65)) || pre_for_mel_map_last_write;

///////////////////////////////////////////////

reg [W-1:0] mul_result_quant_reg;
reg [W-1:0] mul_op_a_d;
reg new_fbank_d;
if (REG)
begin
    always @(posedge clk or negedge rst_n)
        if (!rst_n)
        begin
            mul_result_quant_reg <= #`FFD {W{1'b0}};
            mul_op_a_d           <= #`FFD {W{1'b0}};
            new_fbank_d          <= #`FFD 1'b0;
        end else
        begin
            mul_result_quant_reg <= #`FFD mul_result_quant;
            mul_op_a_d           <= #`FFD abs_result_raw;
            new_fbank_d          <= #`FFD new_fbank;
        end
end else
begin
    always @(*)
    begin
        mul_result_quant_reg = mul_result_quant;
        mul_op_a_d           = abs_result_raw;
        new_fbank_d          = new_fbank;
    end
end
//-------------------------------------------------------------------------------
//
reg [W+7:0] cur_acc;
reg [W+7:0] next_acc;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        cur_acc  <= #`FFD {W+7{1'b0}};
        next_acc <= #`FFD {W+7{1'b0}};
    end else if (new_fbank_d)
    begin
        cur_acc  <= #`FFD next_acc + mul_result_quant_reg;
        next_acc <= #`FFD mul_op_a_d - mul_result_quant_reg;
    end else
    begin
        cur_acc  <= #`FFD cur_acc + mul_result_quant_reg;
        next_acc <= #`FFD next_acc + mul_op_a_d - mul_result_quant_reg;
    end

reg write_fbank_d, write_fbank_d2, write_fbank_d3;
reg [8:0] addr_wr_fbank;
reg [8:0] addr_wr_fbank_d;
reg [8:0] addr_wr_fbank_d2;
if (REG)
begin
    always @(posedge clk or negedge rst_n)
        if (!rst_n)
        begin
            write_fbank_d   <= #`FFD 1'b0;
            write_fbank_d2  <= #`FFD 1'b0;
            write_fbank_d3  <= #`FFD 1'b0;
            addr_wr_fbank   <= #`FFD 9'h0;
            addr_wr_fbank_d <= #`FFD 9'h0;
            addr_wr_fbank_d2<= #`FFD 9'h0;
        end else
        begin
            // 保持原代码逻辑
    write_fbank_d   <= #`FFD write_fbank;
    write_fbank_d2  <= #`FFD write_fbank_d;
    write_fbank_d3  <= #`FFD write_fbank_d2;
    addr_wr_fbank   <= #`FFD prev_cur_fbank-1;
    addr_wr_fbank_d <= #`FFD addr_wr_fbank;
    addr_wr_fbank_d2<= #`FFD addr_wr_fbank_d;
end

always @(*)
begin
    //mel_map_end_of_write = ~complex_abs_en_d[6] & complex_abs_en_d[7];
    //mel_map_end_of_write = ~complex_abs_en_d[7] & complex_abs_en_d[8];
    mel_map_end_of_write = ~complex_abs_en_d[9] & complex_abs_en_d[10];
    pre_for_mel_map_last_write = ~complex_abs_en_d[7] & complex_abs_en_d[8];
end

end else
begin
    always @(*)
    begin
        write_fbank_d   = write_fbank;
        write_fbank_d2  = write_fbank;
        write_fbank_d3  = write_fbank;
        addr_wr_fbank   = prev_cur_fbank-1;
        addr_wr_fbank_d = prev_cur_fbank-1;
        addr_wr_fbank_d2= prev_cur_fbank-1;
    end

    always @(*)
    begin
        mel_map_end_of_write = ~__complex_abs_en & complex_abs_en_d[0];
        pre_for_mel_map_last_write = ~__complex_abs_en & complex_abs_en_d[0];
    end
end

wire __ln_en = op1_from_fifo_vld_rdy && (op == OP_V_LN);
assign ln_result_valid = (REG ? op1_from_fifo_vld_rdy_d[1] : op1_from_fifo_vld_rdy) && (op == OP_V_LN);
wire [5:0] mel_map_quant_out = in_quant_bits_mux;
// exponent

wire exp_start = op1_from_fifo_vld_rdy && (op == OP_V_EXP) && !exp_busy;
wire _exp_en = exp_start;

// assign exp_result_valid = op1_from_fifo_vld_rdy_d[4] && (op == OP_V_EXP);
reg [W-1:0] exp_in_save;

reg [5:0] exp_in_quant_bits_save;

wire [W-1:0] exp_in = exp_start ? op1_from_fifo : exp_in_save;

wire [5:0] exp_in_quant_bits = exp_start ? in_quant_bits_mux : exp_in_quant_bits_save;

always @(posedge clk or negedge rst_n)
if (!rst_n)
begin
    exp_in_save <= #`FFD 0;
    exp_in_quant_bits_save <= #`FFD 0;
end 
else if (exp_start)
begin
    exp_in_save <= #`FFD op1_from_fifo;
    exp_in_quant_bits_save <= #`FFD in_quant_bits_mux;
end

exp #(
    .W(W)
) exp (
    .clk(clk),
    .rst_n(rst_n),

    .start(exp_start),
    .busy(exp_busy),

    .in(exp_in[W-1:0]),
    .in_quant_bits(exp_in_quant_bits[5:0]),

    .out(exp_out[W-1:0]),
    .out_quant_bits(exp_out_quant_bits[5:0]),
    .out_valid(exp_result_valid),

    .mul_en(exp_mul_en),
    .mul_a(exp_mul_a[W-1:0]),
    .mul_b(exp_mul_b[W:0]),
    .mul_prod(exp_mul_prod[2*W:0])
);
///////////////////////////////////////////////////////
// reciprocal
///////////////////////////////////////////////////////

wire reci_en = op1_from_fifo_vld_rdy && op_group_2;
assign reci_result_valid = REG ? op1_from_fifo_vld_rdy_d[3] && op_group_2 : reci_en;
wire __reci_en = reci_en;

signed_norm #(
    .W(W)
) reci_signed_norm (
    .a(operand_1[W-1:0]),
    .o(op1_norm[W-1:0]),
    .quant_bit(op1_norm_quant_bit[5:0])
);

reg [23:0] op1_norm_24b;
// reg [5:0] op1_norm_quant
logic [23:0] reci_result_24b;
reg [6:0] reci_in_quant_bits_d;

wire [6:0] reci_in_quant_bits = {in_quant_bits_mux[5], in_quant_bits_mux} - {op1_norm_quant_bit[5], op1_norm_quant_bit};

// if (W<24)
// begin
if (REG)
begin
    always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        op1_norm_24b <= #'FFD 24'h0;
        // op1_norm_quant_bit_d <= #'FFD 6'h0;
        reci_in_quant_bits_d <= #'FFD 7'h0;
    end 
    else 
    begin
        op1_norm_24b <= #'FFD {op1_norm[W-1:0], {(24-W){1'b0}}};
        // op1_norm_quant_bit_d <= #'FFD op1_norm_quant_bit;
        reci_in_quant_bits_d <= #'FFD reci_in_quant_bits;
    end
end 
else 
begin
    always @(*)
    begin
        op1_norm_24b = {op1_norm[W-1:0], {(24-W){1'b0}}};
        // op1_norm_quant_bit = op1_norm_quant_bit;
        reci_in_quant_bits_d = reci_in_quant_bits;
    end
end

reg [6:0] reci_out_quant_bits_tmp;


reciprocal #(
    .REG(REG)
) reciprocal (
    .clk(clk),
    .rst_n(rst_n),

    .a(op1_norm_24b[23:0]),    // signed, Q23
    .a_quant_bit(reci_in_quant_bits_d[6:0]),  // signed, -64 ~ 63
    .o(reci_result_24b[23:0]),
    .quant_bit(reci_out_quant_bits_tmp[6:0]),  // signed, -64 ~ 63
    .zero_in(zero_in)
);

wire [W:0] reci_result_tmp;
generate
if (W==24)
    assign reci_result_tmp = {reci_result_24b[23:24-W], 1'b0};
else
    assign reci_result_tmp = {reci_result_24b[23:23-W]};
endgenerate

assign reci_result = (reci_result_24b[23:22] == 2'b11) ||
                     (reci_result_24b[23:22] == 2'b00) ? reci_result_tmp : (reci_result_tmp>>1);
assign reci_out_quant_bits = (reci_result_24b[23:22] == 2'b11) ||
                             (reci_result_24b[23:22] == 2'b00) ? reci_out_quant_bits_tmp - 1 : reci_out_quant_bits_tmp;
// Linear interpolation
///////////////////////////////////////////////////////////////////////////////

wire [W-1:0] v_intp_k_pos;

assign v_intp_k_pos = complex_image_dat;  // This is

linear_interpolation #(
    .W(W)
) linear_interpolation (
    .clk(clk),
    .rst_n(rst_n),

    .start_v_intp(start_v_intp),
    .k_pos(v_intp_k_pos),
    .op_from_fifo_vld(op1_from_fifo_vld),
    .is_real(is_real),
    .op_from_fifo(op1_from_fifo),
    .result_vld(v_intp_result_vld),
    .result(v_intp_result),

    .mul_en(v_intp_mul_en),
    .mul_a(v_intp_mul_a),
    .mul_b(v_intp_mul_b),
    .mul_prod(v_intp_mul_prod)
);

// buffer write
///////////////////////////////////////////////////////////////////////////////

reg [8:0] write_addr;
reg [1:0] buf_wbe;
reg [2*W_BUF-1:0] buf_wdata;

wire [W-1:0] ln_out_adjusted;
wire [5:0] ln_out_quant_bit_adjusted;

assign ln_out_adjusted = ln_out >>> (LN_OUT_QUANT - mfcc_out_prec);  
    // Used when (op == OP_MFCC) or (op == OP_LN)
assign ln_out_quant_bit_adjusted = $unsigned(W - 1 - mfcc_out_prec);  
    // Used when (op == OP_MFCC) or (op == OP_LN)

always @(*)
begin
    write_en = 0;
    write_addr = 0;
    buf_wbe = 2'b00;
    buf_wdata = 0;

    case (op)
        OP_ABS:
        begin
            write_en = addr_en_ag1;
            write_addr = addr_ag1;
            //buf_wdata = {{W{1'b0}}, abs_result};
            buf_wdata = {{W_BUF{1'b0}}, complex_abs_o_quant[5:0], abs_result};
            buf_wbe = 2'b01;
        end

        OP_HAM_WIN,
        OP_HAM_WIN_1:
        begin
            write_en = addr_en_ag1;
            write_addr = addr_ag1;
            //buf_wdata = (op == OP_HAM_WIN) ? {{W{1'b0}}, ham_win_result_quant, {W{1'b0}}} : 
            // ham_win_result_quant;
            // The quant bits for HAM_WIN may be a don’t care value here. In real calculation, 
            // out_quant_bits may be used for this process.
            buf_wdata = (op == OP_HAM_WIN) ? {{W_BUF{1'b0}}, 
                        out_quant_bits, ham_win_result_quant}: {out_quant_bits, ham_win_result_quant,{W_BUF{1'b0}}};
            buf_wbe = 2'b11; //(op == OP_HAM_WIN) ? 2'b01 : 2'b10;
        end

        OP_MFCC:
        begin
            write_en = write_fbank_d3;
            write_addr = addr_wr_fbank_d2;
            //buf_wdata = ln_out >>> (LN_OUT_QUANT - mfcc_out_prec);
            buf_wdata = {{W_BUF{1'b0}}, ln_out_quant_bit_adjusted, ln_out_adjusted};
            buf_wbe = 2'b11;
        end

        OP_LN:
        begin
            write_en = rdata_valid;
            write_addr = loop_d;
            //buf_wdata = ln_out >>> (LN_OUT_QUANT - mfcc_out_prec);
            buf_wdata = {{W_BUF{1'b0}}, ln_out_quant_bit_adjusted, ln_out_adjusted};
            buf_wbe = 2'b01;
        end
    OP_V_ADD,
    OP_V_SUB,
    OP_V_SQR,
    OP_V_RECI,
    OP_V_SQRT,
    OP_V_EXP,
    OP_V_MUL,
    OP_V_DOT,
    OP_V_SQR_DIFF,
    OP_V_INTP:
    begin
        write_en = addr_en_ag1;
        write_addr = addr_ag1;
        buf_wdata = result_from_out_fifo;

        buf_wbe = 2'b11;
    end

    OP_V_LN:
    begin
        write_en = addr_en_ag1;
        write_addr = addr_ag1;

        // Need to pipeline output for 1 more cycle to avoid reading / writing to the same buf at the same time
        buf_wdata = result_from_out_fifo_d;

        buf_wbe = 2'b11;
    end
endcase
end


always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        rdata_valid    <= #`FFD 0;
        rdata_valid_d  <= #`FFD 0;
    end else
    begin
        rdata_valid    <= #`FFD read_en;
        rdata_valid_d  <= #`FFD rdata_valid;
    end

wire read_en_op2 = addr_en_op2;

assign buf_addr_a = buf_we_a ? write_addr : 
                   (!op2_in_buf_sel && read_en_op2) ? addr_op2 : read_addr;
//-- assign buf_en_a = ((rescaling ? read_from_a : read_from_a) && read_en) || (!op2_in_buf_sel && read_en_op2) || buf_we_a;
assign buf_en_a = (read_from_a && read_en) || (!op2_in_buf_sel && read_en_op2) || buf_we_a;

assign buf_we_a = !write_to_b && write_en;
assign buf_wbe_a = buf_wbe;
assign buf_wdata_a_full = buf_wdata;

assign buf_addr_b = buf_we_b ? write_addr : 
                   (op2_in_buf_sel && read_en_op2) ? addr_op2 : read_addr;
//-- assign buf_en_b = ((rescaling ? read_from_a : !read_from_a) && read_en) || (op2_in_buf_sel && read_en_op2) || buf_we_b;
assign buf_en_b = ((!read_from_a) && read_en) || (op2_in_buf_sel && read_en_op2) || buf_we_b;

assign buf_we_b = write_to_b && write_en;
assign buf_wbe_b = buf_wbe;
assign buf_wdata_b_full = buf_wdata;

assign result_in = read_from_a;

wire rescale_op = op_group_0 || op_group_1 || op_group_3 || op_group_4 ||
                  (op == OP_V_RECI) || (op == OP_V_SQRT) || (op == OP_V_EXP);

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        out_quant_bits <= #`FFD 6'b0;
    else if (start_d[2])
        out_quant_bits <= #`FFD (op == OP_LN) || (op == OP_MFCC) ?
                            ($unsigned(W)-$unsigned(1)-mfcc_out_prec) :
                          (op == OP_V_LN) ?
                            $unsigned(W-1-LN_OUT_QUANT) :
                          (op == OP_HAM_WIN) ?
                            {in_quant_bits_mux_0[5], in_quant_bits_mux_0} - {2'h0, ham_win_prec_keep} :
                            {in_quant_bits_mux[5], in_quant_bits_mux};
    else if (op_group_5)
        out_quant_bits <= #`FFD in_quant_bits_mux_0;
    else if (rescale_op && (vector_core_state == VEC_WAIT_LAST_WR))
        out_quant_bits <= #`FFD quant_bits_mux;
endmodule
module addr_gen #(
    parameter AW=9
)
(
    input clk,
    input rst_n,
    input start,
    input [1:0] addr_type, // addr update type, should always be valid, 0: {increment};
    input [AW-1:0] addr_start,
    input [AW-1:0] addr_incr,
    input [3:0] addr_incr_interval,
    input [8:0] vector_len, // vector length - 1
    output reg addr_en,
    output reg [AW-1:0] addr,
    output reg addr_en_d,
    output reg end_of_addr_gen,
    output reg [1:0] stage
);

// synopsys translate_off
// assign (weak0, weak1) addr_type=0;
// synopsys translate_on

reg [3:0] addr_incr_cnt;
reg [8:0] vector_len_cnt;
reg enable;
wire addr_update_en = enable && (addr_incr_cnt == addr_incr_interval);
always @(posedge clk or negedge rst_n)
    if (!rst_n)
        begin
            addr <= #`FFD {AW{1'h0}};
        end
    else if (start)
        begin
            addr <= #`FFD addr_start;
        end
    else if (enable && addr_update_en)
        begin
            if (vector_len_cnt != vector_len)
                if (stage != 2'h0)
                    addr <= #`FFD addr - addr_incr;
                else
                    addr <= #`FFD addr + addr_incr;
            else if (addr_type == 2)
                addr <= #`FFD addr + addr_incr;
        end

// always @(*)
//    addr = addr_start + vector_len_cnt;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        addr_incr_cnt <= #`FFD 4'h0;
    else if (start || enable && addr_update_en)
        addr_incr_cnt <= #`FFD 4'h0;
    else if (enable)
        addr_incr_cnt <= #`FFD addr_incr_cnt + 4'h1;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        enable <= #`FFD 1'b0;
    else if (start)
        enable <= #`FFD 1'b1;
    else if (addr_update_en && (vector_len_cnt == vector_len) && ((addr_type == 0) || (stage == 2'h2)))
        enable <= #`FFD 1'b0;
always @(posedge clk or negedge rst_n)
    if (!rst_n)
        stage <= #`FFD 2'b0;
    else if (start)
        stage <= #`FFD 2'h0;
    else if (addr_update_en)
        case (stage)
            2'h0:
                if (vector_len_cnt == vector_len)
                    if (addr_type == 1)
                        stage <= #`FFD 2'h2;
                    else if (addr_type == 2'h2)
                        stage <= #`FFD 2'h1;
            2'h1: stage <= #`FFD 2'h2;
            2'h2:
                if (vector_len_cnt == vector_len)
                    stage <= #`FFD 2'h0;
        endcase

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        vector_len_cnt <= #`FFD 9'h0;
    else if (start || addr_update_en && (vector_len_cnt == vector_len) || (stage == 2'h1))
        vector_len_cnt <= #`FFD 9'h0;
    else if (enable && addr_update_en)
        vector_len_cnt <= #`FFD vector_len_cnt + 9'h1;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
        addr_en_d <= #`FFD 1'b0;
    else
        addr_en_d <= #`FFD addr_en;

always @(*)
    addr_en = enable && (addr_incr_cnt == 0);

assign end_of_addr_gen = addr_update_en && (vector_len_cnt == vector_len);

endmodule
module fifo_1to2 #(
    parameter W=16
)
(
    input clk,
    input rst_n,

    input mode, // 0: 1to2 mode; 1: 2to2 mode
    input [2*W-1:0] din,
    input din_vld,
    output din_rdy,

    output [2*W-1:0] dout,
    output dout_vld,
    input dout_rdy
);

reg [2*W-1:0] mem;
reg mem_vld;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        mem <= #`FFD {2*W{1'b0}};
        mem_vld <= #`FFD 1'b0;
    end
    else if (dout_rdy && dout_vld)
        mem_vld <= #`FFD mode ? din_vld && din_rdy : 0;
    else if (din_vld && din_rdy)
    begin
        mem <= #`FFD din;
        mem_vld <= #`FFD 1'b1;
    end

assign dout_vld = mode ? din_vld || mem_vld : din_vld && mem_vld;
assign dout = mode ? (mem_vld ? mem[2*W-1:0] : din[2*W-1:0]):{din[W-1:0],mem[W-1:0]};
assign din_rdy = mem_vld && dout_rdy || !mem_vld;

endmodule
module fifo_2to1 #(
    parameter W=16,
    parameter D=2
)
(
    input clk,
    input rst_n,

    input flush,
    input [2*W-1:0] din,
    input din_vld,
    output din_rdy,

    output [W-1:0] dout,
    output dout_vld,
    input dout_rdy
);

localparam ND=2<<D;
localparam PTRDEF={D+1{1'b0}};

reg [W-1:0] mem[ND];
reg [D:0] wptr;
reg [D:0] rptr;
wire [D:0] ptr_diff = wptr - rptr;
integer i;

always @(posedge clk or negedge rst_n)
    if (!rst_n)
    begin
        for (i=0; i<ND; i=i+1)
            mem[i] <= #`FFD {W{1'b0}};
        wptr <= #`FFD PTRDEF;
    end 
    else if (flush)
    begin
        for (i=0; i<ND; i=i+1)
            mem[i] <= #`FFD {W{1'b0}};
        wptr <= #`FFD PTRDEF;
    end 
    else if (din_vld && din_rdy)
    begin
        if (dout_rdy && (ptr_diff == 0))
        begin
            mem[wptr[D-1:0]] <= #`FFD din[2*W-1:W];
            wptr <= #`FFD wptr + 1;
        end 
        else
        begin : inc2_blk
            reg [D-1:0] wptr_inc;
            wptr_inc = wptr + 1;
            mem[wptr[D-1:0]] <= #`FFD din[W-1:0];
            mem[wptr_inc[D-1:0]] <= #`FFD din[2*W-1:W];
            wptr <= #`FFD wptr + 2;
        end
    end
always @(posedge clk or negedge rst_n)
    if (!rst_n)
        rptr <= #`FFD PTRDEF;
    else if (flush)
        rptr <= #`FFD PTRDEF;
    else if (dout_vld && dout_rdy && (ptr_diff != 0))
        rptr <= #`FFD rptr + 3'h1;

assign dout    = (ptr_diff != 0) ? mem[rptr[1:0]] : din[W-1:0];
assign dout_vld = (ptr_diff != 0) || din_vld;
assign din_rdy  = (ptr_diff <= 2);

endmodule

