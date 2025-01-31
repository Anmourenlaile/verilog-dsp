// *********************************************************************************
// Project Name : zkx2024
// Author       : Jlan
// Email        : 15533610762@163.com
// Create Time  : 2024-07-31
// File Name    : fft_core.v
// Module Name  : fft_core
// Called By    : jlan
// Abstract     :
//
// 
// *********************************************************************************
// Modification History:
// Date         By              Version                 Change Description
// -----------------------------------------------------------------------
// 2024-07-31    Macro           1.0                     Original
//  
// *********************************************************************************

`ifndef FFT_CORE
`define FFT_CORE

module fft_core#(
    parameter W = 16,
    parameter W_BUF = 22, // = W+6
    parameter REG = 0 //PIPELINE FOR BUTTERFLY
)(
    input   clk,
    input   rst_n,

    input   start,
    output  logic   [0:0]   busy,
    input   ifft_en,// 0:FFT, 1:IFFT
    input   in_buf_sel_mode,//0:use the previos result; 1:set by in_buf_sel
    input   prev_in_buf_sel, // 0:from buffer a ,1;from  bufb
    input   [8:0]   in_buf_offset,
    input   [8:0]   out_buf_offset,
    input   [0:0]   quant_input_en, // quantify 1 bit befrist round , 0:disable 1:enable
    input   [5:0]   prev_in_quant_bits,
    input   [5:0]   in_quant_bits,

    input   [3:0]   fft_n,// calculate 2^fft_n points FFT

    output  logic   [9:0]   tab_addr,
    output  logic   [0:0]   tab_en,
    input   [2*W-1:0]tab_rdata,

    output  logic   [8:0]   buf_addr_a,
    output  logic   [0:0]   buf_en_a,
    output  logic   [0:0]   buf_we_a,
    output  logic   [2*W_BUF-1:0]   buf_wdata_a_full,
    input   [2*W_BUF-1:0]   buf_rdata_a_full,
    
    
    output  logic   [8:0]   buf_addr_b,
    output  logic   [0:0]   buf_en_b,
    output  logic   [0:0]   buf_we_b,
    output  logic   [2*W_BUF-1:0]   buf_wdata_b_full,
    input   [2*W_BUF-1:0]   buf_rdata_b_full,

    output  logic   [5:0]   out_quant_bits,
    output  logic   [0:0]   result_in,

    input   complex_mul_en,
    input   complex_mul_start,
    input   [W-1:0] complex_mul_a_r,
    input   [W-1:0] complex_mul_a_i,
    input   [W-1:0] complex_mul_b_r,
    input   [W-1:0] complex_mul_b_i,
    output  [2*W-1:0]   complex_mul_o_r,
    output  [2*W-1:0]   complex_mul_o_i

);

//clock_gating
logic clk_gate_en = start | busy;
logic gated_clk;

clk_gate_r1p0 fft_clk_gate(
    .CKOUT(gated_clk),
    .CKIN(clk),
    .EN(clk_gate_en)
);

//clock_gating

//---------------------------------------------------------------------------------------------------------------------------------------------
logic   [2*W-1:0]   buf_wdata_a;    //Internal buf_wdata_a without  exp part
logic   [2*W-1:0]   buf_rdata_a;    //Internal buf_rdata_a without  exp part\

assign  buf_rdata_a =   {buf_rdata_a_full[W_BUF+W-1:W_BUF],buf_rdata_a_full[W-1:0]};
assign  buf_wdata_a_full =  {
    out_quant_bits,buf_wdata_a[2*W-1:W],
    out_quant_bits,buf_wdata_a[  W-1:0]
};

logic   [2*W-1:0]   buf_wdata_b;    //Internal buf_wdata_b without  exp part
logic   [2*W-1:0]   buf_rdata_b;    //Internal buf_rdata_b without  exp part\

assign  buf_rdata_b =   {buf_rdata_b_full[W_BUF+W-1:W_BUF],buf_rdata_b_full[W-1:0]};
assign  buf_wdata_b_full =  {
    out_quant_bits,buf_wdata_b[2*W-1:W],
    out_quant_bits,buf_wdata_b[  W-1:0]
};
//---------------------------------------------------------------------------------------------------------------------------------------------
logic   buf_rdata_a_valid,buf_rdata_b_valid,tab_rdata_valid;
logic   [2*W-1:0]   buf_rdata_a_d,buf_rdata_b_d,tab_rdata_d;

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        buf_rdata_a_valid   <= '0;
        buf_rdata_b_valid   <= '0;
        tab_rdata_valid     <= '0;
    end
    else begin
        buf_rdata_a_valid   <= buf_en_a && (!buf_we_a);
        buf_rdata_b_valid   <= buf_en_b && (!buf_we_b);
        tab_rdata_valid     <= tab_en;
    end
end

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        buf_rdata_a_d   <= '0;
        buf_rdata_b_d   <= '0;
        tab_rdata_d     <= '0;
    end
    else begin
        if(buf_rdata_a_valid)
            buf_rdata_a_d   <=  buf_rdata_a;
        
        if(buf_rdata_b_valid)
            buf_rdata_b_d   <=  buf_rdata_b;
        
        if(tab_rdata_valid)
            tab_rdata_d     <=  tab_rdata;
    end
end
//---------------------------------------------------------------------------------------------------------------------------------------------
typedef enum  { 
    IDLE,
    LOOP,
    LOOP_WAIT,
    BIT_REV0,
    BIT_REV1,
    LAST_WAIT
} fft_state_t;

fft_state_t fft_state,fft_state_next;

logic   [3:0]   loop0;  //loop0 0 from fft_n-1: the signal means the number of interations for FFT.
logic   [7:0]   loop1;  //loop1: the signal mean the number of group in a certain round of loop0.
logic   [7:0]   loop2;  //loop2: the signal mean the number of calculate butterfly in a certain round of loop1.

logic   [3:0]   loop0_d;
logic   [7:0]   loop2_d;

logic   [3:0]   loop0_d2;
logic   [7:0]   loop2_d2;

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        loop0_d      <=  '0;
        loop0_d2     <=  '0;
        loop2_d      <=  '0;
        loop2_d2     <=  '0;
    end
    else begin
        loop0_d      <=  loop0;
        loop0_d2     <=  loop0_d;
        loop2_d      <=  loop2;
        loop2_d2     <=  loop2_d;
    end
end

//---------------------------------------------------------------------------------------------------------------------------------------------
//FFT interations control.
logic   [3:0]   loop0_end_th    =   fft_n-1;
logic   [7:0]   loop1_end_th    =   $unsigned((1<<loop0)-1);//there are 2^loop0-1 groups in the loop0th round.
logic   [7:0]   loop2_end_th    =   $unsigned((1<<(loop0_end_th-loop0))-1);//there are 2^(loop0_end_th-loop0)-1 calculations in the loop1th group.

logic   loop0_end   =   loop0 == loop0_end_th;
logic   loop1_end   =   loop1 == loop1_end_th;
logic   loop2_end   =   loop2 == loop2_end_th;
logic   loop_all    =   loop0 == fft_n;//bit_rev need one round!!!

logic           read_2nd;///??????????  complex read/write need 2 cycle? 
logic   [5:0]   read_2nd_d;///??????????

logic   end_of_one_round_read   =   loop1_end && loop2_end &&read_2nd;

logic   end_of_one_round_write;

logic   bit_rev_end =  loop2 == $unsigned((1<<(fft_n-1))-1);////????????????????? 255

logic   [4:0]   end_of_one_round_read_d;
always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)
        end_of_one_round_read_d <=  '0;
    else 
        end_of_one_round_read_d <=  {end_of_one_round_read_d[3:0],end_of_one_round_read};
end

assign  end_of_one_round_write  =  REG ? end_of_one_round_read_d[4] : end_of_one_round_read_d[3];
//???? the write is late read 4cycles. The calculation&write need 4cycles!!!!!!

//FFT_FSM
logic busy_next
always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)
        fft_state   <= IDLE;
    else
        fft_state   <=  fft_state_next;
end

always_comb begin
    fft_state_next  =   fft_state;
    busy_next       =   busy;
    case(fft_state)
    IDLE:
        if(start)begin
            fft_state_next  = LOOP;
            busy_next       = 1'b1;
        end
    LOOP:
        if(loop1_end && loop2_end &&read_2nd)
        fft_state_next  =   LOOP_WAIT; 
    LOOP_WAIT:
        if(end_of_one_round_write)
            if(loop_all)
                fft_state_next  = BIT_REV0;
            else
                fft_state_next  = LOOP;
    BIT_REV0:
        if(bit_rev_end)
            fft_state_next  =  BIT_REV1;
    BIT_REV1://why does fsm need 2 state to rev?
        if(bit_rev_end)
            fft_state_next  = LAST_WAIT;
    LAST_WAIT:
        begin
            fft_state_next  = IDLE;
            busy_next       = 1'b0;
        end
    endcase
end
always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        busy    <= '0;
    end
    else
        busy    <= busy_next;
end

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
       loop0    <= '0; 
    end
    else if(fft_state == LOOP && start)
        loop0   <= '0;
    else if((fft_state == LOOP) && loop1_end && loop2_end && read_2nd)
        loop0   <= loop0 + 1'b1;
end

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        loop1   <= '0;
    end
    else if((fft_state == ILDE) && start)begin
        loop1   <= '0;
    end
    else if((fft_state == LOOP) && loop2_end && read_2nd)begin
        if(loop1_end)
            loop1   <= '0;
        else 
            loop1   <= loop1 + 1'b1;
    end
end

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin 
        loop2   <= '0;
    end
    else if((fft_state == IDLE) && start)
        loop2   <= '0;
    else if((fft_state == LOOP) && read_2nd)begin
        if(loop2_end)
            loop2   <= '0;
        else
            loop2   <=  loop2 + 1'b1;
    end
    else if((fft_state == BIT_REV0) || (fft_state == BIT_REV1))begin
        if(bit_rev_end)
            loop2   <= '0;
        else
            loop2   <= loop2 + 1'b1;
    end
end

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        read_2nd    <= '0;
    end
    else if((fft_state == IDLE) && start)
        read_2nd    <= '0;
    else if((fft_state == LOOP))
        read_2nd   <= ~read_2nd; 
end

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        read_2nd_d  <= '0;
    end
    else begin
        read_2nd_d  <= {read_2nd_d[4:0],read_2nd};//?????
    end
end
//---------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------
//butterfly computation
logic   [W-1:0]bf_a_r;
logic   [W-1:0]bf_a_i;
logic   [W-1:0]bf_b_r;
logic   [W-1:0]bf_b_i;
logic   [W:0]  w_r;//result
logic   [W:0]  w_i;//result
logic   [W-1:0]p_r;//result
logic   [W-1:0]p_i;//result
logic   [W-1:0]q_r;//result
logic   [W-1:0]q_i;//result
logic   buf_read_en;
logic   [5:0]   buf_read_en_d;
logic   bf_start;
//---------------------------------------------------------------------------------------------------------------------------------------------
logic   [2:0] bf_start_d;
if(REG)
begin
    always_ff@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            bf_start_d  <= '0;
        end
        else 
            bf_start_d  <= {bf_start_d[1:0],bf_start};
    end
end
else begin
    always_ff@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            bf_start_d <=  '0;
        end
        else 
            bf_start_d <= {(3){bf_start}};
    end
end
//---------------------------------------------------------------------------------------------------------------------------------------------
logic quant_full;
logic quant_next_round;

butterfly #(
    .W(W),
    .REG(REG)
)  butterfly(
    .clk(clk),
    .rst_n(rst_n),
    .start(start),

    .a_r(bf_a_r),
    .a_i(bf_a_i),
    .b_r(bf_b_r),
    .b_i(bf_b_r),
    .w_r(w_r),// COS/SIN TABLE
    .w_i(w_i),// COS/SIN TABLE
    .p_r(p_r),
    .p_i(p_i),
    .q_r(q_r),
    .q_i(q_i),
    .quant_full(quant_full),

    .complex_mul_en(complex_mul_en),
    .complex_mul_start(complex_mul_start),
    .complex_mul_a_r(complex_mul_a_r),
    .complex_mul_a_i(complex_mul_a_i),
    .complex_mul_b_r(complex_mul_b_r),
    .complex_mul_b_i(complex_mul_b_i),
    .complex_mul_o_r(complex_mul_o_r),
    .complex_mul_o_i(complex_mul_o_i)
);
//---------------------------------------------------------------------------------------------------------------------------------------------
//out_quant_bits control :quant_full -> data left shifter
logic   quant_current_round; // quant full -> quant + 1'b1?

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        quant_next_round <= '0;
    end
    else if((fft_state == ILDE ))
        quant_next_round <= '0;
    else if(bf_start_d[2] && quant_full)
        quant_next_round <= 1'b1;
    else if ((fft_state == LOOP_WAIT) && end_of_one_round_write)
        quant_next_round <= 1'b0;
end

logic [5:0] in_quant_bits_mux ;
assign in_quant_bits_mux = in_buf_sel_mode ? in_quant_bits : prev_in_quant_bits;

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
       out_quant_bits   <= '0;
       quant_current_round  <= '0;
    end
    else if((fft_state == IDLE  )&& start)begin
        out_quant_bits  <= in_quant_bits_mux + (quant_input_en ? 6'b1 : 6'b0);
        quant_current_round <= quant_input_en;
    end
    else if( (fft_state == LOOP_WAIT) && end_of_one_round_write)
    begin
        if(loop_all && ifft_en)
            out_quant_bits  <= out_quant_bits - fft_n;
        if(quant_next_round && !loop_all)
            out_quant_bits <= out_quant_bits + 6'b1;
    
        quant_current_round <= quant_next_round; 
    end
end
//---------------------------------------------------------------------------------------------------------------------------------------------
// read cos/sin table from tab_buf 512FFT ->  256 factor -> 256/4(4-quadrant)=64

logic   w_zero,w_m90;
logic   [1:0]   w_sector;
logic   [7:0]   w_idx = ((loop2_d[7:0]) << (loop0_d + 9 - fft_n));//why use d not d2?
logic   [6:0]   a = (w_idx[6] ? 128 - w_idx[6:0] : w_idx);
logic   [W:0]   w_cos_t,w_sin_t;

logic   [7:0]   w_idx_d;

always_comb begin
    if(w_zero)begin
        w_r = $unsigned(1<<(W-1));
        w_i = 0;
    end
    else if (w_m90)begin
        w_r = 0;
        w_i = $unsigned(ifft_en ? (1<<(W-1)): -(1<<(W-1)));
    end else
    begin
        case(w_sector)
        2'b00:begin
            w_r = w_cos_t;
            w_i = ifft_en ? w_sin_t : -w_sin_t;
        end;
        2'b01:begin
            w_r = w_sin_t;
            w_i = ifft_en ? w_cos_t : -w_cos_t;
        end;
        2'b10:begin
            w_r = -w_sin_t;
            w_i = ifft_en ? w_cos_t : -w_cos_t;
        end;
        2'b11:begin
            w_r = -w_cos_t;
            w_i = ifft_en ? w_sin_t : -w_sin_t;
        end;
        endcase
        w_r = w_r;
        w_i = w_i;
    end
end

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        w_zero      = '0;
        w_m90       = '0;
        w_sector    = '0;
        w_idx_d     = '0;
    end
    else begin
        w_zero    <= w_idx_d == 0;
        w_m90     <= w_idx_d == 8'h80;
        w_secos_t <= w_idx_d[7:6]; //quadrant of rotation of factor
        w_idx_d   <= w_idx;
    end
end

assign w_sin_t = tab_rdata_d[W-1:0];
assign w_cos_t = tab_rdata_d[2*W-1:W];

assign tab_addr = (a-1) & 6'h3f;
assign tab_en   = buf_read_en_d[0];
//---------------------------------------------------------------------------------------------------------------------------------------------
//buffer operation
//ADDR
//loop1 << (fft_n-loop0) => the group of loop1's offset.
//loop2 
//(read_2nd ? $unsigned(1<<(fft_n-1-loop0)) :0) => fft needs 2 operands so that read needs 2 read_opera .
logic [8:0] loop_addr = (loop1 <<(fft_n-loop0)) + loop2 + (read_2nd ? $unsigned(1<<(fft_n-1-loop0)) :0);
logic [44:0] loop_addr_d ; //why does need 5 shifter?

always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        loop_addr_d <= '0;
    end
    else 
        loop_addr_d <= {loop_addr_d[35:0],loop_addr};
end
//W/R
logic   buf_write_en;
logic   read_from_a;
logic   write_to_a;
logic   write_p = REG ? ~read_2nd_d[4] : ~ read_2nd_d[3];
//read operands from bufa/b
always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
    read_from_a <= 1'b1;
    else if((fft_state==ILDE) &&start)begin
        read_from_a <= in_buf_sel_mode ? ~in_buf_sel : ~prev_in_buf_sel;
    end
    else if ((fft_state == LOOP) && loop1_end && loop2_end && read_2nd)
        read_from_a <= ~read_from_a;
    end
end
//write results to bufa/b
always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        write_to_a  <= 1'b1;
    end
    else if((fft_state == IDLE) && start)
        write_to_a  <= in_buf_sel_mode ? in_buf_sel: prev_in_buf_sel;
    else if((fft_state == LOOP) && loop1_end && loop2_end && read_2nd)
        write_to_a  <= ~write_to_a ;
end
//buf read en
always_ff@(posedge gated_clk or negedge rst_n)begin
    if(!rst_n)begin
        buf_read_en <= '0;
    end
    else if((fft_state == ILDE ) && start||
        (fft_state == LOOP_WAIT) && end_of_one_round_write && !loop_all)
        buf_read_en <= 1'b1;
    else if((fft_state == LOOP) && loop1_end && loop2_end && read_2nd)
        buf_read_en <= 1'b0;
end
//bit rev write
logic   bit_rev_write;

always_ff@(posedge gated_clk or negedge rst_n) begin
    if(!rst_n)begin
        buf_read_en_d   <= '0;
    end
    else
        buf_read_en_d   <= buf_read_en;
end
//reading && writing need to be well coordinated!! write a complex number
assign buf_write_en = bit_rev_write ? (REG ? buf_read_en_d[2] : buf_read_en_d[1]) : (REG ? buf_read_en_d[4] : buf_read_en_d[3]);
//---------------------------------------------------------------------------------------------------------------------------------------------
//bit rev read
logic bit_rev_read;
assign bit_rev_read = (fft_state == BIT_REV0) || (fft_state == BIT_REV1);

logic [8:0] bit_rev_read_addr_out = (fft_state == BIT_REV0) ? {1'b0,loop2} : {1'b0,loop2} + &unsigned(1<<(fft_n - 1'b1));
logic [8:0] bit_rev_read_addr_in  = {
    bit_rev_read_addr_out[0],    
    bit_rev_read_addr_out[1],    
    bit_rev_read_addr_out[2],    
    bit_rev_read_addr_out[3],    
    bit_rev_read_addr_out[4],    
    bit_rev_read_addr_out[5],    
    bit_rev_read_addr_out[6],    
    bit_rev_read_addr_out[7],    
    bit_rev_read_addr_out[8]}>>(9-fft_n);//bit rev logic
//---------------------------------------------------------------------------------------------------------------------------------------------
logic [8:0] bit_rev_read_addr_out_d;
always_ff @( posedge gated_clk or negedge rst_n ) begin
    if(!rst_n)begin
        bit_rev_read_addr_out_d <= '0;
        bit_rev_write   <= '0;
    end 
    else begin
        bit_rev_read_addr_out_d <= bit_rev_read_addr_out;
        bit_rev_write   <= bit_rev_read;
    end
    end
//---------------------------------------------------------------------------------------------------------------------------------------------
logic [8:0] read_addr = in_buf_offest + (bit_rev_read ? bit_rev_read_addr_in : loop_addr);

logic [8:0] write_addr = out_buf_offest + (bit_rev_write ? bit_rev_read_addr_out_d : (REG ? loop_addr_d[44:36] : loop_addr_d[35:27]));

assign buf_addr_a = buf_we_a ? write_addr : read_addr;
logic buf_re_a = (buf_read_en || bit_rev_read) && read_from_a;
assign buf_we_a = (buf_write_en || bit_rev_write) && write_to_a;
assign buf_en_a = buf_re_a || buf_we_a;
assign buf_wdata_a = bit_rev_write ? (read_from_a ? buf_rdata_a : buf_rdata_b) : write_p ? {p_i,p_r} : {q_i,q_r};

assign buf_addr_b = buf_we_b ? write_addr : read_addr;
logic buf_re_b = (buf_read_en || bit_rev_read) && !read_from_a;
assign buf_we_b = (buf_write_en || bit_rev_write) && !write_to_a;
assign buf_en_b = buf_re_b || buf_we_b;
assign buf_wdata_b = buf_wdata_a;

logic [1:0] buf_re_a_d;
logic [2*W-1:0] buf_rdata = buf_re_a_d[0] ? buf_rdata_a : buf_rdata_b;
logic [2*W-1:0] buf_rdata_d = buf_re_a_d[1] ? buf_rdata_a_d : buf_rdata_b_d;

logic [2*W-1:0] buf_rdata_d2;
logic [2*W-1:0] buf_rdata_d3;

always_ff @(posedge gated_clk or negedge rst_n ) begin
    if(!rst_n)begin
        buf_rdata_d2 <= '0;
        buf_rdata_d3 <= '0;
        buf_re_a_d <= '0;
    end
    else begin
        buf_rdata_d2    <= buf_rdata_d;
        buf_rdata_d3    <= buf_rdata_d2;
        buf_re_a_d  <= {buf_re_a_d[0],buf_re_a};
    end
end

logic [W-1:0] bf_a_r_0 = read_2nd_d[1] ? buf_rdata_d2[W-1:0] : buf_rdata_d3[W-1:0];
logic [W-1:0] bf_a_i_0 = read_2nd_d[1] ? buf_rdata_d2[2*W-1:W] : buf_rdata_d3[2*W-1:W];
logic [W-1:0] bf_b_r_0 = read_2nd_d[1] ? buf_rdata_d[W-1:0] : buf_rdata_d2[W-1:0];
logic [W-1:0] bf_b_i_0 = read_2nd_d[1] ? buf_rdata_d[2*W-1:W] : buf_rdata_d2[2*W-1:W];

assign bf_a_r = quant_current_round ? (({bf_a_r_0[W-1], bf_a_r_0}+1)>>1) : bf_a_r_0;
assign bf_a_i = quant_current_round ? (({bf_a_i_0[W-1], bf_a_i_0}+1)>>1) : bf_a_i_0;
assign bf_b_r = quant_current_round ? (({bf_b_r_0[W-1], bf_b_r_0}+1)>>1) : bf_b_r_0;
assign bf_b_i = quant_current_round ? (({bf_b_i_0[W-1], bf_b_i_0}+1)>>1) : bf_b_i_0;

assign bf_start = ~read_2nd_d[2] && buf_read_en_d[2]; //bf start after 3 cycles of buffer read(add one pipeline cycle)

assign result_in = read_from_a;
endmodule
`endif
