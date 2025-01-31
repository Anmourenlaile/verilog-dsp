// *********************************************************************************
// Project Name : zkx2024
// Author       : Jlan
// Email        : 15533610762@163.com
// Create Time  : 2024-06-26
// File Name    : Audio_dsp_top.v
// Module Name  : Audio_dsp_top
// Called By    : jlan
// Abstract     : Audio_dsp_top
//
// 
// *********************************************************************************
// Modification History:
// Date         By              Version                 Change Description
// -----------------------------------------------------------------------
// 2024-06-26    Macro           1.0                     Original
//  
// *********************************************************************************

module Audio_dsp_top #(
    parameter   W   =24,
    parameter   W_BUF   =30,    //width of buf_a/buf_b data,which is W+6(6 is for exponent part)
    parameter   DIN_CH  =2,
    parameter   LOG_PREC = 16,
    parameter   LN_OUT_QUANT = 18
)   (
    input   clk,
    input   rst_n,

//    input   [15:0]reg_sbuf0_memctrl,
//    input   [15:0]reg_sbuf1_memctrl,
//    input   [15:0]reg_bufa_memctrl,
//    input   [15:0]reg_bufb_memctrl,
//    input   [15:0]reg_bufc_memctrl,

    input   [W*DIN_CH-1:0] aud_data_in, //pdm data in from vad module
    input   [0:0]          aud_data_valid,

    //register for dsp
    input   [1:0]   reg_ch_mode,    //0:1ch ;1:2ch 3:4ch
    input           reg_sample_save_en,
    input           reg_aud_auto_pre_proc_en,   //0:disable 1:enable
    input   [1:0]   reg_aapp_ch_sel,    //0:left 1:right 2:fornt 3:back
    input           reg_dat_store_mode, //0:block 1:interleave
    input   [7:0]   reg_aapp_fft_size,  //float point frame size ,d=[7:0] ,e=[7:5],frame size = (d+1)*16
    input   [7:0]   reg_frame_size,
    input           reg_frame_mode,
    input   [15:0]  reg_aapp_buf_base,
    input           reg_2ch_handle, // 1:enable handle 2 ch data in once sample
    output   logic  reg_ready_blk_id, //0:bufa 1:bufb

    input           reg_aud_dsp_start, //one clock pulse signal
    output   logic  reg_aud_dsp_busy,   //0:idle 1:busy
    input   [5:0]   reg_aud_dsp_op,// 0:hanmming window;
                                   // 2:complex abs
                                   // 3:ln
                                   // 4:MFCC(abs+mel map +ln)
                                   // 0x20:FFT
                                   // 0x21:ifft
                                   // 0x30:feature extract (ham win+fft+mfcc)
    input           reg_aud_dsp_in_buf_sel_mode,// 0:use previous result; 1:set by aud_dsp_in_buf_sel
    input           reg_aud_dsp_in_buf_sel, //0:from buffer A; 1:from buffer B
    input   [7:0]   reg_aud_dsp_in_buf_offset,
    input   [7:0]   reg_aud_dsp_out_buf_offset,
    input   [7:0]   reg_aud_dsp_coef_buf_offset,
    input   [5:0]   reg_aud_dsp_in_quant_bits, // input data quant bit(Q) ,signal ,read data is DI*2^Q
    output   logic     [5:0]    reg_aud_dsp_out_quant_bits,
    output   logic              reg_aud_dsp_result_in,  //result in buf a ;result in bufb
    input   [31:0]  reg_aud_dsp_param0; //op parameters
    input   [31:0]  reg_aud_dsp_param1; //op parameters
    input   [31:0]  reg_aud_dsp_param2; //op parameters
    
    //ibex bus

    input   [31:0]  h_addr,
    input           h_req_vld,
    output   logic  h_req_rdy,
    input           h_we,
    input   [3:0]   h_wbe,
    input   [31:0]  h_wdata,
    output   logic  [31:0]  h_rdata,
    output   logic          h_vld,

    //aud dsp interrupt signal
    input   [1:0]   intr_mask, // intrruput mask write 1 to enable
    input   [1:0]   clr_intr_status, //pulse signal to clear intr_status
    output   logic  [1:0]   intr_status, // 0: audio frame ready; [1] audio dsp done
    output   logic          intr    // if there is a intr in dsp and the mask is 1 .it will be 1.

);
//-------------------------------------------------------------------------------------------------------
logic   [5:0]   reg_aud_dsp_out_quant_bits_d;
logic   [0:0]   reg_aud_dsp_start_sync;

pulse_2d_sync   reg_aud_dsp_start_sync_reg(
    .dout(reg_aud_dsp_start_sync),
    .clk(clk),
    .rst_n(rst_n),
    .din(reg_aud_dsp_start)
);

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        reg_aud_dsp_out_quant_bits_d    <= '0;
    end
    else
        reg_aud_dsp_out_quant_bits_d    <= reg_aud_dsp_out_quant_bits;
end 
// the out quant bit maybe is the vector core output .
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
XBAR_TCDM_BUS aud_in_fe_2_mux_intf();
XBAR_TCDM_BUS soc_2_mux_intf();
XBAR_TCDM_BUS mux_intf();

//-------------------------------------------------------------------------------------------------------
assign  soc_2_mux_intf.add = h_addr;
assign  soc_2_mux_intf.req = h_req;
assign  h_req_rdy          = soc_2_mux_intf.gnt;
assign  soc_2_mux_intf.wen = h_we;
assign  soc_2_mux_intf.be  = h_wbe;
assign  soc_2_mux_intf.wdata = h_wdata;
assign  h_rdata            = soc_2_mux_intf.r_rdata;
assign  h_vld              = soc_2_mux_intf.r_valid;
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
logic   [10:0]  sample_buf_addr;
logic   [0:0]   sample_buf_en;
logic   [0:0]   sample_buf_we;
logic   [1:0]   sample_buf_wbe; // the minimal unit is an audio data(Width is W)
logic   [2*W-1:0] sample_buf_wdata;
logic   [2*W-1:0] sample_buf_rdata;
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
logic   [31:0]  aud_in_fe_h_addr;
logic   [0:0]   aud_in_fe_h_req_vld;
logic   [0:0]   aud_in_fe_h_req_rdy;
logic   [0:0]   aud_in_fe_h_we;
logic   [3:0]   aud_in_fe_h_wbe;
logic   [31:0]  aud_in_fe_h_wdata;
logic   [31:0]  aud_in_fe_h_rdata;
logic   [0:0]   aud_in_fe_h_vld;
logic   [0:0]   dsp_done;//dsp done :one pulse signal for dual handle for aud dsp
logic   [0:0]   cmd_dsp_done;
//-------------------------------------------------------------------------------------------------------
logic   [0:0]   aud_data_valid_sync;

pulse_2dp0_sync   aud_data_valid_sync_reg(
    .dout(aud_data_valid_sync),
    .clk(clk),
    .rst_n(rst_n);
    .din(aud_data_valid)
);

aud_in_fe #(
    .W(W),
    .W_BUF(W_BUF),
    .CH(DIN_CH),
    .S_BUF_AW(11)
) aud_in_fe(
    .clk(clk),
    .rst_n(rst_n),
    .aud_data_in(aud_data_in[W*DIN_CH-1:0]),
    .aud_data_valid(aud_data_valid_sync),
    .reg_ch_mode(reg_ch_mode[1:0]),
    .reg_sample_save_en(reg_sample_save_en),
    .reg_aud_auto_pre_proc_en(reg_aud_auto_pre_proc_en), //0:disable 1:enable 
    .reg_aapp_ch_sel(reg_aapp_ch_sel[1:0]),
    .reg_dat_store_mode(reg_dat_store_mode), //o:block 1:interleave
    .reg_aapp_fft_size(reg_aapp_fft_size[7:0]),
    .reg_frame_size(reg_frame_size[7:0]),
    .reg_frame_mode(reg_frame_mode),
    .reg_aapp_buf_base(reg_aapp_buf_base[15:0]),
    .reg_aud_dsp_in_quant_bits(reg_aud_dsp_in_quant_bits),
    .reg_2ch_handle(reg_2ch_handle),
    .reg_dsp_done(dsp_done),
    .sample_buf_addr(sample_buf_addr[10:0]),
    .sample_buf_en(sample_buf_en),
    .sample_buf_we(sample_buf_we),
    .sample_buf_wbe(sample_buf_wbe),
    .sample_buf_wdata(sample_buf_wdata[2*W-1:0]),
    .sample_buf_rdata(sample_buf_rdata[2*W-1:0]),

    .mst(aud_in_fe_2_mux_intf),// interface

    .start_aapp(start_aapp), // not define
    .aapp_quant_input(aapp_quant_input), // not define
    .reg_ready_blk_id(reg_ready_blk_id),
    .intr(aud_in_fe_intr) // not define
);
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
logic   [5:0]   reg_cmd_op;
logic   [5:0]   reg_cmd_in_quant_bits;
logic   [5:0]   reg_cmd_in_buf_offset;
logic   [5:0]   reg_cmd_out_buf_offset;
logic   [5:0]   reg_cmd_coef_buf_offset;
logic   [5:0]   reg_cmd_out_quant_bits;
logic   [5:0]   reg_cmd_param0;
logic   [0:0]   cmd_cfg_sel_tmp;
logic   [0:0]   cmd_cfg_sel;
logic   [0:0]   cmd_proc_start;
logic   [5:0]   cmd_proc_op;
logic   [31:0]  cmd_proc_param0;
logic   [7:0]   cmd_proc_coef_buf_offest;
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        cmd_cfg_sel_tmp <= '0;
    end
    else if(reg_aud_dsp_start_sync)
        cmd_cfg_sel_tmp <= 1'b1;
    else if(start_aapp)
        cmd_cfg_sel_tmp <= 1'b0;
end 

assign  cmd_cfg_sel = cmd_cfg_sel_tmp & ~start_aapp | reg_aud_dsp_start_sync;
assign  cmd_proc_start  =   reg_aud_dsp_start_sync | start_aapp;
assign  cmd_proc_op = cmd_cfg_sel ? reg_aud_dsp_op[5:0] : 6'h30;
assign  cmd_proc_param0 = cmd_cfg_sel ? reg_cmd_param0 :
                            {reg_aud_dsp_param0[31:5],aapp_quant_input,reg_aud_dsp_param0[3:0]};

assign  cmd_proc_coef_buf_offest = reg_aud_dsp_coef_buf_offset[7:0];

cmd_proc cmd_proc(
    .clk(clk),
    .rst_n(rst_n),

    .reg_aud_dsp_start(cmd_proc_start),
    .reg_aud_dsp_busy(cmd_reg_aud_dsp_busy),// not define
    .reg_aud_dsp_op(cmd_proc_op[5:0]),
    .reg_aud_dsp_in_buf_sel_mode(reg_aud_dsp_in_buf_sel_mode),
    .reg_aud_dsp_in_buf_sel(reg_aud_dsp_in_buf_sel),
    .reg_aud_dsp_in_quant_bits(reg_aud_dsp_in_quant_bits[5:0]),
    .reg_aud_dsp_in_buf_offset(reg_aud_dsp_in_buf_offset[7:0]),
    .reg_aud_dsp_out_buf_offset(reg_aud_dsp_out_buf_offset[7:0]),
    .reg_aud_dsp_coef_buf_offset(cmd_proc_coef_buf_offset[7:0]),
    .reg_aud_dsp_param0(cmd_proc_param0[31:0]),
    .cmd_dsp_done(cmd_dsp_done),
    .reg_2ch_handle(reg_2ch_handle),
    .reg_cmd_start(reg_cmd_start), // not define
    .reg_cmd_busy(reg_cmd_busy), //not define
    .reg_cmd_op(reg_cmd_op[5:0]),
    .reg_cmd_in_buf_sel_mode(reg_cmd_in_buf_sel_mode),//not difine
    .reg_cmd_in_buf_sel(reg_cmd_in_buf_sel),
    .reg_cmd_in_quant_bits(reg_cmd_in_quant_bits),
    //.reg_cmd_out_quant_bits(reg_cmd_out_quant_bits),
    .reg_cmd_in_buf_offset(reg_cmd_in_buf_offset[7:0]),
    .reg_cmd_out_buf_offset(reg_cmd_out_buf_offset[7:0]),
    .reg_cmd_coef_buf_offset(reg_cmd_coef_buf_offset[7:0]),
    .reg_cmd_param0(reg_cmd_param0)
);
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
logic   [0:0]   cmd_sel;
logic   [0:0]   reg_aud_dsp_start_mux;
logic   [5:0]   reg_aud_dsp_op_mux;
logic   [0:0]   reg_aud_dsp_in_buf_sel_mode_mux;
logic   [0:0]   reg_aud_dsp_in_buf_sel_mux;
logic   [5:0]   reg_aud_dsp_in_quant_bits_mux;
logic   [7:0]   reg_aud_dsp_in_buf_offset_mux;
logic   [7:0]   reg_aud_dsp_out_buf_offset_mux;
logic   [7:0]   reg_aud_dsp_coef_buf_offset_mux;
logic   [31:0]  reg_aud_dsp_param0_mux;


assign  cmd_sel =   cmd_reg_aud_dsp_busy;
assign  reg_aud_dsp_start_mux   =   cmd_sel ? reg_cmd_start : (reg_aud_dsp_start_sync && (reg_aud_dsp_op[5:4] != 2'b11)); //why not 11?
assign  reg_aud_dsp_op_mux  =   cmd_sel ? reg_cmd_op[5:0] : reg_aud_dsp_op[5:0];
assign  reg_aud_dsp_in_buf_sel_mode_mux =   cmd_sel ? reg_cmd_in_buf_sel_mode : reg_aud_dsp_in_buf_sel_mode;
assign  reg_aud_dsp_in_buf_sel_mux  =   cmd_sel ? reg_cmd_in_buf_sel : reg_aud_dsp_in_buf_sel;
assign  reg_aud_dsp_in_quant_bits_mux   =   cmd_sel ? reg_cmd_in_quant_bits[5:0] : reg_aud_dsp_in_quant_bits[5:0];
assign  reg_aud_dsp_in_buf_offset_mux   =   cmd_sel ? reg_cmd_in_buf_offset[7:0] : reg_aud_dsp_in_buf_offset[7:0];
assign  reg_aud_dsp_out_buf_offset_mux   =   cmd_sel ? reg_cmd_out_buf_offset[7:0] : reg_aud_dsp_out_buf_offset[7:0];
assign  reg_aud_dsp_coef_quant_bits_mux   =   cmd_sel ? reg_cmd_coef_quant_bits[7:0] : reg_aud_dsp_coef_quant_bit[7:0];
assign  reg_aud_dsp_param0_mux   =   cmd_sel ? reg_cmd_param0[31:0] : reg_aud_dsp_param0[31:0];
//-------------------------------------------------------------------------------------------------------
logic   [0:0]   reg_fft_start;
logic   [0:0]   reg_fft_busy;
logic   [0:0]   reg_fft_ifft_en; //[0]:fft [1] ifft
logic   [0:0]   reg_fft_in_buf_sel_mode;
logic   [0:0]   reg_fft_in_buf_sel;
logic   [0:0]   reg_fft_quant_input_en; // quantify 1 bit before first round, 0:disable ,1:enable
logic   [3:0]   reg_fft_n; //calculate 2^fft_n = points for fft
logic   [5:0]   reg_fft_out_quant_bits;
logic   [0:0]   reg_fft_result_in;


assign  reg_fft_start   =   reg_aud_dsp_start_mux && (reg_aud_dsp_op_mux[5:4] == 2'b10); //pulse signal
assign  reg_fft_ifft_en =   reg_aud_dsp_op_mux[0];  //[0]:fft; [1]ifft
assign  reg_fft_in_buf_sel_mode =   reg_aud_dsp_in_buf_sel_mode_mux;
assign  reg_fft_in_buf_sel  =   reg_aud_dsp_in_buf_sel_mux;
assign  reg_fft_quant_input_en  =   reg_aud_dsp_param0_mux[4];
assign  reg_fft_n   =   reg_aud_dsp_param0_mux[3:0];    //calculate 2^fft_n
//-------------------------------------------------------------------------------------------------------
logic   [0:0]   reg_vector_start;
logic   [0:0]   reg_vector_busy;
logic   [0:0]   reg_vector_in_buf_sel_mode;
logic   [0:0]   reg_vector_in_buf_sel;
logic   [5:0]   reg_vector_in_quant_bits;
logic   [4:0]   reg_vector_op;
logic   [9:0]   reg_vector_tab_offset;
logic   [0:0]   reg_tab_type;
logic   [12:0]  reg_vector_pre_emp_k;
logic   [W-1:0] reg_ham_win_prev_sample;
logic   [0:0]   reg_use_prev_sample;
logic   [3:0]   reg_ham_win_prec_keep;
logic   [3:0]   reg_mfcc_out_prec;
logic   [8:0]   reg_vector_vector_len;
logic   [8:0]   reg_vector_in_vector_start;
logic   [8:0]   reg_vector_out_vector_start;
logic   [0:0]   scalar_op;
logic   [0:0]   complex_op;
logic   [W-1:0] scalar_dat;
logic   [5:0]   scalar_qbit;
logic   [W-1:0] complex_image_dat;


assign  reg_vector_start    =   reg_aud_dsp_start_mux && (reg_aud_dsp_op_mux[5] == 1'b0);
assign  reg_vector_in_buf_sel_mode  =   reg_aud_dsp_in_buf_sel_mode_mux;
assign  reg_vector_in_buf_sel  =   reg_aud_dsp_in_buf_sel_mux;
assign  reg_vector_in_quant_bits    =   reg_aud_dsp_in_quant_bits_mux;
assign  reg_vector_op   =   reg_aud_dsp_op_mux[4:0]; // why only use 5bit ? because the op[5] is 0 in vector start!
assign  reg_vector_tab_offset   =   {reg_aud_dsp_coef_buf_offset_mux, 3'h0}; // why *8?
assign  reg_tab_type    =   reg_aud_dsp_param0_mux[30];
assign  reg_vector_pre_emp_k    =   reg_aud_dsp_param0_mux[21:9];
assign  reg_ham_win_prev_sample =   reg_aud_dsp_param1[W-1:0];
assign  reg_use_prev_sample =   reg_aud_dsp_param1[24];
assign  reg_ham_win_prec_keep   =   reg_aud_dsp_param0_mux[25:22];
assign  reg_mfcc_out_prec   =   reg_aud_dsp_param0_mux[29:26];
assign  reg_vector_vector_len   =   reg_aud_dsp_param0_mux[8:0];
assign  reg_vector_in_vector_start  =   {reg_aud_dsp_in_buf_offset_mux[7:0],2'b00};
assign  reg_vector_out_vector_start  =   {reg_aud_dsp_out_buf_offset_mux[7:0],2'b00};
assign  scalar_op   =   reg_aud_dsp_param0_mux[30];
assign  complex_op   =   reg_aud_dsp_param0_mux[29];
assign  scalar_dat  =   reg_aud_dsp_param1[W-1:0];
assign  scalar_qbit =   reg_aud_dsp_param1[29:24];
assign  complex_image_dat   =   reg_aud_dsp_param2[W-1:0];


//-------------------------------------------------------------------------------------------------------
logic   [5:0]   reg_vector_out_quant_bits;
logic   [0:0]   reg_vector_result_in;
logic   [0:0]   reg_fft_busy_d;
logic   [0:0]   reg_vector_busy_d;//for negedge edge checking to catch out quants and result_buf

assign  reg_aud_dsp_busy    =   reg_fft_busy | reg_vector_busy | cmd_reg_aud_dsp_busy;
assign  reg_cmd_busy    =   reg_fft_busy | reg_vector_busy;

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        reg_fft_busy_d      <= '0;
        reg_vector_busy_d   <= '0;
    end
    else begin
        reg_fft_busy_d      <= reg_fft_busy;
        reg_vector_busy_d   <= reg_vector_busy;
    end
end 
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
logic   [5:0]   reg_aud_dsp_out_quant_bits_s;
logic   [0:0]   reg_aud_dsp_result_in_s;
//use reg to save out quant and result
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        reg_aud_dsp_out_quant_bits_s    <=  '0;
        reg_aud_dsp_result_in_s         <=  '0;
    end
    else if(!reg_fft_busy && reg_fft_busy_d)begin
        reg_aud_dsp_out_quant_bits_s    <=  reg_fft_out_quant_bits;
        reg_aud_dsp_result_in_s         <=  reg_fft_result_in;
    end
    else if(!reg_vector_busy && reg_vector_busy_d)begin
        reg_aud_dsp_out_quant_bits_s    <=  reg_vector_out_quant_bits;
        reg_aud_dsp_result_in_s         <=  reg_vector_result_in;
    end
end 

always_comb begin
    if(!reg_fft_busy && reg_fft_busy_d)
        {reg_aud_dsp_out_quant_bits,reg_aud_dsp_result_in} = {reg_fft_out_quant_bits,reg_fft_result_in};
    else if(!reg_vector_busy && reg_vector_busy_d)
        {reg_aud_dsp_out_quant_bits,reg_aud_dsp_result_in} = {reg_vector_out_quant_bits,reg_vector_result_in};
    else
        {reg_aud_dsp_out_quant_bits,reg_aud_dsp_result_in} = {reg_aud_dsp_out_quant_bits_s,reg_aud_dsp_result_in_s};

end 
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
logic   [8:0]           core_buf_addr_a;
logic   [0:0]           core_buf_en_a;
logic   [0:0]           core_buf_we_a;
logic   [1:0]           core_buf_wbe_a;
logic   [2*W_BUF-1:0]   core_buf_wdata_a;
logic   [2*W_BUF-1:0]   core_buf_rdata_a;


logic   [8:0]           core_buf_addr_b;
logic   [0:0]           core_buf_en_b;
logic   [0:0]           core_buf_we_b;
logic   [1:0]           core_buf_wbe_b;
logic   [2*W_BUF-1:0]   core_buf_wdata_b;
logic   [2*W_BUF-1:0]   core_buf_rdata_b;


logic   [9:0]           core_tab_addr;//calculate parameter r_addr
logic   [0:0]           core_tab_en;
logic   [2*W-1:0]       core_tab_rdata;//parameters all are 24 bit quants;


logic   [8:0]           fft_buf_addr_a;
logic   [0:0]           fft_buf_en_a;
logic   [0:0]           fft_buf_we_a;
logic   [1:0]           fft_buf_wbe_a;
logic   [2*W_BUF-1:0]   fft_buf_wdata_a;
logic   [2*W_BUF-1:0]   fft_buf_rdata_a;

logic   [8:0]           fft_buf_addr_b;
logic   [0:0]           fft_buf_en_b;
logic   [0:0]           fft_buf_we_b;
logic   [1:0]           fft_buf_wbe_b;
logic   [2*W_BUF-1:0]   fft_buf_wdata_b;
logic   [2*W_BUF-1:0]   fft_buf_rdata_b;


logic   [8:0]           vector_buf_addr_a;
logic   [0:0]           vector_buf_en_a;
logic   [0:0]           vector_buf_we_a;
logic   [1:0]           vector_buf_wbe_a;
logic   [2*W_BUF-1:0]   vector_buf_wdata_a;
logic   [2*W_BUF-1:0]   vector_buf_rdata_a;

logic   [8:0]           vector_buf_addr_b;
logic   [0:0]           vector_buf_en_b;
logic   [0:0]           vector_buf_we_b;
logic   [1:0]           vector_buf_wbe_b;
logic   [2*W_BUF-1:0]   vector_buf_wdata_b;
logic   [2*W_BUF-1:0]   vector_buf_rdata_b;


logic   [9:0]           fft_tab_addr;//calculate parameter r_addr
logic   [0:0]           fft_tab_en;
logic   [2*W-1:0]       fft_tab_rdata;//parameters all are 24 bit quants;


logic   [9:0]           vector_tab_addr;//calculate parameter r_addr
logic   [0:0]           vector_tab_en;
logic   [2*W-1:0]       vector_tab_rdata;//parameters all are 24 bit quants;

logic   [0:0]           complex_mul_en;
logic   [0:0]           complex_mul_start;
logic   [W-1:0]         complex_mul_a_r;
logic   [W-1:0]         complex_mul_a_i;
logic   [W-1:0]         complex_mul_b_r;
logic   [W-1:0]         complex_mul_b_i;
logic   [2*W-1:0]       complex_mul_o_r;
logic   [2*W-1:0]       complex_mul_o_i;

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
//FFT_CORE
fft_core    #(
    .W(W),
    .W_BUF(W_BUF),
    .REG(1)
) fft_core(
    .clk(clk),
    .rst_n(rst_n),

    .start(reg_fft_start),
    .busy(reg_fft_busy),
    .ifft_en(reg_fft_ifft_en),
    .in_buf_sel_mode(reg_fft_in_buf_sel_mode),
    .pre_in_buf_sel(reg_aud_dsp_result_in),//pre in buf select be used in the case which is dsp have cal once and the result in dsp bufa/b
    .in_buf_sel(reg_fft_in_buf_sel),
    .in_buf_offset({reg_aud_dsp_in_buf_offset[7:0],1'b0}),
    .out_buf_offset({reg_aud_dsp_out_buf_offset[7:0],1'b0}),
    .quant_input_en(reg_fft_quant_input_en),
    .prev_in_quant_bits(reg_aud_dsp_out_quant_bits[5:0]),
    .in_quant_bits(reg_aud_dsp_in_quant_bits[5:0]),
    .fft_n(reg_fft_n[3:0]),//1-9 -> 2-512

    .tab_addr(fft_tab_addr[9:0]),
    .tab_en(fft_tab_en),
    .tab_rdata(fft_tab_rdata[2*W-1:0]),

    .buf_addr_a(fft_buf_addr_a[8:0]),
    .buf_we_a(fft_buf_we_a),
    .buf_en_a(fft_buf_en_a),
    .buf_wdata_a_full(fft_buf_wdata_a[2*W_BUF-1:0]),
    .buf_rdata_a_full(fft_buf_rdata_a[2*W_BUF-1:0]),

    
    .buf_addr_b(fft_buf_addr_b[8:0]),
    .buf_we_b(fft_buf_we_b),
    .buf_en_b(fft_buf_en_b),
    .buf_wdata_b_full(fft_buf_wdata_b[2*W_BUF-1:0]),
    .buf_rdata_b_full(fft_buf_rdata_b[2*W_BUF-1:0]),

    .out_quant_bits(reg_fft_out_quant_bits[5:0]),
    .result_in(reg_fft_result_in),


    .complex_mul_en(complex_mul_en),
    .complex_mul_start(complex_mul_start),
    .complex_mul_a_r(complex_mul_a_r[W-1:0]),
    .complex_mul_a_i(complex_mul_a_i[W-1:0]),
    .complex_mul_b_r(complex_mul_b_r[W-1:0]),
    .complex_mul_b_i(complex_mul_b_i[W-1:0]),
    .complex_mul_o_r(complex_mul_o_r[2*W-1:0]),
    .complex_mul_o_i(complex_mul_o_i[2*W-1:0])

);
//-------------------------------------------------------------------------------------------------------

vector_core #(
    .W(W),
    .W_BUF(W_BUF),
    .LOG_PREC(LOG_PREC),
    .LN_OUT_QUANT(LN_OUT_QUANT),
    .REG(1)
)   vector_core(
    .clk(clk),
    .rst_n(rst_n),

    .start(reg_vector_start),
    .busy(reg_vector_busy),
    .in_buf_sel_mode(reg_vector_in_buf_sel_mode),
    .pre_in_buf_sel(reg_aud_dsp_result_in),
    .in_buf_sel(reg_vector_in_buf_sel),
    .prev_in_quant_bits(reg_aud_dsp_out_quant_bits[5:0]),
    .in_quant_bits(reg_vector_in_quant_bits[5:0]),

    .op(reg_vector_op[4:0]),//0:win on real 1:win on image 2:complex abs
    .tab_offset(reg_vector_tab_offset[9:0]),
    .tab_type(reg_tab_type),
    .prev_emp_k(reg_vector_pre_emp_k[12:0]), //k is Q13
    .ham_win_prev_sample(reg_ham_win_prev_sample[W-1:0]),
    .mfcc_out_prec(reg_mfcc_out_prec[3:0]),
    .vector_len(reg_vector_vector_len[8:0]),
    .in_vector_start(reg_vector_in_vector_start[8:0]),
    .out_vector_start(reg_vector_out_vector_start[8:0]),
    .scalar_op(scalar_op),
    .scalar_dat(scalar_dat[W-1:0]),
    .scalar_qbit(scalar_qbit[5:0]),
    .complex_op(complex_op),
    .complex_image_dat(complex_image_dat[W-1:0]),

    .tab_addr(vector_tab_addr[9:0]),
    .tab_en(vector_tab_en),
    .tab_rdata(vector_tab_rdata[2*W-1:0]),

    .buf_addr_a(vector_buf_addr_a[8:0]),
    .buf_we_a(vector_buf_we_a),
    .buf_wbe_a(vector_buf_wbe_a[1:0]),
    .buf_en_a(vector_buf_en_a),
    .buf_wdata_a_full(vector_buf_wdata_a[2*W_BUF-1:0]),
    .buf_rdata_a_full(vector_buf_rdata_a[2*W_BUF-1:0]),

    
    .buf_addr_b(vector_buf_addr_b[8:0]),
    .buf_we_b(vector_buf_we_b),
    .buf_wbe_b(vector_buf_wbe_b[1:0]),
    .buf_en_b(vector_buf_en_b),
    .buf_wdata_b_full(vector_buf_wdata_b[2*W_BUF-1:0]),
    .buf_rdata_b_full(vector_buf_rdata_b[2*W_BUF-1:0]),

    .out_quant_bits(reg_vector_out_quant_bits[5:0]),
    .result_in(reg_vector_result_in),


    .complex_mul_en(complex_mul_en),
    .complex_mul_start(complex_mul_start),
    .complex_mul_a_r(complex_mul_a_r[W-1:0]),
    .complex_mul_a_i(complex_mul_a_i[W-1:0]),
    .complex_mul_b_r(complex_mul_b_r[W-1:0]),
    .complex_mul_b_i(complex_mul_b_i[W-1:0]),
    .complex_mul_o_r(complex_mul_o_r[2*W-1:0]),
    .complex_mul_o_i(complex_mul_o_i[2*W-1:0]),
    .reg_2ch_handle(reg_2ch_handle),
    .reg_dsp_done(cmd_dsp_done)

);


assign  core_buf_addr_a =   vector_buf_en_a ? vector_buf_addr_a : fft_buf_addr_a;
assign  core_buf_en_a   =   fft_buf_en_a | vector_buf_en_a;
assign  core_buf_we_a   =   vector_buf_en_a ? vector_buf_we_a   : fft_buf_we_a;
assign  core_buf_wbe_a   =  vector_buf_en_a ? vector_buf_wbe_a   : 2'h3;
assign  core_buf_wdata_a =  vector_buf_en_a ? vector_buf_wdata_a : fft_buf_wdata_a;
assign  fft_buf_rdata_a =   core_buf_rdata_a;
assign  vector_buf_rdata_a  =   core_buf_rdata_a;


assign  core_buf_addr_b =   vector_buf_en_b ? vector_buf_addr_b : fft_buf_addr_b;
assign  core_buf_en_b   =   fft_buf_en_b | vector_buf_en_b;
assign  core_buf_we_b   =   vector_buf_en_b ? vector_buf_we_b   : fft_buf_we_b;
assign  core_buf_wbe_b   =  vector_buf_en_b ? vector_buf_wbe_b   : 2'h3;
assign  core_buf_wdata_b =  vector_buf_en_b ? vector_buf_wdata_b : fft_buf_wdata_b;
assign  fft_buf_rdata_b =   core_buf_rdata_b;
assign  vector_buf_rdata_b  =   core_buf_rdata_b;

assign  core_tab_addr   =   vector_tab_en ? vector_tab_addr : fft_tab_addr;
assign  core_tab_en     =   vector_tab_en | fft_tab_en;
assign  fft_tab_rdata   =   core_tab_rdata;
assign  vector_tab_rdata =  core_tab_rdata;


//-------------------------------------------------------------------------------------------------------
logic   [10:0]      din_fe_buf_addr;
logic   [0:0]       din_fe_buf_en;
logic   [0:0]       din_fe_buf_we;
logic   [7:0]       din_fe_buf_wbe;
logic   [2*W-1:0]   din_fe_buf_wdata;
logic   [2*W-1:0]   din_fe_buf_rdata;
//-------------------------------------------------------------------------------------------------------
logic   [8:0]       buf_addr_a;
logic   [0:0]       buf_en_a;
logic   [0:0]       buf_we_a;
logic   [7:0]       buf_wbe_a;
logic   [2*W_BUF-1:0]   buf_wdata_a;
logic   [2*W_BUF-1:0]   buf_rdata_a;
//-------------------------------------------------------------------------------------------------------
logic   [8:0]       buf_addr_b;
logic   [0:0]       buf_en_b;
logic   [0:0]       buf_we_b;
logic   [7:0]       buf_wbe_b;
logic   [2*W_BUF-1:0]   buf_wdata_b;
logic   [2*W_BUF-1:0]   buf_rdata_b;
//-------------------------------------------------------------------------------------------------------
logic   [9:0]       tab_addr;
logic   [0:0]       tab_en;
logic   [0:0]       tab_we;
logic   [7:0]       tab_wbe;
logic   [2*W_BUF-1:0]   tab_wdata;
logic   [2*W_BUF-1:0]   tab_rdata;
//-------------------------------------------------------------------------------------------------------

dspu_lint_wait_2m1s host_bus_mux(
    .clk(clk),
    .rst_n(rst_n),

    .slv0(aud_in_fe_2_mux_intf),
    .slv1(soc_2_mux_intf),
    .mst(mux_intf)
);

aud_dsp_bus_mux #(
    .W(W),
    .W_BUF(W_BUF),
    .S_BUF_AW(11)
) aud_dsp_bus_mux(
    .clk(clk),
    .rst_n(rst_n),

    .reg_rdata_sign_ext(reg_aud_dsp_param0[31]), //what mean?

    .h_addr(mux_intf.add[31:0]),
    .h_req_vld(mux_intf.req),
    .h_req_rdy(mux_intf.gnt),
    .h_we(mux_intf.wen),     
    .h_wbe(mux_intf.be[3:0]),    
    .h_wdata(mux_intf.wdata[31:0]),  
    .h_rdata(mux_intf.r_rdata[31:0]),  
    .h_vld(mux_intf.r_valid),

    .sample_buf_addr(sample_buf_addr[10:0]),
    .sample_buf_en(sample_buf_en),
    .sample_buf_we(sample_buf_we),
    .sample_buf_wbe(sample_buf_wbe[1:0]),
    .sample_buf_wdata(sample_buf_wdata[2*W-1:0]),
    .sample_buf_rdata(sample_buf_rdata[2*W-1:0]),

    .core_buf_addr_a(core_buf_addr_a[8:0]),
    .core_buf_en_a(core_buf_en_a),
    .core_buf_we_a(core_buf_we_a),
    .core_buf_wbe_a(core_buf_wbe_a),
    .core_buf_wdata_a(core_buf_wdata_a[2*W_BUF-1:0]),
    .core_buf_rdata_a(core_buf_rdata_a[2*W_BUF-1:0]),

   
    .core_buf_addr_b(core_buf_addr_b[8:0]),
    .core_buf_en_b(core_buf_en_b),
    .core_buf_we_b(core_buf_we_b),
    .core_buf_wbe_b(core_buf_wbe_b),
    .core_buf_wdata_b(core_buf_wdata_b[2*W_BUF-1:0]),
    .core_buf_rdata_b(core_buf_rdata_b[2*W_BUF-1:0]),

    .core_tab_addr(core_tab_addr[9:0]),
    .core_tab_en(core_tab_en),
    .core_tab_rdata(core_tab_rdata[2*W-1:0]),


    .din_fe_buf_addr(din_fe_buf_addr[10:0]),
    .din_fe_buf_en(din_fe_buf_en),
    .din_fe_buf_we(din_fe_buf_we),
    .din_fe_buf_wbe(din_fe_buf_wbe[1:0]),
    .din_fe_buf_wdata(din_fe_buf_wdata[2*W-1:0]),
    .din_fe_buf_rdata(din_fe_buf_rdata[2*W-1:0]),


    .buf_addr_a(buf_addr_a[8:0]),
    .buf_en_a(buf_en_a),
    .buf_we_a(buf_we_a),
    .buf_wbe_a(buf_wbe_a[7:0]),
    .buf_wdata_a(buf_wdata_a[2*W_BUF-1:0]),
    .buf_rdata_a(buf_rdata_a[2*W_BUF-1:0]),

   
    .buf_addr_b(buf_addr_b[8:0]),
    .buf_en_b(buf_en_b),
    .buf_we_b(buf_we_b),
    .buf_wbe_b(buf_wbe_b[7:0]),
    .buf_wdata_b(buf_wdata_b[2*W_BUF-1:0]),
    .buf_rdata_b(buf_rdata_b[2*W_BUF-1:0]),


    .tab_addr(tab_addr[9:0]),
    .tab_en(tab_en),
    .tab_we(tab_we),
    .tab_wbe(tab_wbe[7:0]),
    .tab_wdata(tab_wdata[2*W-1:0]),
    .tab_rdata(tab_rdata[2*W-1:0]),
);///the module is not complete



//-------------------------------------------------------------------------------------------------------
logic   [47:0]  din_fe_buf_wem;
logic   [2*W-1:0] din_fe_buf_rdata ,samp_buf_0_qp,samp_buf_0_q; //don't has dft -> samp_buf_1_qp does't be used!!!
//logic   [2*W-1:0] din_fe_buf_rdata_1 ,samp_buf_1_qp,samp_buf_1_q; //the module only handles 2ch .so it does't need sample buf1.

assign  din_fe_buf_wem ={
    {8{din_fe_buf_wbe[6]}},
    {8{din_fe_buf_wbe[5]}},
    {8{din_fe_buf_wbe[4]}},
    {8{din_fe_buf_wbe[3]}},
    {8{din_fe_buf_wbe[2]}},
    {8{din_fe_buf_wbe[1]}},
    {8{din_fe_buf_wbe[0]}}
};

assign  din_fe_buf_rdata  = samp_buf_0_q;
//synopsys translate_off
sram_model  #(
    .WIDTH(48),
    .DEPTH(1024),
    .ADDR(1024) )
samp_buf_1024x48(
    .ADR(din_fe_buf_addr[9:0]),
    //.D({10'b0,din_fe_buf_addr[9:0]}),
    .D(din_fe_buf_wdata[47:0]),
    .WEM(din_fe_buf_wem[47:0]),
    .WE(din_fe_buf_we),
    .ME(din_fe_buf_en && !din_fe_buf_addr[10]),//sample_buf0 /sample_buf1 does't be used
    .CLK(clk),
    .Q(samp_buf_0_q)
);

//synopsys translate_on

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
logic   [59:0]  buf_wem_a;
logic   [59:0]  buf_a_q;
assign  buf_wem_a   =   {
    {6{buf_wbe_a[6]}}, //same as buf_wbe_a[6] {not [7]}
    {8{buf_wbe_a[6]}},
    {8{buf_wbe_a[5]}},
    {8{buf_wbe_a[4]}},
    {6{buf_wbe_a[2]}}, //same as buf_wbe_a[2] {not [3]}
    {8{buf_wbe_a[2]}},
    {8{buf_wbe_a[1]}},
    {8{buf_wbe_a[0]}}
};

//synopsys translate_off
sram_model  #(
    .WIDTH(60),
    .DEPTH(512),
    .ADDR(512) )
    buf_a_512x60(
    .ADR(buf_addr_a[8:0]),
    //.D({10'b0,din_fe_buf_addr[9:0]}),
    .D(buf_wdata_a[59:0]),
    .WEM(buf_wem_a[59:0]),
    .WE(buf_we_a),
    .ME(buf_en_a),//sample_buf0 /sample_buf1 does't be used
    .CLK(clk),
    .Q(buf_a_q)
);

//synopsys translate_on

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
logic   [59:0]  buf_wem_b;
logic   [59:0]  buf_b_q;
assign  buf_wem_b   =   {
    {6{buf_wbe_b[6]}}, //same as buf_wbe_b[6] {not [7]}
    {8{buf_wbe_b[6]}},
    {8{buf_wbe_b[5]}},
    {8{buf_wbe_b[4]}},
    {6{buf_wbe_b[2]}}, //same as buf_wbe_b[2] {not [3]}
    {8{buf_wbe_b[2]}},
    {8{buf_wbe_b[1]}},
    {8{buf_wbe_b[0]}}
};

//synopsys translate_off
sram_model  #(
    .WIDTH(60),
    .DEPTH(512),
    .ADDR(512) )
    buf_b_512x60(
    .ADR(buf_addr_b[8:0]),
    //.D({10'b0,din_fe_buf_bddr[9:0]}),
    .D(buf_wdata_b[59:0]),
    .WEM(buf_wem_b[59:0]),
    .WE(buf_we_b),
    .ME(buf_en_b),//sample_buf0 /sample_buf1 does't be used
    .CLK(clk),
    .Q(buf_b_q)
);

//synopsys translate_on

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
logic   [47:0]  tab_wem;
logic   [47:0]  tab_mem_out;
logic   [2*W+1:0] const_tab_out;
logic   [0:0]   mem_sel_d;

assign  tab_wem   =   {
    {8{tab_wbe[6]}},
    {8{tab_wbe[5]}},
    {8{tab_wbe[4]}},
    {8{tab_wbe[2]}},
    {8{tab_wbe[1]}},
    {8{tab_wbe[0]}}
};

//synopsys translate_off
sram_model  #(
    .WIDTH(48),
    .DEPTH(512),
    .ADDR(512) )
    tab_512x48(
    .ADR(tab_addr[8:0]),
    //.D({10'b0,din_fe_tabddr[9:0]}),
    .D(tab_wdata[47:0]),
    .WEM(tab_wem[47:0]),
    .WE(tab_we),
    .ME(tab_en),
    .CLK(clk),
    .Q(tab_mem_out)
);

//synopsys translate_on

const_tab #(
    .W(W)
) const_tab(
    .clk(clk),
    .rst_n(rst_n),
    .a(tab_addr[8:0]),
    .en(tab_en && ~tab_addr[9]),//const_tab uses same addr with tab_512x48. But the addr[9] is unsame.
    .const_tab_out(const_tab_out)
);

always_ff @(posedge clk or negedge rst_n) begin
    if(rst_n)begin
        mem_sel_d   <= '0;
    end
    else begin
        mem_sel_d   <= tab_addr[9];
    end
end 

assign  tab_rdata    =  mem_sel_d ? tab_mem_out : const_tab_out;
//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
logic   [0:0]   reg_aud_dsp_busy_d; 
logic   [0:0]   aud_dsp_intr;
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        reg_aud_dsp_busy_d  <=  '0;
    end
    else
        reg_aud_dsp_busy_d  <=  reg_aud_dsp_busy;
end 

assign  aud_dsp_intr    =   !reg_aud_dsp_busy && reg_aud_dsp_busy_d; //used by 2ch_handle!!!

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        dsp_done    <= '0;
        cmd_dsp_done<= '0;
    end
    else if(aud_dsp_intr && reg_2ch_handle) begin
        dsp_done    <= 1'b1; //dsp done is a pulse ,it is a symbol of dsp handle a frame data.
        cmd_dsp_done <= cmd_dsp_done +1'b1;
    end
    else if(!aud_dsp_intr && reg_2ch_handle)
        dsp_done    <= '0;
end 

// interrupt handling 

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        intr_status <= '0;
    end
    else begin
        if(aud_in_fe_intr)
            intr_status[0]  <= 1'b1;
        else if(clr_intr_status[0])
            intr_status[0]  <=  1'b0;
        if(aud_dsp_intr)
            intr_status[1]  <=  1'b1;
        else if(clr_intr_status[1])
            intr_status[1]  <= 1'b0;
    end
end 

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        intr    <=  '0;
    end
    else
        intr    <=  (intr_status[0] & intr_mask[0] )| (intr_status[1] & intr_mask[1]);
end 

//-------------------------------------------------------------------------------------------------------
endmodule
