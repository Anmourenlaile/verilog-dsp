// *********************************************************************************
// Project Name : zkx2024
// Author       : Jlan
// Email        : 15533610762@163.com
// Create Time  : 2024-07-02
// File Name    : aud_in_fe.sv
// Module Name  : aud_in_fe.sv
// Called By    : jlan
// Abstract     : aud_in_fe: sample&store pdm data in auto model.
//
// 
// *********************************************************************************
// Modification History:
// Date         By              Version                 Change Description
// -----------------------------------------------------------------------
// 2024-07-02    Macro           1.0                     Original
//  
// *********************************************************************************

module aud_in_fe #(
    parameter   W = 16,
    parameter   W_BUF = 22,
    parameter   CH = 2,
    parameter   S_BUF_AW = 10
) (
    input   clk,
    input   rst_n,
    input   [W*CH-1:0]  aud_data_in,
    input   [0:0]   aud_data_valid,

    input   [1:0]   reg_ch_mode, //0:1ch ; 1: 2ch  ; 3:4ch;

    input   reg_sample_save_en, //0:disable; 1:enable 
    input   reg_aud_auto_pre_proc_en,
    input   [1:0]   reg_aapp_ch_sel,    // audio channel number of which feature to be extract
    input   [0:0]   reg_dat_store_mode, //o:block 1:interleave

    input   [7:0]   reg_aapp_fft_size, //FFT size ,= (d+1)*16
    input   [7:0]   reg_frame_size, // frame size, = (d+1)*16
    input   [0:0]   reg_frame_mode, // 0:full frame ;1:half frame
    input   [15:0]  reg_aapp_buf_base,

    input   [5:0]   reg_aud_dsp_in_quant_bits, 
    input   [0:0]   reg_2ch_handle,
    input   [0:0]   reg_dsp_done, // 1bit pulse when dsp_done!!
    

    output  logic   [S_BUF_AW-1:0]  sample_buf_addr,
    output  logic   [0:0]           sample_buf_en,
    output  logic   [0:0]           sample_buf_we,
    output  logic   [7:0]           sample_buf_wbe,
    output  logic   [2*W-1:0]       sample_buf_wdata,

    input   [2*W-1:0]   sample_buf_rdata,

    XBAR_TCDM_BUS.Master mst,

    output  logic   start_aapp,
    output  logic   aapp_quant_input,// the signal means that the pdm data  overflowing almostly. the vector will left shifter it ????? 
    output  logic   reg_ready_blk_id,// indicate which sample buffer.now only in sample buf0
    output  logic   intr
);  
//-----------------------------------------------------------------------------
logic   [11:0]  aapp_fft_size_up_lim = {reg_aapp_fft_size[7:0], 4'hf};
logic   [11:0]  frame_size_up_lim   =   {reg_frame_size[7:0], 4'hf};

//-----------------------------------------------------------------------------
typedef enum {
    SAVE_IDLE,
    WAIT_DAT_0,
    WAIT_DAT_1,
    WR_DAT_0,
    WR_DAT_1
}   dat_save_state_t; //save pdm data from vad to sample buffer!!

dat_save_state_t    dat_save_state, dat_save_state_next;

//-----------------------------------------------------------------------------
typedef enum{
    MOVE_IDLE,
    MOVE_DAT_RD,
    MOVE_DAT_WR0,
    MOVE_DAT_WR1,
    MOVE_DAT_WR2,
    MOVE_DAT_WR3,
    MOVE_DAT_WR_VLD,
    MOVE_FILL_ZERO
}   move_dat_state_t; //move pdm data from sample buffer to bufa/b when one frame pdm data done!

move_dat_state_t    move_dat_state,move_dat_state_next;

//-----------------------------------------------------------------------------
typedef enum{
    FILL_ZERO_S0,  //Filing zero sub_state 0
    FILL_ZERO_S1,   //Filing zero sub_state 1
    FILL_ZERO_WR_VLD //waiting for 2nd 0 writing to be finished.
}   fill_zero_sub_state_t;

fill_zero_sub_state_t   fill_zero_sub_state, fill_zero_sub_state_next;

//-----------------------------------------------------------------------------

logic   [11:0]  save_sample_cnt; // sample date for a frame.
logic   [1:0]   sample_wr_cnt; // stroe one pdm data cnt(one pdm data has 2ch data)
logic   [2:0]   ch_num = reg_ch_mode + 1'b1;
logic   [0:0]   blk_done = (save_sample_cnt == frame_size_up_lim); //store a full frame.
logic   [0:0]   aapp_blk_done = (reg_frame_mode ? (save_sample_cnt == {1'b0, frame_size_up_lim[11:1]}):1'b0) | blk_done;
logic   [0;0]   wr_rd_contention = ((dat_save_state == WR_DAT_0) || (dat_save_state == WR_DAT_1)) && (move_dat_state == MOVE_DAT_RD); //store pdm to sample buf content with reading(read then write to bufa/b)  pdm data from sample_buf.
logic   [0:0]   wr_dat_done =   !wr_rd_contention && (sample_wr_cnt == reg_ch_mode); // write done a pdm data.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
logic   [0:0]   reg_sample_save_en_d;
logic   [0:0]   reg_sample_save_en_falling = reg_sample_save_en_d && !reg_sample_save_en;
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        reg_sample_save_en_d    <= '0;
    end
    else
        reg_sample_save_en_d    <= reg_sample_save_en;
end 
//-----------------------------------------------------------------------------
logic   [1:0]   mv_dat_rdy_blk_id;
logic   [0:0]   dsp_done_d;

always_ff @(posedge clk or negedge rst_n) begin:dsp_done_cnt
    if(!rst_n)begin
        dsp_done    <= '0;
    end
    else if(reg_2ch_handle && reg_dsp_done)
        dsp_done    <= !dsp_done;
end 
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        dsp_done_d  <='0;
    end
    else
        dsp_done_d  <= dsp_done;
end 


//-----------------------------------------------------------------------------

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        dat_save_state  <= SAVE_IDLE;
    end
    else
        dat_save_state  <= dat_save_state_next;
end 
//Store sample data to sample buf .The difference between DAT0 and DAT1 is FRAME 0 AND FRAME1.
//The sample buffer can store 2 frame 2ch data ,if the frame size is 512.
always_comb begin
    dat_save_state_next = dat_save_state; //aovid latch!
    if(!reg_sample_save_en)
        dat_save_state_next =   SAVE_IDLE;
    else begin
        unique case(dat_save_state)
            SAVE_IDLE:
                dat_save_state_next = WR_DAT_0;
            WAIT_DAT_0:
                if(aud_data_valid && save_sample_cnt[0]) // WHY [0]?
                    dat_save_state_next = WR_DAT_0;
            WR_DAT_0:
                if(wr_dat_done)
                    if(blk_done)
                        dat_save_state_next = WAIT_DAT_1;
                    else
                        dat_save_state_next = WAIT_DAT_0;
            WAIT_DAT_1:
                if(aud_data_valid && save_sample_cnt[0])
                    dat_save_state_next = WR_DAT_1;
            WR_DAT_1:
                if(wr_dat_done)
                    if(blk_done)
                        dat_save_state_next = WAIT_DAT_0;
                    else
                        dat_save_state_next = WAIT_DAT_1;
        endcase
    end
end 


always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        save_sample_cnt <= '0;
    end
    else if(reg_sample_save_en_falling)
        save_sample_cnt <= '0;
    else if(aud_data_valid && !save_sample_cnt[0] && ((dat_save_state == WAIT_DAT_0) || (dat_save_state == WAIT_DAT_1)))
        save_sample_cnt <= save_sample_cnt + 1'b1;
    else if(wr_dat_done && ((dat_save_state == WR_DAT_0) || (dat_save_state == WR_DAT_1)))begin
        if(blk_done)
            save_sample_cnt <= '0;
        else
            save_sample_cnt <= save_sample_cnt +1'b1;
    end
end 

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        sample_wr_cnt   <= '0;
    else if(reg_sample_save_en_falling)
        sample_wr_cnt   <= '0;
    else if((dat_save_state == WR_DAT_0) || (dat_save_state == WR_DAT_1))begin
        if(!wr_rd_contention)
            sample_wr_cnt   <= sample_wr_cnt + 1'b1;
    end
    else 
            sample_wr_cnt   <= '0;
end 

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
logic   [11:0]  move_sample_cnt;
logic   [0:0]   move_dat_done   =   mst.r_valid && (move_sample_cnt == (frame_size_up_lim-1'b1));
logic   [0:0]   fill_zero_done  =   mst.r_valid && (move_sample_cnt == aapp_fft_size_up_lim) && (fill_zero_sub_state == FILL_ZERO_WR_VLD);

logic   [0:0]   need_fill_zero  =   reg_aapp_fft_size != reg_frame_size ; 
logic   [0:0]   start_move_dat  =   ((dat_save_state == WR_DAT_0) || (dat_save_state == WR_DAT_1)) &&
        wr_dat_done && aapp_blk_done || (!dsp_done_d && dsp_done); //start move one frame to bufa/b !!

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        move_dat_state  <= MOVE_IDLE;
    else
        move_dat_state  <= move_dat_state_next;
end 

always_comb begin
    move_dat_state_next = move_dat_state;
    if(!reg_aud_auto_pre_proc_en || !reg_sample_save_en)
        move_dat_state_next =   MOVE_IDLE;
    else begin
        case(move_dat_state)
            MOVE_IDLE:
                if(start_move_dat)
                    move_dat_state_next = MOVE_DAT_RD; //read 2* 48bit data
                else
                    move_dat_state_next = MOVE_IDLE;
            MOVE_DAT_RD:
                    move_dat_state_next = MOVE_DAT_WR0;//write 1*30bit(24+6) data
            MOVE_DAT_WR0:
                    if(mst.gnt)
                        move_dat_state_next = MOVE_DAT_WR1;
            MOVE_DAT_WR1:
                    if(mst.gnt)
                        move_dat_state_next = MOVE_DAT_WR2;
            MOVE_DAT_WR2:
                    if(mst.gnt)
                        move_dat_state_next = MOVE_DAT_WR3;
            MOVE_DAT_WR3:
                    if(mst.gnt)
                        move_dat_state_next = MOVE_DAT_WR_VLD; // 
            MOVE_DAT_WR_VLD:
                if(move_dat_done)
                begin
                    if(need_fill_zero)
                        move_dat_state_next = MOVE_FILL_ZERO;
                    else
                        move_dat_state_next = MOVE_IDLE;
                end
                    else
                        move_dat_state_next = MOVE_DAT_RD;
            MOVE_FILL_ZERO:
                if((fill_zero_sub_state == FILL_ZERO_WR_VLD) && fill_zero_done)
                    move_dat_state_next = MOVE_IDLE;
        endcase
    end
end 

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        fill_zero_sub_state    <= FILL_ZERO_S0;
    end
    else 
        fill_zero_sub_state    <= fill_zero_sub_state_next;
end 

always_comb begin
    fill_zero_sub_state_next =  fill_zero_sub_state;
    case(move_dat_state)
        MOVE_IDLE:
            fill_zero_sub_state_next    =   FILL_ZERO_S0;
        MOVE_DAT_WR_VLD:
            fill_zero_sub_state_next    =   FILL_ZERO_S0;
        MOVE_FILL_ZERO:
            case(fill_zero_sub_state)
                FILL_ZERO_S0:
                    if(mst.gnt)
                        fill_zero_sub_state_next    =   FILL_ZERO_S1;
                FILL_ZERO_S1:
                    if(mst.gnt)
                        fill_zero_sub_state_next    =   FILL_ZERO_WR_VLD;
                FILL_ZERO_WR_VLD:
                    if(mst.r_valid)
                        fill_zero_sub_state_next    =   FILL_ZERO_S0;
                endcase
    endcase
end 

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        move_sample_cnt <=  '0;
    else if(move_dat_state == MOVE_IDLE)
        move_dat_state  <=  '0;
    else if((move_dat_state == MOVE_DAT_WR_VLD) && mst.r_valid)
        move_sample_cnt <=  move_sample_cnt + 12'h2; // A writing behavior writes 2 pdm datas.
    else if((move_sample_cnt == MOVE_FILL_ZERO) && (fill_zero_sub_state == FILL_ZERO_WR_VLD) && mst.r_valid)
        move_sample_cnt <= move_sample_cnt + 12'h1;
end 
//-----------------------------------------------------------------------------
// sample wdata generate

logic   [W*CH*2-1:0]    aud_data_in_save; // A storing behavior stores CH*2 Pdm data.
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        aud_data_in_save    <= '0;
    end
    else if(aud_data_valid)
        aud_data_in_save    <= {aud_data_in_save[W*CH-1:0] , aud_data_in_save[W*CH-1:0] };
end 

logic   [12:0]  frame_size = ({reg_frame_size[7:0],4'hf} + 1'b1);

int i ,j ;
always_comb begin
    sample_buf_wdata = 0;
    for(i=0 ;i<CH ;i=i+1)begin
        if(i == $signed({1'b0,sample_wr_cnt}))begin
            for(j=0 ;j<W;j=j+1)
                sample_buf_wdata[j] = {aud_data_in_save[W*CH+W*i+j]};
            for(j=W;j<W*2;j=j+1)
                sample_buf_wdata[j] = {aud_data_in_save[W*i+j-W]};
        end
    end
end 
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//sample addr/en/we/wbe generate 
logic   [0:0]   aapp_ch_sel;
assign  aapp_ch_sel =   reg_aapp_ch_sel + dsp_done; //in 2ch handle mode , the second ch is not the aapp_ch_sel 

always_comb begin
    sample_buf_addr =   '0;
    sample_buf_en   =   '0;
    sample_buf_we   =   '0;
    sample_buf_wbe  =   '0;

    if(move_dat_state == MOVE_DAT_RD) //generate addr for reading  
    begin
        case(mv_dat_rdy_blk_id)
            2'h0:
                if(move_sample_cnt[11:1] < frame_size[12:2])
                    sample_buf_addr = reg_aapp_ch_sel * frame_size + frame_size[12:1] + frame_size[12:2] + move_sample_cnt[11:1];
                else
                    sample_buf_addr = reg_aapp_ch_sel * frame_size + move_sample_cnt[11:1] - frame_size[12:2];
            2'h1:
                sample_buf_addr =   aapp_ch_sel * frame_size + move_sample_cnt[11:1]; // be changed!
            2'h2:
                sample_buf_addr =   reg_aapp_ch_sel * frame_size + frame_size[12:2] + move_sample_cnt[11:1];
            2'h3:
                sample_buf_addr =   aapp_ch_sel * frame_size + frame_size[12:1] + move_sample_cnt[11:1];
        endcase
        sample_buf_en   =   !move_sample_cnt[0];
        sample_buf_we   =   1'b0;
    end 
    else if( dat_save_state == WR_DAT_0)  //generate addr for writing the odd frame into block0;
    begin
        sample_buf_addr =   sample_wr_cnt * frame_size + save_sample_cnt[11:1];
        sample_buf_en   =   1'b1;
        sample_buf_we   =   1'b1;
        sample_buf_wbe  =   2'h3;
    end
    else if(dat_save_state == WR_DAT_1)  //generate addr for writing the even frame into block1;
    begin
        sample_buf_addr =   sample_wr_cnt * frame_size  + frame_size[12:1] + save_sample_cnt[11:1];
        sample_buf_en   =   1'b1;
        sample_buf_we   =   1'b1;
        sample_buf_wbe  =   2'h3;
    end
end 

//-----------------------------------------------------------------------------
logic   [0:0]   sample_buf_rd_d;
logic   [W*2-1:0]   sample_buf_rdata_save;

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        sample_buf_rd_d <= '0;
    else
        sample_buf_rd_d <=  sample_buf_en && !sample_buf_we;
end 

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        sample_buf_rdata_save   <= '0;
    end
    else if(sample_buf_rd_d)
        sample_buf_rdata_save   <= sample_buf_rdata;
end 

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        mst.req <=  '0;
    else if((move_dat_state  ==  MOVE_DAT_RD) || (move_dat_state == MOVE_DAT_WR0) || (move_dat_state == MOVE_DAT_WR1) || (move_dat_state == MOVE_DAT_WR2))
        mst.req <=  1'b1;
    else if((move_dat_state == MOVE_DAT_WR3) && mst.gnt)
        mst.req <=  1'b0;
    else if(move_dat_state == MOVE_FILL_ZERO)
        if(fill_zero_sub_state == FILL_ZERO_WR_VLD)
            if(mst.r_rvalid)
                if(fill_zero_done)
                    mst.req <= 1'b0;
                else
                    mst.req <= 1'b1;
            else
                mst.req <= 1'b0;
        else
            mst.req <= 1'b1;
    else if(!reg_aud_auto_pre_proc_en || !reg_sample_save_en)
    begin
        if(mst.req && mst.gnt)
            mst.req <= 1'b0;
    end

end 

always_comb begin
    case(move_dat_state)
        MOVE_DAT_WR0:mst.add = reg_aapp_buf_base[15] ? (dsp_done ? (16'h4000 + move_sample_cnt*8):(16'h8000+move_sample_cnt*8)): (dsp_done ? (16'h8000 + move_sample_cnt*8):(16'h4000+move_sample_cnt*8)); 
        MOVE_DAT_WR1:mst.add = reg_aapp_buf_base[15] ? (dsp_done ? (16'h4000 + move_sample_cnt*8+4):(16'h8000+move_sample_cnt*8+4)): (dsp_done ? (16'h8000 + move_sample_cnt*8+4):(16'h4000+move_sample_cnt*8+4)); 
        MOVE_DAT_WR0:mst.add = reg_aapp_buf_base[15] ? (dsp_done ? (16'h4000 + move_sample_cnt*8):(16'h8000+move_sample_cnt*8)): (dsp_done ? (16'h8000 + move_sample_cnt*8):(16'h4000+move_sample_cnt*8)); 
        MOVE_FILL_ZERO:mst.add = (fill_zero_sub_state == FILL_ZERO_S0)? (reg_aapp_buf_base[15] ? (dsp_done ? (16'h4000 + move_sample_cnt*8):(16'h8000+move_sample_cnt*8)): (dsp_done ? (16'h8000 + move_sample_cnt*8):(16'h4000+move_sample_cnt*8))): (reg_aapp_buf_base[15] ? (dsp_done ? (16'h4000 + move_sample_cnt*8+4):(16'h8000+move_sample_cnt*8+4)): (dsp_done ? (16'h8000 + move_sample_cnt*8+4):(16'h4000+move_sample_cnt*8+4)));
        default:mst.add = reg_aapp_buf_base[15] ? (dsp_done ? (16'h4000 + move_sample_cnt*8+12):(16'h8000+move_sample_cnt*8+12)): (dsp_done ? (16'h8000 + move_sample_cnt*8+12):(16'h4000+move_sample_cnt*8+12));
    endcase
    mst.wen = 1'b1;
    mst.be  = 4'hf;
    if(move_dat_state == MOVE_DAT_WR0)
        mst.wdata = sample_buf_rd_d ? {10'h0, reg_aud_dsp_in_quant_bits[5:0],sample_buf_rdata[W-1:0]} : {10'h0, reg_aud_dsp_in_quant_bits[5:0],sample_buf_rdata_save[W-1:0]};
    else if(move_dat_state == MOVE_DAT_WR2)
        mst.wdata = {10'h0,reg_aud_dsp_in_quant_bits[5:0],sample_buf_rdata_save[W*2-1:W]};
    else
        mst.wdata = 0;
end 


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        start_aapp  <= '0;
    end
    else if((move_dat_state == MOVE_DAT_WR_VLD) && move_dat_done && 
    !need_fill_zero ||(move_dat_state == MOVE_FILL_ZERO) && fill_zero_done)
    start_aapp  <= 1'b1;
    else
    start_aapp  <= 1'b0;
end 

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        aapp_quant_input    <= '0;
    else if(sample_buf_rd_d && ((sample_buf_rdata[W-1]^sample_buf_rdata[W-2])))
        aapp_quant_input    <= 1'b1;
    else if((move_dat_state == MOVE_IDLE) && start_move_dat)
        aapp_quant_input    <= 1'b0;
end 

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        reg_ready_blk_id    <= 1'b0;
end //be changed by jlan in 2024/7/5.
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        mv_dat_rdy_blk_id   <= '0;
    else if((dat_save_state == WR_DAT_0) && wr_dat_done && aapp_blk_done)
        mv_dat_rdy_blk_id   <= blk_done ? 2'h1 : 2'h0;
    else if((dat_save_state == WR_DAT_1) && wr_dat_done && aapp_blk_done)
        mv_dat_rdy_blk_id   <= blk_done ? 2'h3 : 2'h2;
end

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        intr    <= '0;
    else if(((dat_save_state == WR_DAT_0) || (dat_save_state == WR_DAT_1) && wr_dat_done && blk_done))
        intr    <= 1'b1;
    else
        intr    <= 1'b0;
end 
//-----------------------------------------------------------------------------
endmodule
