// *********************************************************************************
// Project Name : zkx2024
// Author       : Jlan
// Email        : 15533610762@163.com
// Create Time  : 2024-07-05
// File Name    : cmd_proc.sv
// Module Name  : cmd_proc
// Called By    : jlan
// Abstract     : store fixed comand for pdm sample and audio feature extract!
//
// 
// *********************************************************************************
// Modification History:
// Date         By              Version                 Change Description
// -----------------------------------------------------------------------
// 2024-07-05    Macro           1.0                     Original
//  
// *********************************************************************************
module cmd_proc(
    input   clk,
    input   rst_n,

    input   [0:0]   reg_aud_dsp_start,
    output  logic[0:0]   reg_aud_dsp_busy,// not define
    input   [5:0]   reg_aud_dsp_op,
    input   [0:0]   reg_aud_dsp_in_buf_sel_mode,
    input   [0:0]   reg_aud_dsp_in_buf_sel,
    input   [5:0]   reg_aud_dsp_in_quant_bits,
    input   [7:0]   reg_aud_dsp_in_buf_offset,
    input   [7:0]   reg_aud_dsp_out_buf_offset,
    input   [7:0]   reg_aud_dsp_coef_buf_offset,
    input   [31:0]  reg_aud_dsp_param0,
    input   [0:0]   cmd_dsp_done,
    input   [0:0]   reg_2ch_handle,
    output  logic[0:0]    reg_cmd_start, // not define
    input   [0:0]   reg_cmd_busy, //not define
    output  logic[5:0]    reg_cmd_op,
    output  logic[0:0]reg_cmd_in_buf_sel_mode,//not difine
    output  logic[0:0]reg_cmd_in_buf_sel,
    output  logic [5:0]reg_cmd_in_quant_bits,
    output  logic [7:0]reg_cmd_in_buf_offset,
    output  logic [7:0]reg_cmd_out_buf_offset,
    output  logic [7:0]reg_cmd_coef_buf_offset,
    output  logic [31:0]reg_cmd_param0
);

//----------------------------------------------------------------------------
///feature extract comnd definition
//[5:0] reg_aud_dsp_op
//[6]reg aud dsp in buf_sel_mode
//[7]reg aud dsp in buf sel
//[13:8]reg aud dsp in quant bits
//[21:14]reg aud dsp in buf offset
//[29:22]reg_aud dsp_out buf offset
//[37:30]reg_aud dsp_coef buf _offset
//[69:38]reg aud dsp param0  

logic   [69:0]  cmd_ham_win = (cmd_dsp_done && reg_2ch_handle) ?
    ({
    {6'h0, reg_aud_dsp_param0[25:9], 9'h 1ff}, // k=0.97 ,vector_len=512
    8'h10,  //coef_buf_offest
    8'h0,   //out_buf_offest
    8'h0,   //in_buf_offest
    reg_cmd_in_quant_bits,
    !reg_aud_dsp_in_buf_sel,
    reg_cmd_in_buf_sel_mode,
    6'h0}): // ham_win
    ({
    {6'h0, reg_aud_dsp_param0[25:9], 9'h 1ff}, // k=0.97 ,vector_len=512
    8'h10,  //coef_buf_offest
    8'h0,   //out_buf_offest
    8'h0,   //in_buf_offest
    reg_cmd_in_quant_bits,
    reg_aud_dsp_in_buf_sel,
    reg_cmd_in_buf_sel_mode,
    6'h0}); // ham_win

logic   [69:0]  cmd_fft = {
    {27'h0 , reg_aud_dsp_param0[4] , 4'd9}, //quant input ,512-point fft
    8'h80,//coef_buf_offest
    8'h0, //out_buf_offest 
    8'h0, //in_buf_offest  
    6'h0,// use previous result!!!!
    1'b0,// use previous result!!!!
    1'b0,// use previous result!!!!
    6'h20 // op: fft
};

logic   [69:0]  cmd_mfcc = {
    {2'h0,reg_aud_dsp_param0[29:26],17'h0,9'd255}, //mel map vector_len = 256
    reg_aud_dsp_coef_buf_offset,
    8'h0,
    8'h0,
    6'h0,
    1'b0,
    1'b0,
    6'h04
};

//----------------------------------------------------------------------------
logic   [7:0]   cmd_rom_0_addr;
logic   [70:0]  cmd_rom_0_rdata;

always_comb begin
    cmd_rom_0_rdata =   0;
    case(cmd_rom_0_addr)
        0: cmd_rom_0_rdata = { 1'b0, cmd_ham_win};
        1: cmd_rom_0_rdata = { 1'b0, cmd_fft};
        2: cmd_rom_0_rdata = { 1'b0, cmd_mfcc};
        3: cmd_rom_0_rdata = { 1'b0, 70'h0};
    endcase
end 
//----------------------------------------------------------------------------
typedef enum {
    CMD_IDLE,
    CMD_LOAD,
    CMD_EXEC
}   cmd_state_t;

cmd_state_t cmd_state,cmd_state_next;

logic   [7:0]   cmd_addr;
logic   [70:0]  cmd_rdata;
logic   [0:0]   cmd_exec_end;
assign  cmd_rom_0_addr  =   cmd_addr;
assign  cmd_rdata   =   cmd_rom_0_rdata;
assign  cmd_exec_end = !reg_cmd_busy;

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        cmd_state   <= CMD_IDLE;
    else
        cmd_state   <= cmd_state_next;
end 
always_comb begin
    cmd_state_next  =   cmd_state;
    case(cmd_state)
        CMD_IDLE:
            if(reg_cmd_start && (reg_aud_dsp_op == 'h30))
                cmd_state_next = CMD_LOAD;
        CMD_LOAD:
            if(cmd_rdata[70])
                cmd_state_next = CMD_IDLE;
            else
                cmd_state_next = CMD_EXEC;
        CMD_EXEC:
            if(cmd_exec_end)
                cmd_state_next = CMD_LOAD;
    endcase
end

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        reg_aud_dsp_busy    <= '0;
    end
    else if((cmd_state == CMD_IDLE) && reg_aud_dsp_start && (reg_aud_dsp_op =='h30))
        reg_aud_dsp_busy    <= 1'b1;
    else if((cmd_state == CMD_LOAD) && cmd_rdata[70])
        reg_aud_dsp_busy    <= 1'b0;
end

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        cmd_addr    <= '0;
    end
    else if(cmd_state == CMD_IDLE)
        cmd_addr    <= '0;
    else if((cmd_state == CMD_EXEC) && cmd_exec_end)
        cmd_addr    <= cmd_addr + 1'b1;
end 

always_comb begin
    reg_cmd_start = (cmd_state == CMD_LOAD) && !cmd_rdata[70];
end 

assign  reg_cmd_op  =   cmd_rdata[5:0];
assign  reg_cmd_in_buf_sel_mode =   cmd_rdata[6];
assign  reg_cmd_in_buf_sel  =   cmd_rdata[7];
assign  reg_cmd_in_quant_bits   =   cmd_rdata[13:8];
assign  reg_cmd_in_buf_offset   =   cmd_rdata[21:14];
assign  reg_cmd_out_buf_offset  =   cmd_rdata[29:22];
assign  reg_cmd_coef_buf_offset =   cmd_rdata[37:30];
assign  reg_cmd_param0  =   cmd_rdata[69:38];
//----------------------------------------------------------------------------
endmodule

