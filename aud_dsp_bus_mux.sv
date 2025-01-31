// *********************************************************************************
// Project Name : zkx2024
// Author       : Jlan
// Email        : 15533610762@163.com
// Create Time  : 2024-07-01
// File Name    : aud_dsp_bus_mux.v
// Module Name  : aud_dsp_bus_mux
// Called By    : aud_dsp_bus_mux
// Abstract     : aud_dsp_bus_mux
//
// 
// *********************************************************************************
// Modification History:
// Date         By              Version                 Change Description
// -----------------------------------------------------------------------
// 2024-07-01    Macro           1.0                     Original
//  
// *********************************************************************************
module aud_dsp_bus_mux#(
    parameter   W = 16,
    parameter   W_BUF = 22,
    parameter   S_BUF_AW =10,
    parameter   AUD_DSP_ADDR_MASK = 32'h0000_ffff,
    parameter   SAMP_BUF_BASE = 32'h0000_0000,
    parameter   BUF_A_BASE = 32'h0000_4000,
    parameter   BUF_B_BASE = 32'h0000_8000,
    parameter   COEF_BUF_BASE = 32'h0000_c000,
    parameter   SAMP_BUF_SIZE = 32'h4000,
    parameter   BUF_A_SIZE = 32'h1000,
    parameter   BUF_B_SIZE = 32'h1000,
    parameter   COEF_BUF_SIZE = 32'h2000
) (
    input   clk,
    input   rst_n,

    input   reg_rdata_sign_ext, //what mean?

    input   [31:0]  h_addr,
    input   [0:0]   h_req_vld,
    output  [0:0]   h_req_rdy,
    input   [0:0]   h_we,     
    input   [3:0]   h_wbe,    
    input   [31:0]  h_wdata,  
    output  [31:0]  h_rdata,  
    output  [0:0]   h_vld,

    input   [S_BUF_AW-1:0]  sample_buf_addr,
    input   [0:0]   sample_buf_en,
    input   [0:0]   sample_buf_we,
    input   [1:0]   sample_buf_wbe,//2BIT one hot !!!
    input   [2*W-1:0]   sample_buf_wdata,
    output  [2*W-1:0]   sample_buf_rdata,

    input   [8:0]   core_buf_addr_a,
    input   [0:0]   core_buf_en_a,
    input   [0:0]   core_buf_we_a,
    input   [1:0]   core_buf_wbe_a,
    input   [2*W_BUF-1:0]   core_buf_wdata_a,
    output  [2*W_BUF-1:0]   core_buf_rdata_a,

   
    input   [8:0]   core_buf_addr_b,
    input   [0:0]   core_buf_en_b,
    input   [0:0]   core_buf_we_b,
    input   [1:0]   core_buf_wbe_b,
    input   [2*W_BUF-1:0]   core_buf_wdata_b,
    output  [2*W_BUF-1:0]   core_buf_rdata_b,

    input   [9:0]   core_tab_addr,
    input   [0:0]   core_tab_en,
    output  [2*W-1:0]   core_tab_rdata,


    output  [S_BUF_AW-1:0]  din_fe_buf_addr,
    output  [0:0]   din_fe_buf_en,
    output  [0:0]   din_fe_buf_we,
    output  [7:0]   din_fe_buf_wbe,
    output  [2*W-1:0]   din_fe_buf_wdata,
    input   [2*W-1:0]   din_fe_buf_rdata,


    output  [8:0]   buf_addr_a,
    output  [0:0]   buf_en_a,
    output  [0:0]   buf_we_a,
    output  [7:0]   buf_wbe_a,
    output  [2*W_BUF-1:0]   buf_wdata_a,
    input   [2*W_BUF-1:0]   buf_rdata_a,

   
    output  [8:0]   buf_addr_b,
    output  [0:0]   buf_en_b,
    output  [0:0]   buf_we_b,
    output  [7:0]   buf_wbe_b,
    output  [2*W_BUF-1:0]   buf_wdata_b,
    input   [2*W_BUF-1:0]   buf_rdata_b,


    output  [9:0]   tab_addr,
    output  [0:0]   tab_en,
    output  [0:0]   tab_we,
    output  [7:0]   tab_wbe,
    output  [2*W-1:0]   tab_wdata,
    input   [2*W-1:0]   tab_rdata



);
//-------------------------------------------------------------------
logic   [31:0]  h_offset;
logic   [0:0]   s_buf_sel;
logic   [0:0]   buf_a_sel;
logic   [0:0]   buf_b_sel;
logic   [0:0]   tab_buf_sel;

assign  h_offset    =   h_addr & AUD_DSP_ADDR_MASK;
assign  s_buf_sel   =   (h_offset >= SAMP_BUF_BASE) && (h_offset < (SAMP_BUF_BASE + SAMP_BUF_SIZE));
assign  buf_a_sel   =   (h_offset >= BUF_A_BASE)    && (h_offset < (BUF_A_BASE + BUF_A_SIZE));
assign  buf_b_sel   =   (h_offset >= BUF_B_BASE)    && (h_offset < (BUF_B_BASE + BUF_B_SIZE));
assign  tab_buf_sel =   (h_offset >= COEF_BUF_BASE) && (h_offset < (COEF_BUF_BASE + COEF_BUF_SIZE));
//-------------------------------------------------------------------
logic   [0:0]   req_pending;
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        req_pending <= '0;
    end
    else if(h_req_vld)
        req_pending <=  1'b1;
    else if(h_vld)
        req_pending <= 1'b0;
end 

assign  h_req_rdy   =   h_req_vld && !(req_pending && !h_vld); //why ????

//-------------------------------------------------------------------
logic   [S_BUF_AW-1:0]  s_buf_addr  =   h_offset[S_BUF_AW+2:3];
logic   [0:0]   s_buf_en    =   s_buf_sel && h_req_vld;
logic   [0:0]   s_buf_we    =   s_buf_sel && h_we;
logic   [7:0]   s_buf_wbe   =   h_offset[2] ? {h_wbe[3:0],4'h0} : {4'h0,h_wbe[3:0]}; //
logic   [2*W-1:0] s_buf_wdata = {h_wdata[W-1:0],h_wdata[W-1:0]};
logic   [2*W-1:0] s_buf_rdata;
logic   [0:0]   s_buf_rdata_vld;
//-------------------------------------------------------------------
logic   [8:0]  bus_buf_addr_a  =   h_offset[11:3];
logic   [0:0]   bus_buf_en_a    =   buf_a_sel && h_req_vld;
logic   [0:0]   bus_buf_we_a    =   buf_a_sel && h_we;
logic   [7:0]   bus_buf_wbe_a   =   h_offset[2] ? {h_wbe[3:0],4'h0} : {4'h0,h_wbe[3:0]}; //
logic   [2*W_BUF-1:0] bus_buf_wdata_a = {h_wdata[W_BUF-1:0],h_wdata[W_BUF-1:0]};
logic   [2*W_BUF-1:0] bus_buf_rdata_a;
logic   [0:0]   bus_buf_rdata_a_vld;
//-------------------------------------------------------------------
logic   [8:0]  bus_buf_addr_b  =   h_offset[11:3];
logic   [0:0]   bus_buf_en_b    =   buf_b_sel && h_req_vld;
logic   [0:0]   bus_buf_we_b    =   buf_b_sel && h_we;
logic   [7:0]   bus_buf_wbe_b   =   h_offset[2] ? {h_wbe[3:0],4'h0} : {4'h0,h_wbe[3:0]}; //
logic   [2*W_BUF-1:0] bus_buf_wdata_b = {h_wdata[W_BUF-1:0],h_wdata[W_BUF-1:0]};
logic   [2*W_BUF-1:0] bus_buf_rdata_b;
logic   [0:0]   bus_buf_rdata_b_vld;
//-------------------------------------------------------------------
logic   [9:0]  bus_tab_addr  =   h_offset[12:3];
logic   [0:0]   bus_tab_en    =   tab_buf_sel && h_req_vld;
logic   [0:0]   bus_tab_we    =   tab_buf_sel && h_we;
logic   [7:0]   bus_tab_wbe   =   h_offset[2] ? {h_wbe[3:0],4'h0} : {4'h0,h_wbe[3:0]}; //
logic   [2*W-1:0] bus_tab_wdata = {h_wdata[W-1:0],h_wdata[W-1:0]};
logic   [2*W-1:0] bus_tab_rdata;
logic   [0:0]   bus_tab_rdata_a_vld;
//-------------------------------------------------------------------
logic   [0:0]   bus_addr_0_d;
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        bus_addr_0_d    <= '0;
    end
    else if(h_req_rdy)
        bus_addr_0_d    <= h_offset[2];
end 

logic   [W_BUF-1:0] h_rdata_0 = s_buf_rdata_vld ? (bus_addr_0_d ? {{(W_BUF-W){1'b0}},s_buf_rdata[2*W-1:W]} : {{(W_BUF-W){1'b0}},s_buf_rdata[W-1:0]}) :
        bus_buf_rdata_a_vld ? (bus_addr_0_d ? bus_buf_rdata_a[2*W_BUF-1:W_BUF] : bus_buf_rdata_a[W_BUF-1:0]):
        bus_buf_rdata_b_vld ? (bus_addr_0_d ? bus_buf_rdata_b[2*W_BUF-1:W_BUF] : bus_buf_rdata_b[W_BUF-1:0]):
        bus_tab_rdata_vld ? (bus_addr_0_d ?  {{(W_BUF-W){1'b0}},bus_tab_rdata[2*W-1:W]} : {{(W_BUF-W){1'b0}},bus_tab_rdata[W-1:0]});


// if data in bufa/b ,it has quant  so the sign extension needed special handle.
//  The quant is sign extented. The data's sign extension is done by vector_core or fft_core.

//assign  h_rdata =   {bus_tab_rdata_a_vld | bus_tab_rdata_b_vld} ? {{(32-W_BUF){1'b0}},h_rdata_0} :
//                    reg_rdata_sign_ext  ?   {{(32-W){h_rdata_0[W-1]}},h_rdata_0[W-1:0]} : {{(32-W){1'b0}},h_rdata_0[W-1:0]}; 
assign  h_rdata =   {bus_tab_rdata_a_vld | bus_tab_rdata_b_vld} ? {{(32-W_BUF){h_rdata_0[W_BUF-1]}},h_rdata_0} :
                    reg_rdata_sign_ext  ?   {{(32-W){h_rdata_0[W-1]}},h_rdata_0[W-1:0]} : {{(32-W){1'b0}},h_rdata_0[W-1:0]}; 

assign  h_vld   =   bus_tab_rdata_b_vld | bus_tab_rdata_a_vld | s_buf_rdata_vld | bus_tab_rdata_vld;
//-------------------------------------------------------------------
sram_mux #(
    .DW(W),
    .AW(S_BUF_AW)
) sram_mux_s_buf(
    .clk(clk),
    .rst_n(rst_n),
    
    .addr_0(sample_buf_addr[S_BUF_AW-1:0]), //req 0
    .en_0(sample_buf_en),
    .we_0(sample_buf_we),
    .wbe_0({{4{sample_buf_wbe[1]}},{4{sample_buf_wbe[0]}}}),
    .wdata_0(sample_buf_wdata[2*W-1:0]),
    .rdata_0(sample_buf_rdata[2*W-1:0]),
    
   
    .addr_1(s_buf_addr[S_BUF_AW-1:0]), //req 1
    .en_1(s_buf_en),
    .we_1(s_buf_we),
    .wbe_1(s_buf_wbe[7:0]),
    .wdata_1(s_buf_wdata[2*W-1:0]),
    .rdata_1(s_buf_rdata[2*W-1:0]),
    .rdata_1_vld(s_buf_rdata_vld),

    
    .sram_addr(din_fe_buf_addr[S_BUF_AW-1:0]), //req(be granted)
    .sram_en(din_fe_buf_en),
    .sram_we(din_fe_buf_we),
    .sram_wbe(din_fe_buf_wbe[7:0]),
    .sram_wdata(din_fe_buf_wdata[2*W-1:0]),
    .sram_rdata(din_fe_buf_rdata[2*W-1:0])
);
//-------------------------------------------------------------------\

//which is the bus that aud_in_fe  writes into the bufa/b?
sram_mux #(
    .DW(W_BUF),
    .AW(9)
) sram_mux_buf_a(
    .clk(clk),
    .rst_n(rst_n),
    
    .addr_0(core_buf_addr_a[8:0]), //req 0
    .en_0(core_buf_en_a),
    .we_0(core_buf_we_a),
    .wbe_0({{4{core_buf_wbe_a[1]}},{4{core_buf_wbe_a[0]}}}),
    .wdata_0(core_buf_wdata_a[2*W_BUF-1:0]),
    .rdata_0(core_buf_rdata_a[2*W_BUF-1:0]),
    
   
    .addr_1(bus_buf_addr_a[8:0]), //req 1
    .en_1(bus_buf_en_a),
    .we_1(bus_buf_we_a),
    .wbe_1(bus_buf_wbe_a[7:0]),
    .wdata_1(bus_buf_wdata_a[2*W_BUF-1:0]),
    .rdata_1(bus_buf_rdata_a[2*W_BUF-1:0]),
    .rdata_1_vld(bus_buf_rdata_a_vld),

    
    .sram_addr(buf_addr_a[8:0]), //req(be granted)
    .sram_en(buf_en_a),
    .sram_we(buf_we_a),
    .sram_wbe(buf_wbe_a[7:0]),
    .sram_wdata(buf_wdata_a[2*W_BUF-1:0]),
    .sram_rdata(buf_rdata_a[2*W_BUF-1:0])
);


sram_mux #(
    .DW(W_BUF),
    .AW(9)
) sram_mux_buf_b(
    .clk(clk),
    .rst_n(rst_n),
    
    .addr_0(core_buf_addr_b[8:0]), //req 0
    .en_0(core_buf_en_b),
    .we_0(core_buf_we_b),
    .wbe_0({{4{core_buf_wbe_b[1]}},{4{core_buf_wbe_b[0]}}}),
    .wdata_0(core_buf_wdata_b[2*W_BUF-1:0]),
    .rdata_0(core_buf_rdata_b[2*W_BUF-1:0]),
    
   
    .addr_1(bus_buf_addr_b[8:0]), //req 1
    .en_1(bus_buf_en_b),
    .we_1(bus_buf_we_b),
    .wbe_1(bus_buf_wbe_b[7:0]),
    .wdata_1(bus_buf_wdata_b[2*W_BUF-1:0]),
    .rdata_1(bus_buf_rdata_b[2*W_BUF-1:0]),
    .rdata_1_vld(bus_buf_rdata_b_vld),

    
    .sram_bddr(buf_addr_b[8:0]), //req(be granted)
    .sram_en(buf_en_b),
    .sram_we(buf_we_b),
    .sram_wbe(buf_wbe_b[7:0]),
    .sram_wdata(buf_wdata_b[2*W_BUF-1:0]),
    .sram_rdata(buf_rdata_b[2*W_BUF-1:0])
);


sram_mux #(
    .DW(W_BUF),
    .AW(10)
) sram_mux_tab_buf(
    .clk(clk),
    .rst_n(rst_n),
    
    .addr_0(core_tab_addr[9:0]), //req 0
    .en_0(core_tab_en),
    .we_0(0),
    .wbe_0(0),
    .wdata_0(0),
    .rdata_0(core_tab_rdata[2*W-1:0]),
    
   
    .addr_1(bus_tab_addr[9:0]), //req 1
    .en_1(bus_tab_en),
    .we_1(bus_tab_we),
    .wbe_1(bus_tab_wbe[7:0]),
    .wdata_1(bus_tab_wdata[2*W-1:0]),
    .rdata_1(bus_tab_rdata[2*W-1:0]),
    .rdata_1_vld(bus_tab_rdata_vld),

    
    .sramddr(tab_addr[9:0]), //req(be granted)
    .sram_en(tab_en),
    .sram_we(tab_we),
    .sram_wbe(tab_wbe[7:0]),
    .sram_wdata(tab_wdata[2*W-1:0]),
    .sram_rdata(tab_rdata[2*W-1:0])
);

//-------------------------------------------------------------------

endmodule

