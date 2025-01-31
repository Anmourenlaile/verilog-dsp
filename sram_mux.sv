// *********************************************************************************
// Project Name : zkx2024
// Author       : Jlan
// Email        : 15533610762@163.com
// Create Time  : 2024-07-02
// File Name    : sram_mux.sv
// Module Name  : sram_mux
// Called By    : jlan
// Abstract     : sram_mux for 2req to 1req
//
// 
// *********************************************************************************
// Modification History:
// Date         By              Version                 Change Description
// -----------------------------------------------------------------------
// 2024-07-02    Macro           1.0                     Original
//  
// *********************************************************************************

module sram_mux #(
    parameter DW = 16,
    parameter AW = 9
) (
    input   [0:0]clk,
    input   [0:0]rst_n,

    input   [AW-1:0]addr_0, //req 0
    input   [0:0]en_0,
    input   [0:0]we_0,
    input   [7:0]wbe_0,
    input   [2*DW-1:0]wdata_0,
    output  logic   [2*DW-1:0]rdata_0,
   
    input   [AW-1:0]addr_1, //req 1 low priority be used for bus
    input   [0:0]en_1,
    input   [0:0]we_1,
    input   [7:0]wbe_1,
    input   [2*DW-1:0]wdata_1,
    output  logic    [2*DW-1:0]rdata_1,
    output  logic    [0:0]rdata_1_vld,

    
    output  logic   [AW-1:0]sram_addr, //req(be granted)
    output  logic   [0:0]sram_en,
    output  logic   [0:0]sram_we,
    output  logic   [7:0]sram_wbe,
    output  logic   [2*DW-1:0]sram_wdata,
    input          [2*DW-1:0]sram_rdata
);


//-----------------------------------------------------------------
logic   [AW-1:0]    addr_1_pending;
logic   [0:0]       en_1_pending;
logic   [0:0]       we_1_pending;
logic   [7:0]       wbe_pending;
logic   [2*DW-1:0]  wdata_1_pending;

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        addr_1_pending  <= '0;  
        en_1_pending    <= '0;
        we_1_pending    <= '0;
        wbe_pending     <= '0;
        wdata_1_pending <= '0;
    end
    else if( en_1 && en_0)begin
        addr_1_pending  <=  addr_1;    
        en_1_pending    <=  1'b1;
        we_1_pending    <=  we_1;
        wbe_pending     <=  wbe_1;
        wdata_1_pending <=  wdata_1;
    end
    else if(!en_0)begin
        en_1_pending    <=  '0;
    end
end 

assign  sram_addr   =   en_0 ? addr_0 :
    (en_1 ? addr_1 : (en_1_pending ? addr_1_pending : '0));

assign  sram_en     =   en_0 | en_1 | en_1_pending;

assign  sram_we     =   en_0 ? we_0 :
    (en_1 ? we_1  : (en_1_pending ? we_1_pending : '0 ));
assign  sram_wbe     =   en_0 ? wbe_0 :
    (en_1 ? wbe_1  : (en_1_pending ? wbe_1_pending : '0 ));
assign  sram_wdata  =   en_0 ? wdata_0 :
    (en_1 ? wdata_1  : (en_1_pending ? wdata_1_pending : '0 ));

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        rdata_1_vld <= '0;
    end
//    else if(en_0 && en_1)
//        rdata_1_vld <= '0;
//    else if(!en_0 && en_1)
//        rdata_1_vld <=  1'b1;
//    else if(en_1_pending)
//        rdata_1_vld <=  1'b1;
//    else
//        rdata_1_vld <= 1'b0;
    else
        rdata_1_vld <=  (en_1_pending || en_1 ) && (!en_0);
end 

assign  rdata_0 = sram_rdata;
assign  rdata_1 = sram_rdata;
//-----------------------------------------------------------------
endmodule

