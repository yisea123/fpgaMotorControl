// (C) 2001-2017 Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License Subscription 
// Agreement, Intel MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Intel and sold by 
// Intel or its authorized distributors.  Please refer to the applicable 
// agreement for further details.



// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// $Id: //acds/rel/17.0std/ip/merlin/altera_merlin_router/altera_merlin_router.sv.terp#1 $
// $Revision: #1 $
// $Date: 2017/01/22 $
// $Author: swbranch $

// -------------------------------------------------------
// Merlin Router
//
// Asserts the appropriate one-hot encoded channel based on 
// either (a) the address or (b) the dest id. The DECODER_TYPE
// parameter controls this behaviour. 0 means address decoder,
// 1 means dest id decoder.
//
// In the case of (a), it also sets the destination id.
// -------------------------------------------------------

`timescale 1 ns / 1 ns

module soc_system_mm_interconnect_0_router_003_default_decode
  #(
     parameter DEFAULT_CHANNEL = 0,
               DEFAULT_WR_CHANNEL = -1,
               DEFAULT_RD_CHANNEL = -1,
               DEFAULT_DESTID = 0 
   )
  (output [110 - 105 : 0] default_destination_id,
   output [51-1 : 0] default_wr_channel,
   output [51-1 : 0] default_rd_channel,
   output [51-1 : 0] default_src_channel
  );

  assign default_destination_id = 
    DEFAULT_DESTID[110 - 105 : 0];

  generate
    if (DEFAULT_CHANNEL == -1) begin : no_default_channel_assignment
      assign default_src_channel = '0;
    end
    else begin : default_channel_assignment
      assign default_src_channel = 51'b1 << DEFAULT_CHANNEL;
    end
  endgenerate

  generate
    if (DEFAULT_RD_CHANNEL == -1) begin : no_default_rw_channel_assignment
      assign default_wr_channel = '0;
      assign default_rd_channel = '0;
    end
    else begin : default_rw_channel_assignment
      assign default_wr_channel = 51'b1 << DEFAULT_WR_CHANNEL;
      assign default_rd_channel = 51'b1 << DEFAULT_RD_CHANNEL;
    end
  endgenerate

endmodule


module soc_system_mm_interconnect_0_router_003
(
    // -------------------
    // Clock & Reset
    // -------------------
    input clk,
    input reset,

    // -------------------
    // Command Sink (Input)
    // -------------------
    input                       sink_valid,
    input  [135-1 : 0]    sink_data,
    input                       sink_startofpacket,
    input                       sink_endofpacket,
    output                      sink_ready,

    // -------------------
    // Command Source (Output)
    // -------------------
    output                          src_valid,
    output reg [135-1    : 0] src_data,
    output reg [51-1 : 0] src_channel,
    output                          src_startofpacket,
    output                          src_endofpacket,
    input                           src_ready
);

    // -------------------------------------------------------
    // Local parameters and variables
    // -------------------------------------------------------
    localparam PKT_ADDR_H = 67;
    localparam PKT_ADDR_L = 36;
    localparam PKT_DEST_ID_H = 110;
    localparam PKT_DEST_ID_L = 105;
    localparam PKT_PROTECTION_H = 125;
    localparam PKT_PROTECTION_L = 123;
    localparam ST_DATA_W = 135;
    localparam ST_CHANNEL_W = 51;
    localparam DECODER_TYPE = 0;

    localparam PKT_TRANS_WRITE = 70;
    localparam PKT_TRANS_READ  = 71;

    localparam PKT_ADDR_W = PKT_ADDR_H-PKT_ADDR_L + 1;
    localparam PKT_DEST_ID_W = PKT_DEST_ID_H-PKT_DEST_ID_L + 1;



    // -------------------------------------------------------
    // Figure out the number of bits to mask off for each slave span
    // during address decoding
    // -------------------------------------------------------
    localparam PAD0 = log2ceil(64'h10008 - 64'h10000); 
    localparam PAD1 = log2ceil(64'h10050 - 64'h10040); 
    localparam PAD2 = log2ceil(64'h10090 - 64'h10080); 
    localparam PAD3 = log2ceil(64'h100d0 - 64'h100c0); 
    localparam PAD4 = log2ceil(64'h20008 - 64'h20000); 
    localparam PAD5 = log2ceil(64'h20060 - 64'h20050); 
    localparam PAD6 = log2ceil(64'h20170 - 64'h20160); 
    localparam PAD7 = log2ceil(64'h20280 - 64'h20270); 
    localparam PAD8 = log2ceil(64'h20390 - 64'h20380); 
    localparam PAD9 = log2ceil(64'h204a0 - 64'h20490); 
    localparam PAD10 = log2ceil(64'h20510 - 64'h20500); 
    localparam PAD11 = log2ceil(64'h20620 - 64'h20610); 
    localparam PAD12 = log2ceil(64'h20730 - 64'h20720); 
    localparam PAD13 = log2ceil(64'h20840 - 64'h20830); 
    localparam PAD14 = log2ceil(64'h20950 - 64'h20940); 
    localparam PAD15 = log2ceil(64'h21060 - 64'h21050); 
    localparam PAD16 = log2ceil(64'h21070 - 64'h21060); 
    localparam PAD17 = log2ceil(64'h21080 - 64'h21070); 
    localparam PAD18 = log2ceil(64'h21090 - 64'h21080); 
    localparam PAD19 = log2ceil(64'h210a0 - 64'h21090); 
    localparam PAD20 = log2ceil(64'h21170 - 64'h21160); 
    localparam PAD21 = log2ceil(64'h21280 - 64'h21270); 
    localparam PAD22 = log2ceil(64'h21390 - 64'h21380); 
    localparam PAD23 = log2ceil(64'h214a0 - 64'h21490); 
    localparam PAD24 = log2ceil(64'h21510 - 64'h21500); 
    localparam PAD25 = log2ceil(64'h21630 - 64'h21620); 
    localparam PAD26 = log2ceil(64'h21740 - 64'h21730); 
    localparam PAD27 = log2ceil(64'h21850 - 64'h21840); 
    localparam PAD28 = log2ceil(64'h21960 - 64'h21950); 
    localparam PAD29 = log2ceil(64'h22070 - 64'h22060); 
    localparam PAD30 = log2ceil(64'h22180 - 64'h22170); 
    localparam PAD31 = log2ceil(64'h22290 - 64'h22280); 
    localparam PAD32 = log2ceil(64'h223a0 - 64'h22390); 
    localparam PAD33 = log2ceil(64'h22410 - 64'h22400); 
    localparam PAD34 = log2ceil(64'h22520 - 64'h22510); 
    localparam PAD35 = log2ceil(64'h22630 - 64'h22620); 
    localparam PAD36 = log2ceil(64'h22740 - 64'h22730); 
    localparam PAD37 = log2ceil(64'h22850 - 64'h22840); 
    localparam PAD38 = log2ceil(64'h22960 - 64'h22950); 
    localparam PAD39 = log2ceil(64'h23070 - 64'h23060); 
    localparam PAD40 = log2ceil(64'h23180 - 64'h23170); 
    localparam PAD41 = log2ceil(64'h23290 - 64'h23280); 
    localparam PAD42 = log2ceil(64'h233a0 - 64'h23390); 
    localparam PAD43 = log2ceil(64'h23410 - 64'h23400); 
    localparam PAD44 = log2ceil(64'h23520 - 64'h23510); 
    localparam PAD45 = log2ceil(64'h23620 - 64'h23610); 
    localparam PAD46 = log2ceil(64'h23720 - 64'h23710); 
    localparam PAD47 = log2ceil(64'h23820 - 64'h23810); 
    localparam PAD48 = log2ceil(64'h23920 - 64'h23900); 
    // -------------------------------------------------------
    // Work out which address bits are significant based on the
    // address range of the slaves. If the required width is too
    // large or too small, we use the address field width instead.
    // -------------------------------------------------------
    localparam ADDR_RANGE = 64'h23920;
    localparam RANGE_ADDR_WIDTH = log2ceil(ADDR_RANGE);
    localparam OPTIMIZED_ADDR_H = (RANGE_ADDR_WIDTH > PKT_ADDR_W) ||
                                  (RANGE_ADDR_WIDTH == 0) ?
                                        PKT_ADDR_H :
                                        PKT_ADDR_L + RANGE_ADDR_WIDTH - 1;

    localparam RG = RANGE_ADDR_WIDTH-1;
    localparam REAL_ADDRESS_RANGE = OPTIMIZED_ADDR_H - PKT_ADDR_L;

      reg [PKT_ADDR_W-1 : 0] address;
      always @* begin
        address = {PKT_ADDR_W{1'b0}};
        address [REAL_ADDRESS_RANGE:0] = sink_data[OPTIMIZED_ADDR_H : PKT_ADDR_L];
      end   

    // -------------------------------------------------------
    // Pass almost everything through, untouched
    // -------------------------------------------------------
    assign sink_ready        = src_ready;
    assign src_valid         = sink_valid;
    assign src_startofpacket = sink_startofpacket;
    assign src_endofpacket   = sink_endofpacket;
    wire [PKT_DEST_ID_W-1:0] default_destid;
    wire [51-1 : 0] default_src_channel;




    // -------------------------------------------------------
    // Write and read transaction signals
    // -------------------------------------------------------
    wire read_transaction;
    assign read_transaction  = sink_data[PKT_TRANS_READ];


    soc_system_mm_interconnect_0_router_003_default_decode the_default_decode(
      .default_destination_id (default_destid),
      .default_wr_channel   (),
      .default_rd_channel   (),
      .default_src_channel  (default_src_channel)
    );

    always @* begin
        src_data    = sink_data;
        src_channel = default_src_channel;
        src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = default_destid;

        // --------------------------------------------------
        // Address Decoder
        // Sets the channel and destination ID based on the address
        // --------------------------------------------------

    // ( 0x10000 .. 0x10008 )
    if ( {address[RG:PAD0],{PAD0{1'b0}}} == 18'h10000  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000000000000000000100;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 50;
    end

    // ( 0x10040 .. 0x10050 )
    if ( {address[RG:PAD1],{PAD1{1'b0}}} == 18'h10040   ) begin
            src_channel = 51'b0000000000000000000000000000000000000000001000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 9;
    end

    // ( 0x10080 .. 0x10090 )
    if ( {address[RG:PAD2],{PAD2{1'b0}}} == 18'h10080   ) begin
            src_channel = 51'b0000000000000000000000000000000000000000000001000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 2;
    end

    // ( 0x100c0 .. 0x100d0 )
    if ( {address[RG:PAD3],{PAD3{1'b0}}} == 18'h100c0   ) begin
            src_channel = 51'b0000000000000000000000000000000000000000000010000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 1;
    end

    // ( 0x20000 .. 0x20008 )
    if ( {address[RG:PAD4],{PAD4{1'b0}}} == 18'h20000   ) begin
            src_channel = 51'b0000000000000000000000000000000000000000000000010;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 8;
    end

    // ( 0x20050 .. 0x20060 )
    if ( {address[RG:PAD5],{PAD5{1'b0}}} == 18'h20050   ) begin
            src_channel = 51'b0000000000000000000000000000000000000000000100000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 4;
    end

    // ( 0x20160 .. 0x20170 )
    if ( {address[RG:PAD6],{PAD6{1'b0}}} == 18'h20160   ) begin
            src_channel = 51'b0000000000000010000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 5;
    end

    // ( 0x20270 .. 0x20280 )
    if ( {address[RG:PAD7],{PAD7{1'b0}}} == 18'h20270   ) begin
            src_channel = 51'b0000000000000000000000000000000000000000100000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 49;
    end

    // ( 0x20380 .. 0x20390 )
    if ( {address[RG:PAD8],{PAD8{1'b0}}} == 18'h20380  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000000000000010000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 37;
    end

    // ( 0x20490 .. 0x204a0 )
    if ( {address[RG:PAD9],{PAD9{1'b0}}} == 18'h20490  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000000000100000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 40;
    end

    // ( 0x20500 .. 0x20510 )
    if ( {address[RG:PAD10],{PAD10{1'b0}}} == 18'h20500  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000000001000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 41;
    end

    // ( 0x20610 .. 0x20620 )
    if ( {address[RG:PAD11],{PAD11{1'b0}}} == 18'h20610  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000000010000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 42;
    end

    // ( 0x20720 .. 0x20730 )
    if ( {address[RG:PAD12],{PAD12{1'b0}}} == 18'h20720  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000000100000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 43;
    end

    // ( 0x20830 .. 0x20840 )
    if ( {address[RG:PAD13],{PAD13{1'b0}}} == 18'h20830  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000001000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 44;
    end

    // ( 0x20940 .. 0x20950 )
    if ( {address[RG:PAD14],{PAD14{1'b0}}} == 18'h20940  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000010000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 45;
    end

    // ( 0x21050 .. 0x21060 )
    if ( {address[RG:PAD15],{PAD15{1'b0}}} == 18'h21050  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000100000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 46;
    end

    // ( 0x21060 .. 0x21070 )
    if ( {address[RG:PAD16],{PAD16{1'b0}}} == 18'h21060  && read_transaction  ) begin
            src_channel = 51'b0000010000000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 47;
    end

    // ( 0x21070 .. 0x21080 )
    if ( {address[RG:PAD17],{PAD17{1'b0}}} == 18'h21070  && read_transaction  ) begin
            src_channel = 51'b0000100000000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 48;
    end

    // ( 0x21080 .. 0x21090 )
    if ( {address[RG:PAD18],{PAD18{1'b0}}} == 18'h21080  && read_transaction  ) begin
            src_channel = 51'b0001000000000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 38;
    end

    // ( 0x21090 .. 0x210a0 )
    if ( {address[RG:PAD19],{PAD19{1'b0}}} == 18'h21090  && read_transaction  ) begin
            src_channel = 51'b0010000000000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 39;
    end

    // ( 0x21160 .. 0x21170 )
    if ( {address[RG:PAD20],{PAD20{1'b0}}} == 18'h21160   ) begin
            src_channel = 51'b0000000000000000000000000000000000000010000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 20;
    end

    // ( 0x21270 .. 0x21280 )
    if ( {address[RG:PAD21],{PAD21{1'b0}}} == 18'h21270   ) begin
            src_channel = 51'b0000000000000000000000000000001000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 21;
    end

    // ( 0x21380 .. 0x21390 )
    if ( {address[RG:PAD22],{PAD22{1'b0}}} == 18'h21380   ) begin
            src_channel = 51'b0000000000000000000000000000010000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 22;
    end

    // ( 0x21490 .. 0x214a0 )
    if ( {address[RG:PAD23],{PAD23{1'b0}}} == 18'h21490   ) begin
            src_channel = 51'b0000000000000000000000000000100000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 23;
    end

    // ( 0x21500 .. 0x21510 )
    if ( {address[RG:PAD24],{PAD24{1'b0}}} == 18'h21500   ) begin
            src_channel = 51'b0000000000000000000000000001000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 24;
    end

    // ( 0x21620 .. 0x21630 )
    if ( {address[RG:PAD25],{PAD25{1'b0}}} == 18'h21620   ) begin
            src_channel = 51'b0000000000000000000000000010000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 25;
    end

    // ( 0x21730 .. 0x21740 )
    if ( {address[RG:PAD26],{PAD26{1'b0}}} == 18'h21730   ) begin
            src_channel = 51'b0000000000000000000000000100000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 26;
    end

    // ( 0x21840 .. 0x21850 )
    if ( {address[RG:PAD27],{PAD27{1'b0}}} == 18'h21840   ) begin
            src_channel = 51'b0000000000000000000000001000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 27;
    end

    // ( 0x21950 .. 0x21960 )
    if ( {address[RG:PAD28],{PAD28{1'b0}}} == 18'h21950  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000000000000000000001000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 12;
    end

    // ( 0x22060 .. 0x22070 )
    if ( {address[RG:PAD29],{PAD29{1'b0}}} == 18'h22060  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000010000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 13;
    end

    // ( 0x22170 .. 0x22180 )
    if ( {address[RG:PAD30],{PAD30{1'b0}}} == 18'h22170  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000000100000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 14;
    end

    // ( 0x22280 .. 0x22290 )
    if ( {address[RG:PAD31],{PAD31{1'b0}}} == 18'h22280  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000001000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 15;
    end

    // ( 0x22390 .. 0x223a0 )
    if ( {address[RG:PAD32],{PAD32{1'b0}}} == 18'h22390  && read_transaction  ) begin
            src_channel = 51'b0000000000000000010000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 16;
    end

    // ( 0x22400 .. 0x22410 )
    if ( {address[RG:PAD33],{PAD33{1'b0}}} == 18'h22400  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000010000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 17;
    end

    // ( 0x22510 .. 0x22520 )
    if ( {address[RG:PAD34],{PAD34{1'b0}}} == 18'h22510  && read_transaction  ) begin
            src_channel = 51'b0000000000000000000100000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 18;
    end

    // ( 0x22620 .. 0x22630 )
    if ( {address[RG:PAD35],{PAD35{1'b0}}} == 18'h22620  && read_transaction  ) begin
            src_channel = 51'b0000000000000000001000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 19;
    end

    // ( 0x22730 .. 0x22740 )
    if ( {address[RG:PAD36],{PAD36{1'b0}}} == 18'h22730   ) begin
            src_channel = 51'b0000000000000000100000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 28;
    end

    // ( 0x22840 .. 0x22850 )
    if ( {address[RG:PAD37],{PAD37{1'b0}}} == 18'h22840   ) begin
            src_channel = 51'b0000000000000001000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 29;
    end

    // ( 0x22950 .. 0x22960 )
    if ( {address[RG:PAD38],{PAD38{1'b0}}} == 18'h22950   ) begin
            src_channel = 51'b0000000000000100000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 30;
    end

    // ( 0x23060 .. 0x23070 )
    if ( {address[RG:PAD39],{PAD39{1'b0}}} == 18'h23060   ) begin
            src_channel = 51'b0000000000001000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 31;
    end

    // ( 0x23170 .. 0x23180 )
    if ( {address[RG:PAD40],{PAD40{1'b0}}} == 18'h23170   ) begin
            src_channel = 51'b0000000000010000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 32;
    end

    // ( 0x23280 .. 0x23290 )
    if ( {address[RG:PAD41],{PAD41{1'b0}}} == 18'h23280   ) begin
            src_channel = 51'b0000000000100000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 33;
    end

    // ( 0x23390 .. 0x233a0 )
    if ( {address[RG:PAD42],{PAD42{1'b0}}} == 18'h23390   ) begin
            src_channel = 51'b0000000001000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 34;
    end

    // ( 0x23400 .. 0x23410 )
    if ( {address[RG:PAD43],{PAD43{1'b0}}} == 18'h23400   ) begin
            src_channel = 51'b0000000010000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 35;
    end

    // ( 0x23510 .. 0x23520 )
    if ( {address[RG:PAD44],{PAD44{1'b0}}} == 18'h23510   ) begin
            src_channel = 51'b0000000100000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 36;
    end

    // ( 0x23610 .. 0x23620 )
    if ( {address[RG:PAD45],{PAD45{1'b0}}} == 18'h23610  && read_transaction  ) begin
            src_channel = 51'b0000001000000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 10;
    end

    // ( 0x23710 .. 0x23720 )
    if ( {address[RG:PAD46],{PAD46{1'b0}}} == 18'h23710   ) begin
            src_channel = 51'b0100000000000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 6;
    end

    // ( 0x23810 .. 0x23820 )
    if ( {address[RG:PAD47],{PAD47{1'b0}}} == 18'h23810  && read_transaction  ) begin
            src_channel = 51'b1000000000000000000000000000000000000000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 3;
    end

    // ( 0x23900 .. 0x23920 )
    if ( {address[RG:PAD48],{PAD48{1'b0}}} == 18'h23900   ) begin
            src_channel = 51'b0000000000000000000000000000000000000000000000001;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 0;
    end

end


    // --------------------------------------------------
    // Ceil(log2()) function
    // --------------------------------------------------
    function integer log2ceil;
        input reg[65:0] val;
        reg [65:0] i;

        begin
            i = 1;
            log2ceil = 0;

            while (i < val) begin
                log2ceil = log2ceil + 1;
                i = i << 1;
            end
        end
    endfunction

endmodule


