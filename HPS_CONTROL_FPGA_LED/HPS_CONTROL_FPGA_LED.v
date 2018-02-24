// ============================================================================
// Copyright (c) 2014 by Terasic Technologies Inc.
// ============================================================================
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// ============================================================================
//           
//  Terasic Technologies Inc
//  9F., No.176, Sec.2, Gongdao 5th Rd, East Dist, Hsinchu City, 30070. Taiwan
//  
//  
//                     web: http://www.terasic.com/  
//                     email: support@terasic.com
//
// ============================================================================
//Date:  Tue Dec  2 09:28:38 2014
// ============================================================================

`define ENABLE_HPS
//`define ENABLE_CLK

module HPS_CONTROL_FPGA_LED(

      ///////// ADC /////////
      output             ADC_CONVST,
      output             ADC_SCK,
      output             ADC_SDI,
      input              ADC_SDO,

      ///////// ARDUINO /////////
      inout       [15:0] ARDUINO_IO,
      inout              ARDUINO_RESET_N,

`ifdef ENABLE_CLK
      ///////// CLK /////////
      output             CLK_I2C_SCL,
      inout              CLK_I2C_SDA,
`endif /*ENABLE_CLK*/

      ///////// FPGA /////////
      input              FPGA_CLK1_50,
      input              FPGA_CLK2_50,
      input              FPGA_CLK3_50,

      ///////// GPIO /////////
      inout       [35:0] GPIO_0,
      inout       [35:0] GPIO_1,

`ifdef ENABLE_HPS
      ///////// HPS /////////
      inout              HPS_CONV_USB_N,
      output      [14:0] HPS_DDR3_ADDR,
      output      [2:0]  HPS_DDR3_BA,
      output             HPS_DDR3_CAS_N,
      output             HPS_DDR3_CKE,
      output             HPS_DDR3_CK_N,
      output             HPS_DDR3_CK_P,
      output             HPS_DDR3_CS_N,
      output      [3:0]  HPS_DDR3_DM,
      inout       [31:0] HPS_DDR3_DQ,
      inout       [3:0]  HPS_DDR3_DQS_N,
      inout       [3:0]  HPS_DDR3_DQS_P,
      output             HPS_DDR3_ODT,
      output             HPS_DDR3_RAS_N,
      output             HPS_DDR3_RESET_N,
      input              HPS_DDR3_RZQ,
      output             HPS_DDR3_WE_N,
      output             HPS_ENET_GTX_CLK,
      inout              HPS_ENET_INT_N,
      output             HPS_ENET_MDC,
      inout              HPS_ENET_MDIO,
      input              HPS_ENET_RX_CLK,
      input       [3:0]  HPS_ENET_RX_DATA,
      input              HPS_ENET_RX_DV,
      output      [3:0]  HPS_ENET_TX_DATA,
      output             HPS_ENET_TX_EN,
      inout              HPS_GSENSOR_INT,
      inout              HPS_I2C0_SCLK,
      inout              HPS_I2C0_SDAT,
      inout              HPS_I2C1_SCLK,
      inout              HPS_I2C1_SDAT,
      inout              HPS_KEY,
      inout              HPS_LED,
      inout              HPS_LTC_GPIO,
      output             HPS_SD_CLK,
      inout              HPS_SD_CMD,
      inout       [3:0]  HPS_SD_DATA,
      output             HPS_SPIM_CLK,
      input              HPS_SPIM_MISO,
      output             HPS_SPIM_MOSI,
      inout              HPS_SPIM_SS,
      input              HPS_UART_RX,
      output             HPS_UART_TX,
      input              HPS_USB_CLKOUT,
      inout       [7:0]  HPS_USB_DATA,
      input              HPS_USB_DIR,
      input              HPS_USB_NXT,
      output             HPS_USB_STP,
`endif /*ENABLE_HPS*/

      ///////// KEY /////////
      input       [1:0]  KEY,

      ///////// LED /////////
      output      [7:0]  LED,
		
      ///////// SW /////////
      input       [3:0]  SW
);


//=======================================================
//  REG/WIRE declarations
//=======================================================
wire  hps_fpga_reset_n;
wire [31:0] enc_count[11:0];
//wire [31:0] enc_count[7:0];
wire [31:0] enc_reset;
wire [31:0] pid_error[7:0];
wire [31:0] pid_correction[7:0];
wire [31:0] pid_values;
wire [31:0] pwm_values[7:0];
wire [7:0] limit_switches;
//limit_switches[7:0] <= GPIO_1[23:16];

wire fpga_clk_5khz;
assign fpga_clk_50=FPGA_CLK1_50;



//=======================================================
//  Structural coding
//=======================================================


 soc_system u0 (
		//Clock&Reset
		.clk_clk                               (FPGA_CLK1_50 ),                               //                            clk.clk
		.reset_reset_n                         (1'b1         ),                         //                          reset.reset_n
		//HPS ddr3
		.memory_mem_a                          ( HPS_DDR3_ADDR),                       //                memory.mem_a
		.memory_mem_ba                         ( HPS_DDR3_BA),                         //                .mem_ba
		.memory_mem_ck                         ( HPS_DDR3_CK_P),                       //                .mem_ck
		.memory_mem_ck_n                       ( HPS_DDR3_CK_N),                       //                .mem_ck_n
		.memory_mem_cke                        ( HPS_DDR3_CKE),                        //                .mem_cke
		.memory_mem_cs_n                       ( HPS_DDR3_CS_N),                       //                .mem_cs_n
		.memory_mem_ras_n                      ( HPS_DDR3_RAS_N),                      //                .mem_ras_n
		.memory_mem_cas_n                      ( HPS_DDR3_CAS_N),                      //                .mem_cas_n
		.memory_mem_we_n                       ( HPS_DDR3_WE_N),                       //                .mem_we_n
		.memory_mem_reset_n                    ( HPS_DDR3_RESET_N),                    //                .mem_reset_n
		.memory_mem_dq                         ( HPS_DDR3_DQ),                         //                .mem_dq
		.memory_mem_dqs                        ( HPS_DDR3_DQS_P),                      //                .mem_dqs
		.memory_mem_dqs_n                      ( HPS_DDR3_DQS_N),                      //                .mem_dqs_n
		.memory_mem_odt                        ( HPS_DDR3_ODT),                        //                .mem_odt
		.memory_mem_dm                         ( HPS_DDR3_DM),                         //                .mem_dm
		.memory_oct_rzqin                      ( HPS_DDR3_RZQ),                        //                .oct_rzqin                                  
		//HPS ethernet		
		.hps_0_hps_io_hps_io_emac1_inst_TX_CLK ( HPS_ENET_GTX_CLK),       //                             hps_0_hps_io.hps_io_emac1_inst_TX_CLK
		.hps_0_hps_io_hps_io_emac1_inst_TXD0   ( HPS_ENET_TX_DATA[0] ),   //                             .hps_io_emac1_inst_TXD0
		.hps_0_hps_io_hps_io_emac1_inst_TXD1   ( HPS_ENET_TX_DATA[1] ),   //                             .hps_io_emac1_inst_TXD1
		.hps_0_hps_io_hps_io_emac1_inst_TXD2   ( HPS_ENET_TX_DATA[2] ),   //                             .hps_io_emac1_inst_TXD2
		.hps_0_hps_io_hps_io_emac1_inst_TXD3   ( HPS_ENET_TX_DATA[3] ),   //                             .hps_io_emac1_inst_TXD3
		.hps_0_hps_io_hps_io_emac1_inst_RXD0   ( HPS_ENET_RX_DATA[0] ),   //                             .hps_io_emac1_inst_RXD0
		.hps_0_hps_io_hps_io_emac1_inst_MDIO   ( HPS_ENET_MDIO ),         //                             .hps_io_emac1_inst_MDIO
		.hps_0_hps_io_hps_io_emac1_inst_MDC    ( HPS_ENET_MDC  ),         //                             .hps_io_emac1_inst_MDC
		.hps_0_hps_io_hps_io_emac1_inst_RX_CTL ( HPS_ENET_RX_DV),         //                             .hps_io_emac1_inst_RX_CTL
		.hps_0_hps_io_hps_io_emac1_inst_TX_CTL ( HPS_ENET_TX_EN),         //                             .hps_io_emac1_inst_TX_CTL
		.hps_0_hps_io_hps_io_emac1_inst_RX_CLK ( HPS_ENET_RX_CLK),        //                             .hps_io_emac1_inst_RX_CLK
		.hps_0_hps_io_hps_io_emac1_inst_RXD1   ( HPS_ENET_RX_DATA[1] ),   //                             .hps_io_emac1_inst_RXD1
		.hps_0_hps_io_hps_io_emac1_inst_RXD2   ( HPS_ENET_RX_DATA[2] ),   //                             .hps_io_emac1_inst_RXD2
		.hps_0_hps_io_hps_io_emac1_inst_RXD3   ( HPS_ENET_RX_DATA[3] ),   //                             .hps_io_emac1_inst_RXD3		  
		//HPS SD card 
		.hps_0_hps_io_hps_io_sdio_inst_CMD     ( HPS_SD_CMD    ),           //                               .hps_io_sdio_inst_CMD
		.hps_0_hps_io_hps_io_sdio_inst_D0      ( HPS_SD_DATA[0]     ),      //                               .hps_io_sdio_inst_D0
		.hps_0_hps_io_hps_io_sdio_inst_D1      ( HPS_SD_DATA[1]     ),      //                               .hps_io_sdio_inst_D1
		.hps_0_hps_io_hps_io_sdio_inst_CLK     ( HPS_SD_CLK   ),            //                               .hps_io_sdio_inst_CLK
		.hps_0_hps_io_hps_io_sdio_inst_D2      ( HPS_SD_DATA[2]     ),      //                               .hps_io_sdio_inst_D2
		.hps_0_hps_io_hps_io_sdio_inst_D3      ( HPS_SD_DATA[3]     ),      //                               .hps_io_sdio_inst_D3
		//HPS USB 		  
		.hps_0_hps_io_hps_io_usb1_inst_D0      ( HPS_USB_DATA[0]    ),      //                               .hps_io_usb1_inst_D0
		.hps_0_hps_io_hps_io_usb1_inst_D1      ( HPS_USB_DATA[1]    ),      //                               .hps_io_usb1_inst_D1
		.hps_0_hps_io_hps_io_usb1_inst_D2      ( HPS_USB_DATA[2]    ),      //                               .hps_io_usb1_inst_D2
		.hps_0_hps_io_hps_io_usb1_inst_D3      ( HPS_USB_DATA[3]    ),      //                               .hps_io_usb1_inst_D3
		.hps_0_hps_io_hps_io_usb1_inst_D4      ( HPS_USB_DATA[4]    ),      //                               .hps_io_usb1_inst_D4
		.hps_0_hps_io_hps_io_usb1_inst_D5      ( HPS_USB_DATA[5]    ),      //                               .hps_io_usb1_inst_D5
		.hps_0_hps_io_hps_io_usb1_inst_D6      ( HPS_USB_DATA[6]    ),      //                               .hps_io_usb1_inst_D6
		.hps_0_hps_io_hps_io_usb1_inst_D7      ( HPS_USB_DATA[7]    ),      //                               .hps_io_usb1_inst_D7
		.hps_0_hps_io_hps_io_usb1_inst_CLK     ( HPS_USB_CLKOUT    ),       //                               .hps_io_usb1_inst_CLK
		.hps_0_hps_io_hps_io_usb1_inst_STP     ( HPS_USB_STP    ),          //                               .hps_io_usb1_inst_STP
		.hps_0_hps_io_hps_io_usb1_inst_DIR     ( HPS_USB_DIR    ),          //                               .hps_io_usb1_inst_DIR
		.hps_0_hps_io_hps_io_usb1_inst_NXT     ( HPS_USB_NXT    ),          //                               .hps_io_usb1_inst_NXT
		//HPS SPI 		  
		.hps_0_hps_io_hps_io_spim1_inst_CLK    ( HPS_SPIM_CLK  ),           //                               .hps_io_spim1_inst_CLK
		.hps_0_hps_io_hps_io_spim1_inst_MOSI   ( HPS_SPIM_MOSI ),           //                               .hps_io_spim1_inst_MOSI
		.hps_0_hps_io_hps_io_spim1_inst_MISO   ( HPS_SPIM_MISO ),           //                               .hps_io_spim1_inst_MISO
		.hps_0_hps_io_hps_io_spim1_inst_SS0    ( HPS_SPIM_SS   ),             //                               .hps_io_spim1_inst_SS0
		//HPS UART		
		.hps_0_hps_io_hps_io_uart0_inst_RX     ( HPS_UART_RX   ),          //                               .hps_io_uart0_inst_RX
		.hps_0_hps_io_hps_io_uart0_inst_TX     ( HPS_UART_TX   ),          //                               .hps_io_uart0_inst_TX
		//HPS I2C1
		.hps_0_hps_io_hps_io_i2c0_inst_SDA     ( HPS_I2C0_SDAT  ),        //                               .hps_io_i2c0_inst_SDA
		.hps_0_hps_io_hps_io_i2c0_inst_SCL     ( HPS_I2C0_SCLK  ),        //                               .hps_io_i2c0_inst_SCL
		//HPS I2C2
		.hps_0_hps_io_hps_io_i2c1_inst_SDA     ( HPS_I2C1_SDAT  ),        //                               .hps_io_i2c1_inst_SDA
		.hps_0_hps_io_hps_io_i2c1_inst_SCL     ( HPS_I2C1_SCLK  ),        //                               .hps_io_i2c1_inst_SCL
		//GPIO 
		.hps_0_hps_io_hps_io_gpio_inst_GPIO09  ( HPS_CONV_USB_N ),  //                               .hps_io_gpio_inst_GPIO09
		.hps_0_hps_io_hps_io_gpio_inst_GPIO35  ( HPS_ENET_INT_N ),  //                               .hps_io_gpio_inst_GPIO35
		.hps_0_hps_io_hps_io_gpio_inst_GPIO40  ( HPS_LTC_GPIO   ),  //                               .hps_io_gpio_inst_GPIO40
		.hps_0_hps_io_hps_io_gpio_inst_GPIO53  ( HPS_LED   ),  //                               .hps_io_gpio_inst_GPIO53
		.hps_0_hps_io_hps_io_gpio_inst_GPIO54  ( HPS_KEY   ),  //                               .hps_io_gpio_inst_GPIO54
		.hps_0_hps_io_hps_io_gpio_inst_GPIO61  ( HPS_GSENSOR_INT ),  //                               .hps_io_gpio_inst_GPIO61
	  
	  
		//FPGA Partion
		.dipsw_pio_external_connection_export  ( SW	),  //  dipsw_pio_external_connection.export
		.button_pio_external_connection_export ( KEY	), // button_pio_external_connection.export
		.hps_0_h2f_reset_reset_n               (hps_fpga_reset_n ),                //                hps_0_h2f_reset.reset_n
		.led_pio_external_connection_export    ( LED	),    //    led_pio_external_connection.export
		
		.quad_pio_0_external_connection_export (	enc_count[0][31:0]	),
		.quad_pio_1_external_connection_export (	enc_count[1][31:0]	),
		.quad_pio_2_external_connection_export (	enc_count[2][31:0]	),
		.quad_pio_3_external_connection_export (	enc_count[3][31:0]	),
		.quad_pio_4_external_connection_export (	enc_count[4][31:0]	),
		.quad_pio_5_external_connection_export (	enc_count[5][31:0]	),
		.quad_pio_6_external_connection_export (	enc_count[6][31:0]	),
		.quad_pio_7_external_connection_export (	enc_count[7][31:0]	),
		//External encoders
		.quad_pio_8_external_connection_export (	enc_count[8][31:0]	),
		.quad_pio_9_external_connection_export (	enc_count[9][31:0]	),
		.quad_pio_10_external_connection_export (	enc_count[10][31:0]	),
		.quad_pio_11_external_connection_export (	enc_count[11][31:0]	),
								
								
		.quad_reset_pio_external_connection_export (	enc_reset[31:0]	),
		
		//.gpio_pio_0_external_connection_export    (	GPIO_0[31:24]	),
		.gpio_pio_1_external_connection_export    (	GPIO_1[15:8]	),
		.limit_pio_external_connection_export		(	GPIO_1[23:16]	),
		
		.pid_error_pio_0_external_connection_export(	pid_error[0][31:0]	),
		.pid_error_pio_1_external_connection_export(	pid_error[1][31:0]	),
		.pid_error_pio_2_external_connection_export(	pid_error[2][31:0]	),
		.pid_error_pio_3_external_connection_export(	pid_error[3][31:0]	),
		.pid_error_pio_4_external_connection_export(	pid_error[4][31:0]	),
		.pid_error_pio_5_external_connection_export(	pid_error[5][31:0]	),
		.pid_error_pio_6_external_connection_export(	pid_error[6][31:0]	),
		.pid_error_pio_7_external_connection_export(	pid_error[7][31:0]	),
		
		.pid_correction_pio_0_external_connection_export(	pid_correction[0][31:0]	),
		.pid_correction_pio_1_external_connection_export(	pid_correction[1][31:0]	),
		.pid_correction_pio_2_external_connection_export(	pid_correction[2][31:0]	),
		.pid_correction_pio_3_external_connection_export(	pid_correction[3][31:0]	),
		.pid_correction_pio_4_external_connection_export(	pid_correction[4][31:0]	),
		.pid_correction_pio_5_external_connection_export(	pid_correction[5][31:0]	),
		.pid_correction_pio_6_external_connection_export(	pid_correction[6][31:0]	),
		.pid_correction_pio_7_external_connection_export(	pid_correction[7][31:0]	),
		.pid_values_pio_external_connection_export(	pid_values[31:0]),
		
		.pwm_pio_0_external_connection_export(	pwm_values[0][31:0]	),
		.pwm_pio_1_external_connection_export(	pwm_values[1][31:0]	),
		.pwm_pio_2_external_connection_export(	pwm_values[2][31:0]	),
		.pwm_pio_3_external_connection_export(	pwm_values[3][31:0]	),
		.pwm_pio_4_external_connection_export(	pwm_values[4][31:0]	),
		.pwm_pio_5_external_connection_export(	pwm_values[5][31:0]	),
		.pwm_pio_6_external_connection_export(	pwm_values[6][31:0]	),
		.pwm_pio_7_external_connection_export(	pwm_values[7][31:0]	)
	  
	  
 );

pwm pwm0(
	.clk        (fpga_clk_50),
	.in         (pwm_values[0][7:0]),  
	.out        (GPIO_1[4])
);

pwm pwm1(
	.clk        (fpga_clk_50),
	.in         (pwm_values[1][7:0]),  
	.out        (GPIO_1[5])
);

pwm pwm2(
	.clk        (fpga_clk_50),
	.in         (pwm_values[2][7:0]),  
	.out        (GPIO_1[6])
);

pwm pwm3(
	.clk        (fpga_clk_50),
	.in         (pwm_values[3][7:0]),  
	.out        (GPIO_1[7])
);

pwm pwm4(
	.clk        (fpga_clk_50),
	.in         (pwm_values[4][7:0]),  
	.out        (GPIO_1[0])
);

pwm pwm5(
	.clk        (fpga_clk_50),
	.in         (pwm_values[5][7:0]),  
	.out        (GPIO_1[1])
);

pwm pwm6(
	.clk        (fpga_clk_50),
	.in         (pwm_values[6][7:0]),  
	.out        (GPIO_1[2])
);

pwm pwm7(
	.clk        (fpga_clk_50),
	.in         (pwm_values[7][7:0]),  
	.out        (GPIO_1[3])
);


quad_counter quad0(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_0[12]),
	.quadB		(GPIO_0[13]), 
	.count		(enc_count[0][31:0]),
	.reset		(enc_reset[0])
);

quad_counter quad1(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_0[14]),
	.quadB		(GPIO_0[15]), 
	.count		(enc_count[1][31:0]),
	.reset		(enc_reset[1])
);

quad_counter quad2(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_0[16]),
	.quadB		(GPIO_0[17]), 
	.count		(enc_count[2][31:0]),
	.reset		(enc_reset[2])
);

quad_counter quad3(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_0[18]),
	.quadB		(GPIO_0[19]), 
	.count		(enc_count[3][31:0]),
	.reset		(enc_reset[3])
);

quad_counter quad4(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_0[20]),
	.quadB		(GPIO_0[21]), 
	.count		(enc_count[4][31:0]),
	.reset		(enc_reset[4])
);

quad_counter quad5(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_0[22]),
	.quadB		(GPIO_0[23]), 
	.count		(enc_count[5][31:0]),
	.reset		(enc_reset[5])
);

quad_counter quad6(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_0[24]),
	.quadB		(GPIO_0[25]), 
	.count		(enc_count[6][31:0]),
	.reset		(enc_reset[6])
);

quad_counter quad7(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_0[26]),
	.quadB		(GPIO_0[27]), 
	.count		(enc_count[7][31:0]),
	.reset		(enc_reset[7])
);

//For external encoders
quad_counter quad8(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_1[24]),
	.quadB		(GPIO_1[25]), 
	.count		(enc_count[8][31:0]),
	.reset		(enc_reset[8])
);

quad_counter quad9(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_1[26]),
	.quadB		(GPIO_1[27]), 
	.count		(enc_count[9][31:0]),
	.reset		(enc_reset[9])
);

quad_counter quad10(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_1[28]),
	.quadB		(GPIO_1[29]), 
	.count		(enc_count[10][31:0]),
	.reset		(enc_reset[10])
);

quad_counter quad11(
	.clk  		(fpga_clk_50),
	.quadA		(GPIO_1[30]),
	.quadB		(GPIO_1[31]), 
	.count		(enc_count[11][31:0]),
	.reset		(enc_reset[11])
);


PID pid0(
	.u_out 		(pid_correction[0][31:0]),
	.e_in			(pid_error[0][31:0]),
	.clk			(fpga_clk_5khz),
	.reset		(enc_reset[0]),
	.k1			(pid_values[7:0]),
	.k2			(pid_values[15:8]),
	.k3			(pid_values[23:16])
);

PID pid1(
	.u_out 		(pid_correction[1][31:0]),
	.e_in			(pid_error[1][31:0]),
	.clk			(fpga_clk_5khz),
	.reset		(enc_reset[0]),
	.k1			(pid_values[7:0]),
	.k2			(pid_values[15:8]),
	.k3			(pid_values[23:16])
);

PID pid2(
	.u_out 		(pid_correction[2][31:0]),
	.e_in			(pid_error[2][31:0]),
	.clk			(fpga_clk_5khz),
	.reset		(enc_reset[0]),
	.k1			(pid_values[7:0]),
	.k2			(pid_values[15:8]),
	.k3			(pid_values[23:16])
);

PID pid3(
	.u_out 		(pid_correction[3][31:0]),
	.e_in			(pid_error[3][31:0]),
	.clk			(fpga_clk_5khz),
	.reset		(enc_reset[0]),
	.k1			(pid_values[7:0]),
	.k2			(pid_values[15:8]),
	.k3			(pid_values[23:16])
);

PID pid4(
	.u_out 		(pid_correction[4][31:0]),
	.e_in			(pid_error[4][31:0]),
	.clk			(fpga_clk_5khz),
	.reset		(enc_reset[0]),
	.k1			(pid_values[7:0]),
	.k2			(pid_values[15:8]),
	.k3			(pid_values[23:16])
);

PID pid5(
	.u_out 		(pid_correction[5][31:0]),
	.e_in			(pid_error[5][31:0]),
	.clk			(fpga_clk_5khz),
	.reset		(enc_reset[0]),
	.k1			(pid_values[7:0]),
	.k2			(pid_values[15:8]),
	.k3			(pid_values[23:16])
);

PID pid6(
	.u_out 		(pid_correction[6][31:0]),
	.e_in			(pid_error[6][31:0]),
	.clk			(fpga_clk_5khz),
	.reset		(enc_reset[0]),
	.k1			(pid_values[7:0]),
	.k2			(pid_values[15:8]),
	.k3			(pid_values[23:16])
);

PID pid7(
	.u_out 		(pid_correction[7][31:0]),
	.e_in			(pid_error[7][31:0]),
	.clk			(fpga_clk_5khz),
	.reset		(enc_reset[0]),
	.k1			(pid_values[7:0]),
	.k2			(pid_values[15:8]),
	.k3			(pid_values[23:16])
);

scale_clock scaled_clock_5khz(
	.clk_50Mhz (fpga_clk_50),
	.rst	  (enc_reset[0]),
	.clk_2Hz	  (fpga_clk_5khz)
);


endmodule
