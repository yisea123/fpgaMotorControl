#ifndef HEADER_FILE
#define HEADER_FILE

#include "fpga_headers/hps_0.h"
#include "fpga_headers/hwlib.h"
#include "fpga_headers/socal.h"
#include "fpga_headers/hps.h"
#include "fpga_headers/alt_gpio.h"

//Globals
volatile unsigned long *h2p_lw_led_addr;
volatile unsigned long *h2p_lw_gpio_addr;
volatile unsigned long *h2p_lw_heartbeat_addr;
volatile unsigned long *h2p_lw_pid_values_addr;
volatile unsigned long *h2p_lw_quad_reset_addr;
volatile unsigned long *h2p_lw_limit_switch_addr;
volatile unsigned long *h2p_lw_pid_e_stop;
volatile unsigned long *h2p_lw_quad_addr[8];
volatile unsigned long *h2p_lw_quad_addr_external[4];
volatile unsigned long *h2p_lw_pid_input_addr[8];
volatile unsigned long *h2p_lw_pid_output_addr[8];
volatile unsigned long *h2p_lw_pwm_values_addr[8];
volatile unsigned long *h2p_lw_adc;

//Functions
int calc_current_offset(volatile unsigned long *h2p_lw_adc);

#endif