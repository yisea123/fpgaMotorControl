#ifndef HEADER_FILE
#define HEADER_FILE

#include "fpga_headers/hps_0.h"
#include "fpga_headers/hwlib.h"
#include "fpga_headers/socal.h"
#include "fpga_headers/hps.h"
#include "fpga_headers/alt_gpio.h"
#include <pthread.h>

//MACROS
#define SAMPLE_RATE 1000
#define SYNC_TOLERANCE 10
#define dt (1.0/(float)SAMPLE_RATE)
#define interval_time_us ((int)(dt * 1000000))
#define update_dt 0.01
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define MAX_TRAVEL_RANGE 10000000
#define MAX_CURRENT 2.5 //amps

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
volatile int32_t position_setpoints[8];

int E_STATE;
int ERR_RESET;
int CONNECTED;//global flag to indicate if a connection has been made
int32_t beat;
uint8_t switch_states[8];
int32_t internal_encoders[8];
int32_t arm_encoders1,arm_encoders2,arm_encoders3,arm_encoders4;
int exit_flag;

int portnumber_global;
int socket_error;
int system_state;
int sockfd, newsockfd; //global socket value such that it can be called in signal catcher to close ports

int CURRENT_FLAG;
int TRAVEL_FLAG;
int ETSOP_FLAG;

uint8_t P;
uint8_t I;
uint8_t D;
float controllerGain;
float avg_current;

pthread_t pth, pth_heartbeat;	// this is our thread identifier

//Struct -> not used?
struct axis_motor{
		double accGoal;
		double velGoal;
		double posGoal;
		double accCurrent;
		double velCurrent;
		double posCurrent;
		double posGoalCurrent;
		double dutyCyle;
		int setpointUpdated;
		int startupFlag;
};

//Functions
int calc_current_offset(volatile unsigned long *h2p_lw_adc);

void *heartbeat_func(void *arg);

void *threadFunc(void *arg);

void zero_motors(char *write_buffer,int newsockfd);

void error(const char *msg);

void my_handler(int s);

void mask_sig(void);

uint32_t createMask(uint32_t startBit, int num_bits);

uint32_t createNativeInt(uint32_t input, int size);

uint64_t GetTimeStamp();

#endif