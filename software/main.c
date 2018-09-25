#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "soc_cv_av/socal/socal.h"
#include "soc_cv_av/socal/hps.h"
#include "soc_cv_av/socal/alt_gpio.h"
#include "hps_0.h"
#include "led.h"
#include <stdbool.h>
#include <math.h>
#include <sys/timeb.h>  /* ftime, timeb (for timestamp in millisecond) */
#include <sys/time.h>   /* gettimeofday, timeval (for timestamp in microsecond) */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#define SAMPLE_RATE 1000
#define SYNC_TOLERANCE 10
#define dt (1.0/(float)SAMPLE_RATE)
#define interval_time_us ((int)(dt * 1000000))
#define update_dt 0.01
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define MAX_TRAVEL_RANGE 500000
#define MAX_CURRENT 1.25 //amps

volatile unsigned long *h2p_lw_led_addr;//=NULL;
volatile unsigned long *h2p_lw_gpio_addr;//=NULL;
volatile unsigned long *h2p_lw_heartbeat_addr;//=NULL;
volatile unsigned long *h2p_lw_pid_values_addr;//=NULL;
volatile unsigned long *h2p_lw_quad_reset_addr;//=NULL;
volatile unsigned long *h2p_lw_limit_switch_addr;//=NULL;
volatile unsigned long *h2p_lw_pid_e_stop;
volatile unsigned long *h2p_lw_quad_addr[8];//=NULL;
volatile unsigned long *h2p_lw_quad_addr_external[4];//=NULL
volatile unsigned long *h2p_lw_pid_input_addr[8];//=NULL;
volatile unsigned long *h2p_lw_pid_output_addr[8];//=NULL;
volatile unsigned long *h2p_lw_pwm_values_addr[8];//=NULL;
volatile unsigned long *h2p_lw_adc;

volatile int32_t position_setpoints[8];

FILE *file;

int E_STATE = 0;
int ERR_RESET = 1;
int CONNECTED = 0;  //global flag to indicate if a connection has been made
int32_t beat = 0;
int32_t position_offsets[8];
uint8_t switch_states[8];
int32_t internal_encoders[8];
int32_t arm_encoders1=0,arm_encoders2=0,arm_encoders3=0,arm_encoders4=0;
int exit_flag = 0;

int CURRENT_FLAG = 0;
int TRAVEL_FLAG = 0;
int ETSOP_FLAG = 0;


uint8_t P=5;
uint8_t I=0;
uint8_t D=0;
float controllerGain = 0.01;
float avg_current = 0;

int portnumber_global;
int socket_error = 0;
int system_state = 1;
int sockfd, newsockfd; //global socket value such that it can be called in signal catcher to close ports

uint32_t createMask(uint32_t startBit, int num_bits);
uint32_t createNativeInt(uint32_t input, int size);
uint64_t GetTimeStamp();
int calc_current_offset(volatile unsigned long *h2p_lw_adc);
void *threadFunc(void *arg);
void *heartbeat_func(void *arg);
void error(const char *msg);
void zero_motor_axis(void);
void zero_motors(char *write_buffer,int newsockfd);

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

struct axis_motor global_motor_1 = {.accGoal = 5000, .velGoal = 0, .posGoal = 0, .accCurrent = 100, .velCurrent = 0, .posCurrent = 0, .posGoalCurrent = 0, .dutyCyle = 0, .setpointUpdated = 0, .startupFlag = 1};
struct axis_motor global_motor_2 = {.accGoal = 5000, .velGoal = 0, .posGoal = 0, .accCurrent = 100, .velCurrent = 0, .posCurrent = 0, .posGoalCurrent = 0, .dutyCyle = 0, .setpointUpdated = 0, .startupFlag = 1};
struct axis_motor global_motor_3 = {.accGoal = 5000, .velGoal = 0, .posGoal = 0, .accCurrent = 100, .velCurrent = 0, .posCurrent = 0, .posGoalCurrent = 0, .dutyCyle = 0, .setpointUpdated = 0, .startupFlag = 1};
struct axis_motor global_motor_4 = {.accGoal = 5000, .velGoal = 0, .posGoal = 0, .accCurrent = 100, .velCurrent = 0, .posCurrent = 0, .posGoalCurrent = 0, .dutyCyle = 0, .setpointUpdated = 0, .startupFlag = 1};

void my_handler(int s){
		int n;
		char write_buffer[256];
		bzero(write_buffer,256);

		printf("Caught signal %d\n",s);
		//printf("Storing motor encoder positions to encoder_values.txt\n");
		//fprintf(file, "%d,%d,%d,%d,%d,%d,%d,%d", internal_encoders[0], internal_encoders[1], internal_encoders[2],\
			internal_encoders[3], internal_encoders[4], internal_encoders[5], internal_encoders[6], internal_encoders[7]);

		/*--------------------------------
		write back to python to kill sockets
		--------------------------------*/
		if(CONNECTED==1){
			printf("Writing back to python side, closing sockets\n");
	   		sprintf(write_buffer,"* closeports *");
	    	n = write(newsockfd,write_buffer,256);
	    	// if (n < 0){
	    	// 	error("ERROR writing to socket upon closing ports, port");
	    	// }
		    close(newsockfd);
		    close(sockfd);
		}

		//fclose(file);
        exit_flag = 1; 
}

int main(int argc, char **argv)
{
	/*------------------------------------------
	Generic setup below for port communication
	-----------------------------------------*/
    if (argc < 2) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }	
   	portnumber_global = atoi(argv[1]);
	pthread_t pth, pth_heartbeat;	// this is our thread identifier

	/*--------------------------------
	ctrl-c catcher
	--------------------------------*/
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
   	sigemptyset(&sigIntHandler.sa_mask);
  	sigIntHandler.sa_flags = 0;
  	sigaction(SIGINT, &sigIntHandler, NULL);
	/*

	------------------------------------------
	Setup FPGA communication
	------------------------------------------
	*///sig
	void *virtual_base;
	int fd;
	int i;
	int j;
	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return(1);
	}
	h2p_lw_led_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_gpio_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_heartbeat_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HEARTBEAT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_reset_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_RESET_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_values_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_VALUES_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_limit_switch_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LIMIT_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_e_stop = virtual_base + ( ( unsigned long )(ALT_LWFPGASLVS_OFST + E_STOP_BASE) & ( unsigned long)( HW_REGS_MASK ) );

	h2p_lw_pwm_values_addr[0] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[1] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[2] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[3] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[4] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[5] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[6] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pwm_values_addr[7] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PWM_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	//Motor bank encoder counts
	h2p_lw_quad_addr[0]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[1]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[2]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[3]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[4]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[5]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[6]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[7]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	//External encoders for robot arm
	h2p_lw_quad_addr_external[0] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_8_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr_external[1] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_9_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr_external[2] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_10_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr_external[3] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_11_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	
	h2p_lw_pid_input_addr[0] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[1] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[2] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[3] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[4] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[5] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[6] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_input_addr[7] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_ERROR_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	
	h2p_lw_pid_output_addr[0] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[1] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[2] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[3] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[4] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[5] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[6] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_pid_output_addr[7] = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PID_CORRECTION_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	h2p_lw_adc = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	pthread_create(&pth_heartbeat,NULL,heartbeat_func,NULL);
	pthread_create(&pth,NULL,threadFunc,NULL);

	/*--------------------------------
	setup file to store setpoint on shutdown and load on boot
	--------------------------------*/
	char str[100];
	char *val;

	/*
	file = fopen("encoder_values.txt", "r+");
	if (file){
		fgets(str,100,file);
		val = strtok(str, ",");
		printf("Loaded setpoints\n");
		for(i=0;i<8;i++){
			position_setpoints[i] = atoi(val);
			position_offsets[i] = atoi(val);
			val = strtok(NULL, ",");
			printf("setpoints %d\n", position_setpoints[i]);
		}
		fclose(file);
		file = fopen("encoder_values.txt", "w+");
	}
	else {
		for(i=0;i<8;i++){
			position_setpoints[i] = 0;
			position_offsets[i] = 0;
		}
		file = fopen("encoder_values.txt", "w+");
	}
	*/

	for(i=0;i<8;i++){
			position_setpoints[i] = 0;
			position_offsets[i] = 0;
		}
	/*--------------------------------
	initial pwm and pid values -> check how pid values are set
	--------------------------------*/

	//set PWM values to zero
	for(j=0;j<8;j++){
		alt_write_word(h2p_lw_pwm_values_addr[j], 0);
	}
		
	//calculate PID values
	uint32_t PID_values = 0;
	PID_values = PID_values | (P+I+D);
	//Question for this
	PID_values = PID_values | ((P+2*D) << 8);
	//PID_values = PID_values | (10 << 8);
	PID_values = PID_values | (D << 16);
	alt_write_word(h2p_lw_pid_values_addr, PID_values);
	uint32_t pid_values_read = *(uint32_t*)h2p_lw_pid_values_addr;
	printf("\nSent: P: %d, I: %d, D: %d\n", P, I, D);
	printf("Received: P: %d, I: %d, D: %d\n\n", (uint8_t)(pid_values_read & (0x000000FF)), (uint8_t)((pid_values_read & (0x0000FF00))>>8), (uint8_t)((pid_values_read & (0x00FF0000))>>16));
	

	/*--------------------------------
	calibrate current sense
	--------------------------------*/
	int current_offset;
	printf("Calculating current offset please wait...\n");
	current_offset = calc_current_offset(h2p_lw_adc);
	printf("Current offset is: %d\n", current_offset);

	/*
	------------------------------------------
	Run controller
	------------------------------------------
	*/
	int dir_bitmask;
	long myCounter = 0;
	int32_t e_stop=0;
	struct timeval timer_usec;
	int max_val = 0;
	int min_val = 0;
	double tracking_error[8];

	int adc_data;
	float current;

	for(j = 0; j<8; j++){
		tracking_error[j] = 0;
	}

	while(exit_flag == 0)
	{
		//On motor bank limit switches
		uint32_t switches = alt_read_word(h2p_lw_limit_switch_addr);
		switch_states[0] = (switches&1<<0)==0;
		switch_states[1] = (switches&1<<3)==0;
		switch_states[2] = (switches&1<<2)==0;
		switch_states[3] = (switches&1<<1)==0;
		switch_states[4] = (switches&1<<7)==0;
		switch_states[5] = (switches&1<<6)==0;
		switch_states[6] = (switches&1<<5)==0;
		switch_states[7] = (switches&1<<4)==0;

		//ADC read
		*(h2p_lw_adc) = 0; //write starts adc read
		adc_data = *(h2p_lw_adc); //read
		current = (adc_data - current_offset) * 0.001;
		current = current * (current > 0);
		avg_current = 0.1 * current + 0.9 * avg_current;
		//avg_current = avg_current * (avg_current > 0);

		//Read encoder positions and add offset from file
		for(j = 0; j<8; j++){
			int32_t output = alt_read_word(h2p_lw_quad_addr[j]);
			internal_encoders[j] = output + position_offsets[j];

			if(j==7 && output > max_val)
				max_val = output;

			if(j==7 && output < min_val)
				min_val = output;
			
			//read external joint encoders
			if(j==7){
				arm_encoders1 = alt_read_word(h2p_lw_quad_addr_external[0]);
				arm_encoders2 = alt_read_word(h2p_lw_quad_addr_external[1]);
				arm_encoders3 = alt_read_word(h2p_lw_quad_addr_external[2]);
				arm_encoders4 = alt_read_word(h2p_lw_quad_addr_external[3]);
			}


			e_stop = alt_read_word(h2p_lw_pid_e_stop);
			//printf("e_stop value %d", e_stop);
			//E stop state checking
			if (abs(internal_encoders[j]) > MAX_TRAVEL_RANGE || abs(position_setpoints[j]) > MAX_TRAVEL_RANGE || ERR_RESET || e_stop || avg_current > MAX_CURRENT){
				E_STATE = 1;
				ERR_RESET = 1;
				//printf("e_stop value: %d\n", e_stop);
				//Reset pwm and pid loops, [0:7] and [20:27]
				int32_t reset_mask = 255 | 255<<20;
				alt_write_word(h2p_lw_quad_reset_addr, reset_mask);

				if(avg_current > MAX_CURRENT){
					CURRENT_FLAG = 1;
				}
				if(abs(internal_encoders[j]) > MAX_TRAVEL_RANGE){
					TRAVEL_FLAG = 1;
				}
				if(e_stop){
					ETSOP_FLAG = 1;
				}
			}
			//normal operation
			else{
				alt_write_word(h2p_lw_quad_reset_addr, 0);
				int32_t error = internal_encoders[j] - position_setpoints[j];
				tracking_error[j] = tracking_error[j]*0.99 + error*.01; //only used in printout

				alt_write_word(h2p_lw_pid_input_addr[j], error);
				int32_t check_error = (int32_t)(*h2p_lw_quad_addr[j]);// - position_setpoints[j];
				
				usleep(10);
				int32_t pid_output = (int32_t)(alt_read_word(h2p_lw_pid_output_addr[j])) * controllerGain;
				int32_t positive_pid_output = (pid_output>=0);
				int32_t pid_output_cutoff = fabs(pid_output)*(fabs(pid_output) <= 255) + 255*(fabs(pid_output) > 255);	
				
				//For direction
				if(j < 4 ){
					alt_write_word(h2p_lw_pwm_values_addr[j], (pid_output_cutoff));
					
					dir_bitmask = alt_read_word(h2p_lw_gpio_addr);
					if(positive_pid_output)
						dir_bitmask |= (1<<(j+4));
					else
						dir_bitmask &= ~(1<<(j+4));
					//dir_bitmask = dir_bitmask
					//alt_write_word(h2p_lw_gpio_addr, (positive_pid_output * (0b00010000<<j)));
					//alt_write_word(h2p_lw_gpio_addr, (positive_pid_output * (1<<(j+4))));
					alt_write_word(h2p_lw_gpio_addr, dir_bitmask);
				}
				else{
					if (j < 8){
					alt_write_word(h2p_lw_pwm_values_addr[j], (pid_output_cutoff));
					dir_bitmask = alt_read_word(h2p_lw_gpio_addr);
					if(positive_pid_output == 0)
						dir_bitmask |= (1<<(j-4));
					else
						dir_bitmask &= ~(1<<(j-4));
					alt_write_word(h2p_lw_gpio_addr, dir_bitmask);
					}
				}
				

				if(myCounter%100 == 0 && j == 7){
					printf("Raw value read from the ADC is : %d\n", adc_data);
					printf("Average current value is: %f\n", avg_current);

					//External encoder stuff -> this is currently the linear encoder on stage
					printf("Linear encoder counts is: %d\n", arm_encoders1);
					float inches = M_PI*1.141*arm_encoders1*0.15/360*1.029; //1.029 is a calibration constant
					printf("Linear encoder value in inches is: %f\n", inches);

					// int dval = max_val - min_val;
					// printf("Axis: %d; AVG tracking error: %lf Position Setpoint: %d; Error: %d; Current PID output, unsigned: %d; Cutoff output: %d\n", j, tracking_error[j], position_setpoints[j], error, pid_output, pid_output_cutoff);
					// printf("Heartbeat: %d; Error: %d; Error read: %d; PID out: %d\n", myCounter, error, check_error, pid_output);
					// printf("value range: %d\n", dval);

					printf("Motor encoder counts: %d,%d,%d,%d,%d,%d,%d,%d\n", internal_encoders[0], internal_encoders[1], internal_encoders[2], internal_encoders[3], internal_encoders[4], internal_encoders[5], internal_encoders[6], internal_encoders[7]);
					
					// printf("External encoder counts: %d, %d, %d, %d\n", arm_encoders1, arm_encoders2, arm_encoders3, arm_encoders4);
					// printf("%d,%d,%d,%d,%d,%d,%d,%d\n", switch_states[0],switch_states[1],switch_states[2],switch_states[3],switch_states[4],switch_states[5],switch_states[6],switch_states[7]);
					// printf("%d,%d,%d,%d,%d,%d,%d,%d\n", position_setpoints[0],position_setpoints[1],position_setpoints[2],position_setpoints[3],position_setpoints[4],position_setpoints[5],position_setpoints[6],position_setpoints[7]);

					printf("\n\n");
				}
			}
		}

		myCounter++;
		usleep(1000);

	}
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	printf("\n\n Exiting safely \n\n");
	close( fd );
	return 0;
}


void *threadFunc(void *arg)
{
/*--------------------------------
setup socket communication
--------------------------------*/
	char * pch;
	char deadsok[20], arm_state[20], zero_state[20];
	strcpy(deadsok, "stop");
	strcpy(arm_state, "arm");
	strcpy(zero_state, "zero");
	int portno;
    socklen_t clilen;
    char buffer[256];
    char old_buffer[256];
    char write_buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n, j, k;
    int state=1;
    int avg_current_ma = 0;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0){ 
        error("ERROR opening socket");
        return;
    }

    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = portnumber_global;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);

    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        error("ERROR on binding");
        return;
    }

    listen(sockfd,5);
    clilen = sizeof(cli_addr);

    newsockfd = accept(sockfd, 
                (struct sockaddr *) &cli_addr, 
                &clilen);
    CONNECTED = 1; //change CONNECTED to 1 if connection is made, 

    if (newsockfd < 0){ 
        error("ERROR on accept");
        return;
    }


    bzero(buffer,256);
    bzero(old_buffer,256);
    bzero(write_buffer,256);
	char *str;

	str=(char*)arg;

	//initialize python side with position
	sprintf(write_buffer,"nn %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d qq", internal_encoders[0], internal_encoders[1], internal_encoders[2],\
		internal_encoders[3], internal_encoders[4], internal_encoders[5], internal_encoders[6], internal_encoders[7], switch_states[0],\
		switch_states[1], switch_states[2], switch_states[3], switch_states[4], switch_states[5], switch_states[6], switch_states[7],\
		arm_encoders1, arm_encoders2, arm_encoders3, arm_encoders4, avg_current_ma);
	n = write(newsockfd,write_buffer,256);

	while(system_state == 1 && socket_error == 0 ){

		nanosleep((const struct timespec[]){{0, 2500000L}}, NULL);

		avg_current_ma = (int)(avg_current*1000);
		/*--------------------------------
		write switch, external encoder, internal encoder state to socket
		--------------------------------*/
		if (E_STATE==1){
			printf("ERROR_STATE...\n");
			sprintf(write_buffer,"nn err qq");
			if (CURRENT_FLAG){
				printf("Current limit hit\n");
			}
			if (TRAVEL_FLAG){
				printf("Travel limit hit\n");
			}
			if (ETSOP_FLAG){
				printf("ESTOP pressed\n");
			}

		}
		else{
	   		sprintf(write_buffer,"nn %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d qq", internal_encoders[0], internal_encoders[1], internal_encoders[2],\
	   			internal_encoders[3], internal_encoders[4], internal_encoders[5], internal_encoders[6], internal_encoders[7], switch_states[0],\
	   			switch_states[1], switch_states[2], switch_states[3], switch_states[4], switch_states[5], switch_states[6], switch_states[7],\
	   			arm_encoders1, arm_encoders2, arm_encoders3, arm_encoders4, avg_current_ma);
	   	}

    	n = write(newsockfd,write_buffer,256);
    	if (n < 0){
    		error("ERROR writing to socket");
    		break;
    	}

		/*--------------------------------
		read from socket
		--------------------------------*/
		n = read(newsockfd,buffer,255); //blocking function, unless set with fcntl or something
   		if (n < 0){
   			error("ERROR reading from socket");
   			break;
   		}

    	/*--------------------------------
		parse received message from socket
		--------------------------------*/
   		pch = strtok (buffer,"bd ");
    	for(k = 0; k<8; k++){
			if(strncmp(pch,deadsok,4)==0){
				close(newsockfd);
				E_STATE = 1;
				ERR_RESET = 1;
			    listen(sockfd,5);
			    clilen = sizeof(cli_addr);

			    newsockfd = accept(sockfd, 
			                (struct sockaddr *) &cli_addr, 
			                &clilen);

				sprintf(write_buffer,"nn %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d qq", internal_encoders[0], internal_encoders[1], internal_encoders[2],\
					internal_encoders[3], internal_encoders[4], internal_encoders[5], internal_encoders[6], internal_encoders[7], switch_states[0],\
					switch_states[1], switch_states[2], switch_states[3], switch_states[4], switch_states[5], switch_states[6], switch_states[7],\
					arm_encoders1, arm_encoders2, arm_encoders3, arm_encoders4, avg_current_ma);
				n = write(newsockfd,write_buffer,256);

			    CONNECTED = 1; //change CONNECTED to 1 if connection is made, 

			    if (newsockfd < 0){ 
			        error("ERROR on accept");
			        return;
			    }
				break;
			}
			else if(strncmp(pch,arm_state,3)==0){
				printf("System armed");
				//Reset flags
				E_STATE = 0;
				ERR_RESET = 0;
				CURRENT_FLAG = 0;
				TRAVEL_FLAG = 0;
				ETSOP_FLAG = 0;
				break;
			}
			else if(strncmp(pch,zero_state,4)==0){
				zero_motors(write_buffer,newsockfd);
				break;
			}
			else if(pch == NULL){
				error("ERROR parsing message");
				break;
			}

			else{
				position_setpoints[k] = atoi(pch);
			}
			pch = strtok (NULL,"bd ");
		}
	}
	printf("Exited thread loop\n");
    close(newsockfd);
    close(sockfd);
}


//Pthread function
void *heartbeat_func(void *arg){
	int counter = 0;
	while(1){
		if(counter==0){
			alt_write_word(h2p_lw_heartbeat_addr, 0);
			counter=1;
		}
		else{
			alt_write_word(h2p_lw_heartbeat_addr, 0xFFFFFFFF);
			counter=0;
		}
		usleep(0.1*10000);//1.1 seconds
	}
}



void error(const char *msg)
{
    perror(msg);
    socket_error = 1;
}

void zero_motor_axis(void){
	int j, dir_bitmask, positive_pid_output=1;
	for(j = 0; j<8; j++){
		if(j < 4 ){
			alt_write_word(h2p_lw_pwm_values_addr[j], (25));
			
			dir_bitmask = alt_read_word(h2p_lw_gpio_addr);
			if(positive_pid_output)
				dir_bitmask |= (1<<(j+4));
			else
				dir_bitmask &= ~(1<<(j+4));
			//dir_bitmask = dir_bitmask
			//alt_write_word(h2p_lw_gpio_addr, (positive_pid_output * (0b00010000<<j)));
			//alt_write_word(h2p_lw_gpio_addr, (positive_pid_output * (1<<(j+4))));
			alt_write_word(h2p_lw_gpio_addr, dir_bitmask);
		}
		else{
			if (j < 8){

			alt_write_word(h2p_lw_pwm_values_addr[j], (25));
			dir_bitmask = alt_read_word(h2p_lw_gpio_addr);
			if(positive_pid_output == 0)
				dir_bitmask |= (1<<(j-4));
			else
				dir_bitmask &= ~(1<<(j-4));
			alt_write_word(h2p_lw_gpio_addr, dir_bitmask);
			}
		}
	}
}

void zero_motors(char *write_buffer,int newsockfd){
	int i, k, j, n;
	int rate=0, direction=1,switch_count=0, done=0;
	int zero_rates[6]={5,5,5,1,1,1};

	while(rate<5){
		//Moves out until all switches read 1
		while(direction==1){
			switch_count = 0;
			for(k=0; k<8; k++){
				switch_count = switch_count + switch_states[k];
				position_setpoints[k] = position_setpoints[k] + (!switch_states[k])*zero_rates[rate];
			}
			if(switch_count==8){ //if true all switches read 1
				direction = 0;
				rate=rate+1;
			}
			/*--------------------------------
			write switch state to socket
			--------------------------------*/
			/*
			sprintf(write_buffer,"nn %d %d %d %d %d %d %d %d qq", switch_states[0], switch_states[1], switch_states[2], switch_states[3], switch_states[4], switch_states[5], switch_states[6], switch_states[7]);
			//n = write(newsockfd,write_buffer,256);
			if (n < 0){
				error("ERROR writing to socket");
				break;
			}
			*/
			usleep(10000);
		}

		//Moves in until all switches read 0
		while(direction==0){
			switch_count = 0;
			for(k=0; k<8; k++){
				switch_count = switch_count + switch_states[k];
				position_setpoints[k] = position_setpoints[k] - switch_states[k]*zero_rates[rate];
			}
			if(!switch_count){ //want switch count to be 0, at this point all switches are 0
				rate=rate+1;
				direction = 1;
			}
			/*--------------------------------
			write switch state to socket
			--------------------------------*/
			/*
			sprintf(write_buffer,"nn %d %d %d %d %d %d %d %d qq", switch_states[0], switch_states[1], switch_states[2], switch_states[3], switch_states[4], switch_states[5], switch_states[6], switch_states[7]);
			//n = write(newsockfd,write_buffer,256);
			if (n < 0){
				error("ERROR writing to socket");
				break;
			}
			*/
			usleep(10000);
		}
		printf("Zeroing here**********************************");
	}

	//Need to pass write_buffer and newsockfd
	//done=1;
	//sprintf(write_buffer,"nn %d %d qq",done,rate);
	//n = write(newsockfd,write_buffer,256);
	if (n < 0){
		error("ERROR writing to socket");
		//break;
	}
}

uint32_t createMask(uint32_t startBit, int num_bits)
{
   uint32_t  mask;
	mask = ((1 << num_bits) - 1) << startBit;
   return mask;
}

uint32_t createNativeInt(uint32_t input, int size)
{
	int32_t nativeInt;
	const int negative = ((input & (1 << (size - 1))) != 0);
	if (negative)
		  nativeInt = input | ~((1 << size) - 1);
	else 
		  nativeInt = input;	
	return nativeInt;
}


uint64_t GetTimeStamp() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

int calc_current_offset(volatile unsigned long *h2p_lw_adc){
	int counter = 300, offset;
	double adc_sum = 0;

	//sample at 100hz for 3 seconds
	while(counter > 0){
		*(h2p_lw_adc) = 0; //write starts adc read
		adc_sum = *(h2p_lw_adc) + adc_sum; //read
		// printf("Counter: %d\n", counter);
		counter = counter - 1;
		usleep(10000);
	}
	
	offset = (int)(adc_sum/400);
	return offset;
}