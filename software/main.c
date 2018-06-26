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
#include <time.h>       /* time_t, time (for timestamp in second) */
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

#define RUN_SINE 0

#define SAMPLE_RATE 	1000
#define SYNC_TOLERANCE 10

#define dt (1.0/(float)SAMPLE_RATE)
#define interval_time_us ((int)(dt * 1000000))
#define update_dt 0.01

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

volatile unsigned long *h2p_lw_led_addr;//=NULL;
volatile unsigned long *h2p_lw_gpio_addr;//=NULL;
volatile unsigned long *h2p_lw_heartbeat_addr;//=NULL;
volatile unsigned long *h2p_lw_pid_values_addr;//=NULL;
volatile unsigned long *h2p_lw_quad_reset_addr;//=NULL;
volatile unsigned long *h2p_lw_limit_switch_addr;//=NULL;

volatile unsigned long *h2p_lw_quad_addr[8];//=NULL;
volatile unsigned long *h2p_lw_quad_addr_external[4];//=NULL
volatile unsigned long *h2p_lw_pid_input_addr[8];//=NULL;
volatile unsigned long *h2p_lw_pid_output_addr[8];//=NULL;
volatile unsigned long *h2p_lw_pwm_values_addr[8];//=NULL;

volatile int32_t position_setpoints[8];
int32_t position_offsets[8];

int32_t beat = 0;

FILE *file;
int sockfd, newsockfd; //global socket value such that it can be called in signal catcher to close ports

int exit_flag = 0;

uint32_t createMask(uint32_t startBit, int num_bits);
uint32_t createNativeInt(uint32_t input, int size);
void *threadFunc(void *arg);
void *heartbeat_func(void *arg);
void error(const char *msg);
void zero_motor_axis(void);
void zero_motors(char *write_buffer,int newsockfd);


int freezeMain = 0;

uint8_t P=30;
uint8_t I=0;
uint8_t D=0;
float controllerGain = 0.01;


int portnumber_global;
int socket_error = 0;
int system_state = 1;

uint8_t switch_states[8];
int32_t internal_encoders[8];

int32_t arm_encoders1=0,arm_encoders2=0,arm_encoders3=0,arm_encoders4=0;

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

uint64_t GetTimeStamp() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

void my_handler(int s){
		int n;
		char write_buffer[256];
		bzero(write_buffer,256);

		printf("Caught signal %d\n",s);
		printf("Storing motor encoder positions to encoder_values.txt\n");
		fprintf(file, "%d,%d,%d,%d,%d,%d,%d,%d", internal_encoders[0], internal_encoders[1], internal_encoders[2],\
			internal_encoders[3], internal_encoders[4], internal_encoders[5], internal_encoders[6], internal_encoders[7]);

		printf("Writing back to python side, closing sockets\n");

		/*--------------------------------
		write back to python to kill sockets
		--------------------------------*/
   		sprintf(write_buffer,"* closeports *");
    	n = write(newsockfd,write_buffer,256);
    	// if (n < 0){
    	// 	error("ERROR writing to socket upon closing ports, port");
    	// }

	    close(newsockfd);
	    close(sockfd);

		fclose(file);
        exit_flag = 1; 
}

int main(int argc, char **argv)
{
	/*
	------------------------------------------
	Generic setup below for port communication
	------------------------------------------
	*/
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
	

	pthread_create(&pth_heartbeat,NULL,heartbeat_func,NULL);
	pthread_create(&pth,NULL,threadFunc,NULL);

/*--------------------------------
setup file to store setpoint on shutdown and load on boot
--------------------------------*/
	char str[100];
	char *val;

	file = fopen("encoder_values.txt", "r+");
	if (file){
		fgets(str,100,file);
		val = strtok(str, ",");

		printf("Loaded setpoitns\n");
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

	//below resets just one counter
	/*
	unsigned current_count = *h2p_lw_quad_reset_addr;
	printf("before change: %d\n", *(uint32_t*)h2p_lw_quad_reset_addr);
	current_count = current_count | (1 << 0);
	alt_write_word(h2p_lw_quad_reset_addr, current_count);
	printf("number to write: %d, after change: %d\n", current_count, *(uint32_t*)h2p_lw_quad_reset_addr);
	usleep(1000*50);
	current_count = *h2p_lw_quad_reset_addr;
	current_count = current_count & ~(1 << 0);
	alt_write_word(h2p_lw_quad_reset_addr, current_count);
	*/
	

	//zero_motor_axis();
	//usleep(1000*15000);
	//below resets all counters, PID controllers

	/*alt_write_word(h2p_lw_quad_reset_addr, 0xFFFFFFFF);
	usleep(1000*1000);
	alt_write_word(h2p_lw_quad_reset_addr, 0);
	usleep(1000*1000);

	*/

	//set PWM values to zero
	for(j=0;j<8;j++){
		alt_write_word(h2p_lw_pwm_values_addr[j], 0);
	}
	
	//alt_write_word(h2p_lw_pwm_values_addr[7], 150);
	
	//calculate PID values
	uint32_t PID_values = 0;
	PID_values = PID_values | (P+I+D);
	//Question for this
	PID_values = PID_values | ((P+2*D) << 8);
	//PID_values = PID_values | (10 << 8);
	PID_values = PID_values | (D << 16);
	alt_write_word(h2p_lw_pid_values_addr, PID_values);
	uint32_t pid_values_read = *(uint32_t*)h2p_lw_pid_values_addr;
	printf("Sent: P: %d, I: %d, D: %d\n\n", P, I, D);
	printf("Received: P: %d, I: %d, D: %d\n\n", (uint8_t)(pid_values_read & (0x000000FF)), (uint8_t)((pid_values_read & (0x0000FF00))>>8), (uint8_t)((pid_values_read & (0x00FF0000))>>16));
	
	
	/*
	------------------------------------------
	Run controller
	------------------------------------------
	*/

	int dir_bitmask;
	long myCounter = 0;
	
	struct timeval timer_usec; 

	double tracking_error[8];

	int max_val = 0;
	int min_val = 0;
	double sine_magnitude = 1000.0;

	for(j = 0; j<8; j++){
		//position_setpoints[j] = 0;
		tracking_error[j] = 0;
	}

	//clock_t start = clock();
	//clock_t loop_start = clock();

	uint64_t start = GetTimeStamp();
	while(exit_flag == 0)
	{

		if(!freezeMain){
		
			//uint32_t mask = createMask(0,30);
			uint32_t switches = alt_read_word(h2p_lw_limit_switch_addr);
			switch_states[0] = (switches&1<<0)==0;
			switch_states[1] = (switches&1<<3)==0;
			switch_states[2] = (switches&1<<2)==0;
			switch_states[3] = (switches&1<<1)==0;
			switch_states[4] = (switches&1<<7)==0;
			switch_states[5] = (switches&1<<6)==0;
			switch_states[6] = (switches&1<<5)==0;
			switch_states[7] = (switches&1<<4)==0;
			//j = 0;

			if(RUN_SINE){
				
				uint64_t delta = (GetTimeStamp() - start) / 1000;
				int sp = sin((double)(delta*(2.0*3.14))/1000*8) * sine_magnitude;
				//int sp = 1000;
				//printf("\n\n%lf %d\n\n", 1, sp);
				for(j = 0; j<8; j++){
					position_setpoints[j] = sp;
				}
			}

			for(j = 0; j<8; j++){
				int32_t output = alt_read_word(h2p_lw_quad_addr[j]); //& mask;
				internal_encoders[j] = output + position_offsets[j];

				if(j==7 && output > max_val)
					max_val = output;

				if(j==7 && output < min_val)
					min_val = output;
				
				if(j==7){
					arm_encoders1 = alt_read_word(h2p_lw_quad_addr_external[0]);
					arm_encoders2 = alt_read_word(h2p_lw_quad_addr_external[1]);
					arm_encoders3 = alt_read_word(h2p_lw_quad_addr_external[2]);
					arm_encoders4 = alt_read_word(h2p_lw_quad_addr_external[3]);
				}

				//int32_t nativeInt = createNativeInt(output, 30);

				int32_t error = internal_encoders[j] - position_setpoints[j];
				tracking_error[j] = tracking_error[j]*0.99 + error*.01;

				alt_write_word(h2p_lw_pid_input_addr[j], error);
				int32_t check_error = (int32_t)(*h2p_lw_quad_addr[j]);// - position_setpoints[j];
				
				//int32_t pid_output = *(int32_t*)(h2p_lw_pid_output_addr[j]);
				usleep(10);
				int32_t pid_output = (int32_t)(alt_read_word(h2p_lw_pid_output_addr[j])) * controllerGain;
				int32_t positive_pid_output = (pid_output>=0);
				int32_t pid_output_cutoff = fabs(pid_output)*(fabs(pid_output) <= 255) + 255*(fabs(pid_output) > 255);	
				
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
				
				char string_write[255];
				sprintf(string_write, "Axis %d, Position Setpoint %d, Current count %d, Error %d, Current PID output unsigned %d\n", j, position_setpoints[j], internal_encoders[j], error, pid_output);
				//if(j==2)
				//	fprintf(fp, string_write);
				if(myCounter%100 == 0 && j == 7){
					//printf("%d", beat);
					printf("Axis: %d; AVG tracking error: %lf Position Setpoint: %d; Error: %d; Current PID output, unsigned: %d; Cutoff output: %d\n", j, tracking_error[j], position_setpoints[j], error, pid_output, pid_output_cutoff);
					printf("Heartbeat: %d; Error: %d; Error read: %d; PID out: %d\n", myCounter, error, check_error, pid_output);
					int dval = max_val - min_val;
					printf("value range: %d\n", dval);

					if (dval > sine_magnitude * 1.5){
						max_val = 0;
						min_val = 0;
					}
					// printf("External encoder counts: %d, %d, %d, %d\n", arm_encoders1, arm_encoders2, arm_encoders3, arm_encoders4);
					// printf("%d,%d,%d,%d,%d,%d,%d,%d\n", switch_states[0],switch_states[1],switch_states[2],switch_states[3],switch_states[4],switch_states[5],switch_states[6],switch_states[7]);
					// printf("%d,%d,%d,%d,%d,%d,%d,%d\n", position_setpoints[0],position_setpoints[1],position_setpoints[2],position_setpoints[3],position_setpoints[4],position_setpoints[5],position_setpoints[6],position_setpoints[7]);
					// if(j==7)
					// 	printf("\n\n");
				}
			}
			//if(myCounter%100 == 0)
			//	printf("\n\n");
			/*LEDR_AllOn();
			usleep(1000*50);
			LEDR_AllOff();
			usleep(1000*50);
			*/
			myCounter++;
		}
		usleep(1000);

	}
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );

	}

	printf("\n\n Exiting safely \n\n");
	close( fd );
	//fclose(fp);
	return 0;
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

void *threadFunc(void *arg)
{


/*--------------------------------
setup socket communication
--------------------------------*/
	char * pch;

	int portno;
    socklen_t clilen;
    char buffer[256];
    char old_buffer[256];
    char write_buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n, j, k;
    int state=1;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
	//fcntl(sockfd, F_SETFL, O_NONBLOCK); //sets the socket to nonblocking, will poll and return -1 and errno will be set to EAGAIN or EWOULDBLOCK


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

    // while (newsockfd < 0){
	   //  newsockfd = accept(sockfd, 
	   //              (struct sockaddr *) &cli_addr, 
	   //              &clilen);
	   //  sleep(1);
    // }

    newsockfd = accept(sockfd, 
                (struct sockaddr *) &cli_addr, 
                &clilen);

    if (newsockfd < 0){ 
        error("ERROR on accept");
        return;
    }


    bzero(buffer,256);
    bzero(old_buffer,256);
    bzero(write_buffer,256);
	char *str;

	str=(char*)arg;


	while(system_state == 1 && socket_error == 0 ){

		nanosleep((const struct timespec[]){{0, 2500000L}}, NULL);

		/*--------------------------------
		read from socket
		--------------------------------*/
		
		n = read(newsockfd,buffer,255); //blocking function, unless set with fcntl as above
		printf("Reading DATA NOW************************************\n");
		// if (n>0){
		// 	memcpy(old_buffer, buffer, 256);
		// }
		// memcpy(buffer, old_buffer, 256);

		
		// printf("data %d\n", n);
		// printf("string %s\n", buffer);
		// printf("old_string %s\n", old_buffer);


   		if (n < 0){
   			error("ERROR reading from socket");
   			break;
   		}


    	/*--------------------------------
		parse received message from socket
		--------------------------------*/
    	for(k = 0; k<9; k++){
    		if(k==0){
				pch = strtok (buffer,"bd ");
				if(pch == NULL){ //found end of string early
					error("ERROR parsing message");
					break;
				}
				state = atoi(pch);
				//position_setpoints[k] = atoi(pch);
			}
			else{
				pch = strtok (NULL,"bd ");
				if(pch == NULL){ //found end of string early
					error("ERROR parsing message");
					break;
				}
				if(state){
					position_setpoints[k-1] = atoi(pch);
				}
			}
		}
		//When state == 0, we are zeroing
		if(state==0){
			zero_motors(write_buffer,newsockfd);
		}

		/*--------------------------------
		write switch, external encoder, internal encoder state to socket
		--------------------------------*/
   		sprintf(write_buffer,"nn %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d qq", internal_encoders[0], internal_encoders[1], internal_encoders[2],\
   			internal_encoders[3], internal_encoders[4], internal_encoders[5], internal_encoders[6], internal_encoders[7], switch_states[0],\
   			switch_states[1], switch_states[2], switch_states[3], switch_states[4], switch_states[5], switch_states[6], switch_states[7],\
   			arm_encoders1, arm_encoders2, arm_encoders3, arm_encoders4);

    	n = write(newsockfd,write_buffer,256);
    	printf("PRINTING DATA NOW************************************");
    	if (n < 0){
    		error("ERROR writing to socket");
    		break;
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
			//printf("1");
			alt_write_word(h2p_lw_heartbeat_addr, 0);
			counter=1;
		}

		else{
			//printf("0");
			alt_write_word(h2p_lw_heartbeat_addr, 0xFFFFFFFF);
			counter=0;
		}
		///sleep(1);

		//printf("This is our first beat: %d", *h2p_lw_heartbeat_addr);

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
	}

	//Need to pass write_buffer and newsockfd
	done=1;
	sprintf(write_buffer,"nn %d %d qq",done,rate);
	//n = write(newsockfd,write_buffer,256);
	//if (n < 0){
	//	error("ERROR writing to socket");
	//	//break;
	//}

	//want to move back from limit switches
	// for(i=1; i<10; i++){
	// 	for(k=0; k<8; k++){
	// 		position_setpoints[k] = position_setpoints[k] - 100;
	// 	}
	// 	usleep(10000);
	// }


	/*printf("**Done zeroing**\n");

	//set PWM values to zero
	for(j=0;j<8;j++){
		alt_write_word(h2p_lw_pwm_values_addr[j], 255);
	}

	//below resets all counters, PID controllers
	alt_write_word(h2p_lw_quad_reset_addr, 0xFFFFFFFF);
	usleep(1000*1000);
	alt_write_word(h2p_lw_quad_reset_addr, 0);
	usleep(1000*1000);
	//freezeMain = 0;*/
	
}
