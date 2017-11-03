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
volatile unsigned long *h2p_lw_pid_values_addr;//=NULL;
volatile unsigned long *h2p_lw_quad_reset_addr;//=NULL;
volatile unsigned long *h2p_lw_limit_switch_addr;//=NULL;

volatile unsigned long *h2p_lw_quad_addr[8];//=NULL;
volatile unsigned long *h2p_lw_pid_input_addr[8];//=NULL;
volatile unsigned long *h2p_lw_pid_output_addr[8];//=NULL;
volatile unsigned long *h2p_lw_pwm_values_addr[8];//=NULL;

volatile unsigned long position_setpoints[8];

uint32_t createMask(uint32_t startBit, int num_bits);
uint32_t createNativeInt(uint32_t input, int size);
void *threadFunc(void *arg);
void error(const char *msg);
void zero_motor_axis(void);

uint8_t P=1;
uint8_t I=0;
uint8_t D=0;

int portnumber_global;
long setpoint_global;
long setpoint_global2;
double velocity_global;
double velocity_global2;
int setpoint_update = 0;
int setpoint_update2 = 0;
long setpoint_delta = 0;
long setpoint_delta2 = 0;
int socket_error = 0;
int system_state = 1;

int switch_states[8];
int switch_states_old[8];
int zeroed_switcheds[8]={0};


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
	pthread_t pth;	// this is our thread identifier
	pthread_create(&pth,NULL,threadFunc,NULL);
	
	/*
	------------------------------------------
	Setup motors
	------------------------------------------
	*/
	struct axis_motor motor_1 = {.accGoal = 5000, .velGoal = 10, .posGoal = 0, .accCurrent = 100, .velCurrent = 0, .posCurrent = 0, .posGoalCurrent = 0, .dutyCyle = 0, .setpointUpdated = 0, .startupFlag = 1};
	struct axis_motor motor_2 = {.accGoal = 5000, .velGoal = 10, .posGoal = 0, .accCurrent = 100, .velCurrent = 0, .posCurrent = 0, .posGoalCurrent = 0, .dutyCyle = 0, .setpointUpdated = 0, .startupFlag = 1};
	struct axis_motor motor_3 = {.accGoal = 5000, .velGoal = 10, .posGoal = 0, .accCurrent = 100, .velCurrent = 0, .posCurrent = 0, .posGoalCurrent = 0, .dutyCyle = 0, .setpointUpdated = 0, .startupFlag = 1};
	struct axis_motor motor_4 = {.accGoal = 5000, .velGoal = 10, .posGoal = 0, .accCurrent = 100, .velCurrent = 0, .posCurrent = 0, .posGoalCurrent = 0, .dutyCyle = 0, .setpointUpdated = 0, .startupFlag = 1};

	
	/*
	------------------------------------------
	Setup FPGA communication
	------------------------------------------
	*/
	int ret;
	void *virtual_base;
	int fd;
	int i;
	int j;
	int k;
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

	h2p_lw_quad_addr[0]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[1]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_1_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[2]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_2_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[3]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_3_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[4]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_4_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[5]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_5_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[6]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_6_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_quad_addr[7]=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + QUAD_PIO_7_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	
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
	alt_write_word(h2p_lw_quad_reset_addr, 0xFFFFFFFF);
	usleep(1000*500);
	alt_write_word(h2p_lw_quad_reset_addr, 0);
	
	//set PWM values to zero
	for(j=0;j<8;j++){
		alt_write_word(h2p_lw_pwm_values_addr[j], 255);
	}
	
	//alt_write_word(h2p_lw_pwm_values_addr[7], 150);
	
	//calculate PID values
	uint32_t PID_values = 0;
	PID_values = PID_values | (P+I+D);
	PID_values = PID_values | ((P+2*D) << 8);
	PID_values = PID_values | (D << 16);
	alt_write_word(h2p_lw_pid_values_addr, PID_values);
	uint32_t pid_values_read = *(uint32_t*)h2p_lw_pid_values_addr;
	printf("Sent: P: %d, I: %d, D: %d\n\n", P, I, D);
	printf("P: %d, I: %d, D: %d\n\n", (uint8_t)(pid_values_read & (0x000000FF)), (uint8_t)((pid_values_read & (0x0000FF00))>>8), (uint8_t)((pid_values_read & (0x00FF0000))>>16));
	
	
	/*
	------------------------------------------
	Run controller
	------------------------------------------
	*/
	
	int curr_state = 0;
	int pin_state;
	int err = 0;
	int flag = 0;
	int homed = 0;

	int setpoint_flag = 0;
	int setpoint_flag2 = 0;

	int dir_bitmask;
	long myCounter = 0;
	
	uint32_t switches;
	int32_t setpoint[8] = {0};
	
	struct timeval timer_usec; 

  	long long int timestamp_usec; /* timestamp in microsecond */
	
	for(j = 0; j<8; j++){
		position_setpoints[j] = 0;
	}
	
	while(1)
	{
		
		uint32_t mask = createMask(0,30);
		uint32_t output = alt_read_word(h2p_lw_quad_addr[j]) & mask;
		if(myCounter!=0){
			for(j=0;j<8;j++){
				switch_states_old[j] = switch_states[j];
			}
		}
		switches = alt_read_word(h2p_lw_limit_switch_addr);
		switch_states[0] = (switches&1<<4)>0;
		switch_states[1] = (switches&1<<6)>0;
		switch_states[2] = (switches&1<<7)>0;
		switch_states[3] = (switches&1<<5)>0;
		
		/*if(myCounter==0){
			for(j=0;j<4;j++){
				switch_states_old[j] = switch_states[j];
			}
		}*/
		//j = 0;
		for(j = 0; j<4; j++){
			/*
			if(!homed && myCounter % 10 == 0){
			//position_setpoints, switch_states
					if(!zeroed_switcheds[j]){
						if(switch_states[j]==0){
							position_setpoints[j]+=1;
						}
						else{
							position_setpoints[j]-=1;
						}
					zeroed_switcheds[j] = switch_states_old[j] != switch_states[k];
					}
				if((zeroed_switcheds[0] + zeroed_switcheds[1] + zeroed_switcheds[2] + zeroed_switcheds[3]) == 4)
					homed = 1;
				}
			}*/
			
			int32_t nativeInt = createNativeInt(output, 30);
			int32_t error = nativeInt -position_setpoints[j];
			alt_write_word(h2p_lw_pid_input_addr[j], error);
			int32_t check_error = alt_read_word(h2p_lw_quad_addr[j]);
			
			//int32_t pid_output = *(int32_t*)(h2p_lw_pid_output_addr[j]);
			int32_t pid_output = alt_read_word(h2p_lw_pid_output_addr[j]);
			int8_t sign_pid_output = (pid_output >= 0) - (pid_output < 0);
			int32_t positive_pid_output = (pid_output>=0);
			int32_t pid_output_cutoff = fabs(pid_output)*(fabs(pid_output) <= 255) + 255*(fabs(pid_output) > 255);	
			
			if(j < 4 ){
				alt_write_word(h2p_lw_pwm_values_addr[j], (255-pid_output_cutoff));
				
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
				alt_write_word(h2p_lw_pwm_values_addr[j], (255-pid_output_cutoff));
				dir_bitmask = alt_read_word(h2p_lw_gpio_addr);
				if(positive_pid_output == 0)
					dir_bitmask |= (1<<(j-4));
				else
					dir_bitmask &= ~(1<<(j-4));
				alt_write_word(h2p_lw_gpio_addr, dir_bitmask);
				}
			}
			
			
			if(myCounter%1000 == 0){
				//printf("Axis: %d;Position Setpoint: %d; Current count: %d; Error: %d; Current PID output, unsigned: %d; Current PID output, signed: %d; Current Sign: %d; Cutoff output: %d\n", j, position_setpoints[j], nativeInt, error, pid_output, pid_output*sign_pid_output, sign_pid_output, pid_output_cutoff);
				//printf("Error: %d; Error read: %d; PID out: %d\n", error, check_error, pid_output);
				int a = 0;
			}
		}
		if(myCounter%100 == 0)
		{
			switches = alt_read_word(h2p_lw_limit_switch_addr);
			printf("%d,%d,%d,%d\n", switch_states[0],switch_states[1],switch_states[2],switch_states[3]);
		}
		//if(myCounter%100 == 0)
		//	printf("\n\n");
		/*LEDR_AllOn();
		usleep(1000*50);
		LEDR_AllOff();
		usleep(1000*50);
		*/
		myCounter++;
		usleep(2000);
	}
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );

	}
	close( fd );
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
	char * pch;
	int sockfd, newsockfd, portno;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n, j;
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
    if (newsockfd < 0){ 
        error("ERROR on accept");
        return;
    }

    bzero(buffer,256);
	char *str;

	str=(char*)arg;

	while(system_state == 1 && socket_error == 0)
	{
		nanosleep((const struct timespec[]){{0, 5000000L}}, NULL);

		//reads information from socket
		n = read(newsockfd,buffer,255);

   		if (n < 0){
   			error("ERROR reading from socket");
   			break;
   		}
//    		printf("Here is the message: %s\n",buffer);
    	n = write(newsockfd,"I got your message",18);
    	if (n < 0){
    		error("ERROR writing to socket");
    		break;
    	}

		pch = strtok (buffer,"bd ");
    	
    	//printf("1: %d", atoi(pch));
    	global_motor_1.posGoal = atoi(pch);
    	global_motor_3.posGoal = atoi(pch);

    	for(j = 0; j<8; j++){
				position_setpoints[j] = atoi(pch);
		}
    	pch = strtok (NULL,"* ");
    	//printf(" 2: %d", atoi(pch));
    	global_motor_1.velGoal = atof(pch);
    	global_motor_3.velGoal = atof(pch);
    	pch = strtok (NULL,"bd ");
    	//printf(" 3: %d", atoi(pch));
    	global_motor_2.posGoal = atoi(pch);
    	global_motor_4.posGoal = atoi(pch);
		pch = strtok (NULL,"bd ");
		//printf(" 4: %d\n", atoi(pch));
		global_motor_2.velGoal = atof(pch);
		global_motor_4.velGoal = atof(pch);
    	pch = strtok (NULL,"bd ");
    	//printf("parsed message, setpoint 1: %lf, velocity 1: %lf, setpoint 2: %lf, velocity 2: %lf\n", global_motor_1.posGoal, global_motor_1.velGoal, global_motor_2.posGoal, global_motor_2.velGoal);
		setpoint_update = 0;
		setpoint_delta = 0;
		setpoint_update2 = 0;
		setpoint_delta2 = 0;

		global_motor_1.setpointUpdated = 1;
		global_motor_2.setpointUpdated = 1;
		global_motor_3.setpointUpdated = 1;
		global_motor_4.setpointUpdated = 1;

	}

	close(newsockfd);
   close(sockfd);
   return;
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
			
			alt_write_word(h2p_lw_pwm_values_addr[j], (255-25));
			
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

			alt_write_word(h2p_lw_pwm_values_addr[j], (255-25));
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
