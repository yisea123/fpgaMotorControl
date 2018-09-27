#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "../main.h"

//Heartbeat Function Thread
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

//Communication Function Thread
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

void error(const char *msg)
{
    perror(msg);
    socket_error = 1;
}