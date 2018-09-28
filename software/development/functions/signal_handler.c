#include "../main.h"
#include <stdio.h>
#include <unistd.h>
#include <strings.h>


void my_handler(int s){
		int n;
		char write_buffer[256];
		bzero(write_buffer,256);

		printf("\nCaught signal %d\n",s);

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

        exit_flag = 1; 
}