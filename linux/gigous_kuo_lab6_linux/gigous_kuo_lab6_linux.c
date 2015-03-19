#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "gigous_kuo_lab6_linux.h"
#include "omapl138_gpiofuncs.h"
#include "../../sharedmem_com/sharedmem.h"

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
#define DELAYTIME 100000

#define REF_RIGHT_WALL 2
#define FRONT_ERROR_THRESHOLD 3
#define KP_RIGHT_WALL 4
#define KP_FRONT_WALL 5
#define FRONT_TURN_VELOCITY 6
#define TURN_COMMAND_SATURATION 7


// functions declared at bottom of this file
int mygetch(void);
void gs_killapp(int s);
int init_server(void);
void sd_signal_handler_IO (int status);
int run_server(void);
void writefloats_sharedmem_5times(void);

sharedmemstruct* myshared;  // pointer to shared memory stucture used to communicate between Linux and the OMAPL138's DSP

volatile GPIOregs* mygpio;  // pointer to GPIO registers so Linux can control GPIO Pins
int TCPIPtransmissionerror = 0;		// Initialize transmission error count
int TCPIPbeginnewdata = 0;
int TCPIPdatasize = 0;
int numTXChars = 0;

char tmpbuf[LINUX_COMSIZE];

long tcpipcount = 0;

#define INBUFFSIZE (256)
#define OUTBUFFSIZE (INBUFFSIZE + 2)
volatile char TCPIPMessageArray[INBUFFSIZE];
volatile char tempReadbackToCleanCache[INBUFFSIZE];
volatile int tempReadbackSize;

volatile unsigned int delaycount = 0;

int fd;  // variables used to map to physical memory addresses
void *map_base;
off_t target;

char inbuf[INBUFFSIZE];  // TCPIP input and output buffers
unsigned char outbuf[OUTBUFFSIZE];
int accepted_skt;

float DSPFloats[NUM_FLOATS_FROM_LINUX_TO_DSP];  // temporay variables used in the communication with DSP core    
volatile float tempFloats[NUM_FLOATS_FROM_LINUX_TO_DSP];
int firsttime = 1;

/*
* main()
*   process command line input
*/
int main (int argc, char **argv)
{
	int index = 0;
	char buffer[200];  // used by fgets to read character string typed by user.
	char mychar;
	int inputdone = 0;
	float tempfloat = 0;
	float DVel = 0;
	float turn = 0.0;
	
	float ref_right_wall;
	float front_error_threshold;
	float Kp_right_wall = 0.002;
	float Kp_front_wall = 0.001;
	float front_turn_velocity = 0.4;
	float turn_command_saturation = 1.0;
	
	/* ****************************************************************/
	/*  Memory map to physical memory spaces of the GPIO control registers and shared memory space */	
	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
		printf("/dev/mem could not be opened.\n");
		exit(1);
	} else {
		printf("/dev/mem opened.\n");
	}
	fflush(stdout);
	
	
	target = 0x01E26000;
	/* Map one page for shared memory structure*/
	map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target);
	if(map_base == (void *) -1) {
		printf("Memory map failed.\n");
	} else {
		printf("gpio Struct mapped at address %p.\n", map_base);
	}
	fflush(stdout);
	mygpio = (GPIOregs *) map_base;
	
	target = 0x80000000;
	/* Map one page for shared memory structure*/
	map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
	if(map_base == (void *) -1) {
		printf("Memory map failed.\n");
	} else {
		printf("Shared Struct mapped at address %p.\n", map_base);
	}
	fflush(stdout);
	
	myshared = (sharedmemstruct *) map_base;
	/* ******************************************************************/

	GPIO_setOutput(mygpio,LVDATA_TO_LINUX_BANK,LVDATA_TO_LINUX_FLAG,OUTPUT_HIGH);  // Set = linux is ready for LV data from DSP
	GPIO_setOutput(mygpio,LVDATA_FROM_LINUX_BANK,LVDATA_FROM_LINUX_FLAG,OUTPUT_LOW);  // Clear = linux will write LV data for shared memory for DSP
	GPIO_setOutput(mygpio,DATA_TO_LINUX_BANK,DATA_TO_LINUX_FLAG,OUTPUT_HIGH);  // Set = linux is ready for data from DSP
	GPIO_setOutput(mygpio,DATA_FROM_LINUX_BANK,DATA_FROM_LINUX_FLAG,OUTPUT_LOW);  // Clear = linux will write data for shared memory for DSP
	
	for (index=0;index<NUM_FLOATS_FROM_LINUX_TO_DSP;index++) {
		DSPFloats[index] = 0.0;
	}

	DSPFloats[REF_RIGHT_WALL] = 500;
	DSPFloats[FRONT_ERROR_THRESHOLD] = 500;
	DSPFloats[KP_RIGHT_WALL] = 0.002;
	DSPFloats[KP_FRONT_WALL] = 0.001;
	DSPFloats[FRONT_TURN_VELOCITY] = 0.25;
	DSPFloats[TURN_COMMAND_SATURATION] = 1.0;


	writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory

	// stay in this while loop receiving input from the user until user exits by selecting option e or v  
	while (!inputdone) {
		printf("\n\n");
		printf("Menu of Selections\n");
		printf("DO NOT PRESS CTRL-C when in this menu selection\n");
		printf("e - Exit Application\n");
		printf("v - Done.  Connect to LabView\n");
		printf("s - enter Desired Velocity Setpoint\n");
		printf("q - increment Left\n");
		printf("p - increment Right\n");
		
		printf("1 - ref right wall\n");
		printf("2 - front error threshold\n");
		printf("3 - Kp right wall\n");
		printf("4 - Kp front wall\n");
		printf("5 - front turn velocity\n");
		printf("6 - turn command saturation\n");

		mychar = (char) mygetch();
		
		switch (mychar) {
		case 'q':
			// if (turn > 0.0) {
			// 	turn = 0.0;
			// } else {
			turn = turn - 0.2;
			// }
			printf("turn =%.3f\n",turn);
			
			// send new turn value to DSP
			DSPFloats[1] = turn;
			// Writes the DSPFloats array values to shared memory for DSP to read
			writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
			break;
		case 'p':                                
			// if (turn < 0.0) {
			// 	turn = 0.0;
			// } else {
			turn = turn + 0.2;
			// }
			printf("turn =%.3f\n",turn);
			
			// send new turn value to DSP
			DSPFloats[1] = turn;
			writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
			break;
		case 'v':
			inputdone = 2;
			break;
		case 'e':
			inputdone = 1;
			break;
		case 's':
			// request new Velocity reference point from user and send it to DSP
			printf("Enter Desired Velocity Setpoint\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					DVel = tempfloat;
					printf("DVel = %.3f\n",DVel);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: DVel not changed\n");
			}
			// send new Vref value to DSP
			DSPFloats[0] = DVel;
			writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
			break;

		case '1':
			// request new Velocity reference point from user and send it to DSP
			printf("Enter Desired ref_right_wall :)\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					ref_right_wall = tempfloat;
					printf("ref_right_wall = %.3f\n",ref_right_wall);
				}  else {
					printf("Error: Non numerical value typed. You messed up.\n");
				}
			} else {
				printf("Error: ref_right_wall not changed\n");
			}
			// send new Vref value to DSP
			DSPFloats[REF_RIGHT_WALL] = ref_right_wall;
			writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
			break;

		case '2':
			// request new Velocity reference point from user and send it to DSP
			printf("Enter Desired front_error_threshold :)\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					front_error_threshold = tempfloat;
					printf("front_error_threshold = %.3f\n",front_error_threshold);
				}  else {
					printf("Error: Non numerical value typed. You messed up.\n");
				}
			} else {
				printf("Error: front_error_threshold not changed\n");
			}
			// send new Vref value to DSP
			DSPFloats[FRONT_ERROR_THRESHOLD] = front_error_threshold;
			writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
			break;

		case '3':
			// request new Velocity reference point from user and send it to DSP
			printf("Enter Desired Kp_right_wall :)\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					Kp_right_wall = tempfloat;
					printf("Kp_right_wall = %.3f\n",Kp_right_wall);
				}  else {
					printf("Error: Non numerical value typed. You messed up.\n");
				}
			} else {
				printf("Error: Kp_right_wall not changed\n");
			}
			// send new Vref value to DSP
			DSPFloats[KP_RIGHT_WALL] = Kp_right_wall;
			writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
			break;

		case '4':
			// request new Velocity reference point from user and send it to DSP
			printf("Enter Desired Kp_front_wall :)\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					Kp_front_wall = tempfloat;
					printf("Kp_front_wall = %.3f\n",Kp_front_wall);
				}  else {
					printf("Error: Non numerical value typed. You messed up.\n");
				}
			} else {
				printf("Error: Kp_front_wall not changed\n");
			}
			// send new Vref value to DSP
			DSPFloats[KP_FRONT_WALL] = Kp_front_wall;
			writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
			break;

		case '5':
			// request new Velocity reference point from user and send it to DSP
			printf("Enter Desired front_turn_velocity :)\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					front_turn_velocity = tempfloat;
					printf("front_turn_velocity = %.3f\n",front_turn_velocity);
				}  else {
					printf("Error: Non numerical value typed. You messed up.\n");
				}
			} else {
				printf("Error: front_turn_velocity not changed\n");
			}
			// send new Vref value to DSP
			DSPFloats[FRONT_TURN_VELOCITY] = front_turn_velocity;
			writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
			break;

		case '6':
			// request new Velocity reference point from user and send it to DSP
			printf("Enter Desired turn_command_saturation :)\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					turn_command_saturation = tempfloat;
					printf("turn_command_saturation = %.3f\n",turn_command_saturation);
				}  else {
					printf("Error: Non numerical value typed. You messed up.\n");
				}
			} else {
				printf("Error: turn_command_saturation not changed\n");
			}
			// send new Vref value to DSP
			DSPFloats[TURN_COMMAND_SATURATION] = turn_command_saturation;
			writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
			break;
		default:
			
			break;
		}
	} 

	// stop robot
	DSPFloats[0] = 0.0;  // vref
	DSPFloats[1] = 0.0;  //turn
	writefloats_sharedmem_5times();  // Our hack way of "hopefully" making sure our data is not sitting in cache but in the actual shared memory
	
	if (inputdone == 2) {  // Start TCPIP connection with LabView application    
		if (argc ==2) {  // port number enter when starting this application
			printf("using port %s\n",argv[1]);
			gs_port_coms = atoi(argv[1]);
		}
		printf("Setting signal handler...\n");
		signal(SIGKILL, gs_killapp);
		signal(SIGINT, gs_killapp);
		printf("...OK\n");
		
		while (!gs_exit) {
			gs_quit = 0;
			if (!firsttime) {
				printf("Waiting 2 Seconds to make sure socket data is cleaned from the network.\n");
				sleep(2);
			}
			printf("Initializing listening connection...\n");
			init_server();
			firsttime = 0;
			printf("...OK\n");

			printf("Accepting connections...\n");
			run_server();  // sit in this function until TCPIP connection disconnects
		}	
		
		if(munmap((void*) mygpio, MAP_SIZE) == -1) {
			printf("Memory unmap failed.\n");	
		}
		if(munmap((void*) myshared, MAP_SIZE) == -1) {
			printf("Memory unmap failed.\n");	
		}
	}
}


/*
* run_server()
*   This function runs until the server quits and it:
*
*   1. accepts incoming TCP connections
*   2. reads from ready sockets and writes incoming messages
*      to the serial port / controls devices
*   3. reads from serial port and prints
*/
int run_server(void)
{

	sigset_t sigio_set; 
	struct sigaction saio;           /* definition of signal action */
	static struct sockaddr_in addr;  
	int i;

	// Accept new connection
	accepted_skt = connaccept(gs_coms_skt, &addr);
	// We don't wanna block either source
	sock_set_nonblocking(accepted_skt);
	int flags = fcntl(accepted_skt, F_GETFL,0);
	fcntl (accepted_skt,F_SETFL,flags|FASYNC);
	fcntl(accepted_skt, F_SETOWN, getpid());

	sigemptyset(&sigio_set);  // set up call back function called when new data received over ethernet. function "sd_signal_handler_IO"
	sigaddset(&sigio_set,SIGIO); 
	saio.sa_handler = sd_signal_handler_IO;
	saio.sa_flags = SA_SIGINFO;
	sigemptyset(&saio.sa_mask);
	saio.sa_mask = sigio_set;
	if (sigaction(SIGIO,&saio,NULL)) {
		printf("Error in sigaction()\n");
		return 1;
	}

	if (accepted_skt>0) {
		printf("Connection accepted from %d.%d.%d.%d\n",
		(addr.sin_addr.s_addr&0x000000ff),      
		(addr.sin_addr.s_addr&0x0000ff00)>>8,
		(addr.sin_addr.s_addr&0x00ff0000)>>16,
		(addr.sin_addr.s_addr&0xff000000)>>24);
	}
	printf(".\n");
	while (!gs_quit) {  // sit inside this while checking for new data from the DSP until TCPIP connection disconnected
		sched_yield();  // allow other processes to run if necessary
		if  (GPIO_getOutput(mygpio,LVDATA_TO_LINUX_BANK,LVDATA_TO_LINUX_FLAG) == 0) { // New data from DSP ?    
			
			if (myshared->DSPSend_size > LINUX_COMSIZE) myshared->DSPSend_size = LINUX_COMSIZE;
			for (i=0;i<myshared->DSPSend_size;i++) {  // read character string communicated to Linux from the DSP core
				tmpbuf[i] = myshared->DSPSend_buf[i];
			}
			tmpbuf[i] = '\0';
			numTXChars = sprintf(((char *) outbuf),"%s\r\n",tmpbuf);  // create string to be sent to LV with added start and stop character.  
			write(accepted_skt, outbuf, numTXChars);  // write string to TCPIP socket, LV waiting on the other end.

			printf("%s\n",tmpbuf);
			
			GPIO_setOutput(mygpio,LVDATA_TO_LINUX_BANK,LVDATA_TO_LINUX_FLAG,OUTPUT_HIGH); // set flag indicating to DSP that Linux is ready for more data
		}
		
	}
}



void sd_signal_handler_IO (int status)
{
	int i,j;
	int numrecChars = 0;
	
	for (i=0;i<64;i++) {
		inbuf[i] = '\0';
	}
	numrecChars = read(accepted_skt, inbuf, 64);  // read string of characters from TCPIP socket 
	if (numrecChars > 0)  {  // if 1 or more characters read
		for (i=0;i<numrecChars;i++) {  // loop through all the recieved characters
			if (!TCPIPbeginnewdata) {// Only true if have not yet begun a message
				if (253 == (unsigned char)inbuf[i]) {// Check for start char
					TCPIPdatasize = 0;		// amount of data collected in message set to 0
					TCPIPbeginnewdata = 1;		// flag to indicate we are now collecting a message
				}
			} else {	// Filling data
				// Dont go too large... limit message to 256 chars  this can be made larger if you change inbuf size
				if ((TCPIPdatasize < 256) && ((unsigned char)inbuf[i] != 255)) {  // also check that it is not stop character	
					TCPIPMessageArray[TCPIPdatasize] = inbuf[i];  // add character to message array				
					TCPIPdatasize++;  // increment number of characters in message
				} else {  // too much data or 255 char received
					if ((unsigned char)inbuf[i] != 255) {// check if end character recvd if not print overflow times
						// Not received
						TCPIPtransmissionerror++;
						printf("TXerrors = %d\n",TCPIPtransmissionerror);
					} 
					// Whether or not message terminated correctly, send data to other tasks
					TCPIPMessageArray[TCPIPdatasize] = '\0'; 	// Null terminate the string
					TCPIPdatasize++;  // increment by one so that the Null is copied to shared memory 
					printf("New Test String:%s %d\n",TCPIPMessageArray,TCPIPdatasize);
					

					if (GPIO_getOutput(mygpio,LVDATA_FROM_LINUX_BANK,LVDATA_FROM_LINUX_FLAG)  == 0) {  // is DSP ready for new data?
						
						// WRITE date to shared memory
						// until we figure out how to invalidate and writeback the ARM's cache we write to the shared memory 5 times and "hope" that it cleans the cache.
						// first write to shared memory
						myshared->DSPRec_size = TCPIPdatasize;
						for (j=0;j<LINUX_COMSIZE;j++) {
							myshared->DSPRec_buf[j] = TCPIPMessageArray[j];
						}
						// read back right away to attempt to clear data out of cache
						tempReadbackSize = myshared->DSPRec_size;
						for (j=0;j<LINUX_COMSIZE;j++) {
							tempReadbackToCleanCache[j] = myshared->DSPRec_buf[j];
						}

						// second write
						for (delaycount=0;delaycount<DELAYTIME;delaycount++){ }
						myshared->DSPRec_size = TCPIPdatasize;
						for (j=0;j<LINUX_COMSIZE;j++) {
							myshared->DSPRec_buf[j] = TCPIPMessageArray[j];
						}

						// third write
						for (delaycount=0;delaycount<DELAYTIME;delaycount++){ }
						myshared->DSPRec_size = TCPIPdatasize;
						for (j=0;j<LINUX_COMSIZE;j++) {
							myshared->DSPRec_buf[j] = TCPIPMessageArray[j];
						}

						// fourth write
						for (delaycount=0;delaycount<DELAYTIME;delaycount++){ }
						myshared->DSPRec_size = TCPIPdatasize;
						for (j=0;j<LINUX_COMSIZE;j++) {
							myshared->DSPRec_buf[j] = TCPIPMessageArray[j];
						}

						// fifth write
						for (delaycount=0;delaycount<DELAYTIME;delaycount++){ }
						myshared->DSPRec_size = TCPIPdatasize;
						for (j=0;j<LINUX_COMSIZE;j++) {
							myshared->DSPRec_buf[j] = TCPIPMessageArray[j];
						}
						// set flag to tell DSP new data is ready for DSP to read.
						GPIO_setOutput(mygpio,LVDATA_FROM_LINUX_BANK,LVDATA_FROM_LINUX_FLAG,OUTPUT_HIGH);
					}
					TCPIPbeginnewdata = 0;	// Reset the flag
					TCPIPdatasize = 0;	// Reset the number of chars collected
				}	
			}
		}
	} else {
		printf("numrecChars = %d\n",numrecChars);
		printf("\nReseting\n");
		gs_quit = 1;
		close(gs_coms_skt);  
	}

	// call this kill when kill string is sent
	//      gs_killapp(0);

	return;
}

// until we figure out how to invalidate and writeback the ARM's cache we write to the shared memory 5 times and "hope" that it cleans the cache.
void writefloats_sharedmem_5times(void) {
	int j=0;
	
	if (GPIO_getOutput(mygpio,DATA_FROM_LINUX_BANK,DATA_FROM_LINUX_FLAG)  == 0) {  // DSP ready for new data

		for (j=0;j<NUM_FLOATS_FROM_LINUX_TO_DSP;j++) {
			myshared->Floats_to_DSP[j] = DSPFloats[j];
		}
		// one attempt to clean the cache
		for (j=0;j<NUM_FLOATS_FROM_LINUX_TO_DSP;j++) {
			tempFloats[j] = myshared->Floats_to_DSP[j];
		}
		// small delay and second attempt make cache clean itself
		for (delaycount=0;delaycount<DELAYTIME;delaycount++){ }
		for (j=0;j<NUM_FLOATS_FROM_LINUX_TO_DSP;j++) {
			myshared->Floats_to_DSP[j] = DSPFloats[j];
		}

		// small delay and third attempt make cache clean itself
		for (delaycount=0;delaycount<DELAYTIME;delaycount++){ }
		for (j=0;j<NUM_FLOATS_FROM_LINUX_TO_DSP;j++) {
			myshared->Floats_to_DSP[j] = DSPFloats[j];
		}

		// small delay and forth attempt make cache clean itself
		for (delaycount=0;delaycount<DELAYTIME;delaycount++){ }
		for (j=0;j<NUM_FLOATS_FROM_LINUX_TO_DSP;j++) {
			myshared->Floats_to_DSP[j] = DSPFloats[j];
		}

		// small delay and fifth attempt make cache clean itself
		for (delaycount=0;delaycount<DELAYTIME;delaycount++){ }
		for (j=0;j<NUM_FLOATS_FROM_LINUX_TO_DSP;j++) {
			myshared->Floats_to_DSP[j] = DSPFloats[j];
		}
		GPIO_setOutput(mygpio,DATA_FROM_LINUX_BANK,DATA_FROM_LINUX_FLAG,OUTPUT_HIGH);
	}	
}	



int mygetch(void)
{
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

/*  
* gs_killapp()
*   ends application safely
*
*/
void gs_killapp(int s)
{
	printf("\nTerminating\n");

	gs_quit = 1;
	gs_exit = 1;
	close(gs_coms_skt);
	return;
}


/*
* init_server()
*   Starts up listening socket
*/
int init_server(void)
{
	int on = 1;

	// Communications sockets,
	// tcp:
	gs_coms_skt = tcpsock();  

	if (setsockopt(gs_coms_skt, SOL_SOCKET,SO_REUSEADDR, (char*)&on, sizeof(on)) < 0) {
		printf("Error Reusing Socket\n");
	}

	sockbind(gs_coms_skt, gs_port_coms);
	socklisten(gs_coms_skt);
	printf("Listening on port %d\n.",gs_port_coms);
	sock_set_blocking(gs_coms_skt);
}