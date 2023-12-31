// Link layer protocol implementation
#include "link_layer.h"
#include "constants.h"
#include "frame.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

// Global variables
volatile int STOP;

unsigned int alarmCount = 0;
int alarmEnabled = FALSE;
int fd;

unsigned char buf[MAX_FRAME_SIZE] = {0};

LinkLayer cp;
struct termios oldtioSave; // Save current port settings


static volatile int sequenceNumber = 0;  // Initial sequence number
static volatile int lastAckReceived = -1; // Last acknowledged sequence number


// alarm setter function
void setAlarm() {
    alarm(3); // Set the initial alarm for 3 seconds
    alarmEnabled = TRUE;
}

void removeAlarm() {
    alarmEnabled = FALSE;
    alarm(0);
    alarmCount = 0;
}

void alarmHandler() {
    alarmEnabled = FALSE;
    alarmCount++;
}

////////////////////////////////////////////////
// LLOPEN (completed) sort of
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    cp = connectionParameters;
    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(cp.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror("Error opening tty");
        return -1;
    }

    struct termios oldtio;
    oldtioSave = oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = cp.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 1;  // Blocking read until 5 chars received

    // Now clean the line and activate the settings for the port
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        close(fd);
        return -1; // Return an error code
    }

    if(cp.role == LlTx) {
        Frame setFrame = createControlFrame(A_FRAME_SENDER, C_SET);

	if(write(fd, setFrame.data, setFrame.size) == -1)
		return -1;

        // set alarm handler
        (void)signal(SIGALRM, alarmHandler);
        // set alarm
        setAlarm();

        if(checkControlFrame(fd, A_ANSWER_RECEIVER, C_UA) == 1) {
            printf("Received UA FRAME from receiver\n");
            removeAlarm();
            return 1;
        }

        // send SET packet and wait for UA response nRetransmissions times
        while(alarmCount < cp.nRetransmissions) {
            if(!alarmEnabled) {
                printf("Alarm #%d: Resending the packet.\n", alarmCount);

                // Send SET packet
                Frame setFrame = createControlFrame(A_FRAME_SENDER, C_SET);
                if(write(fd, setFrame.data, setFrame.size) == -1) {
                    printf("ERROR : sending SET frame exiting...\n");
                    return -1;
                } else {
                    printf("SET FRAME RE-sent.\n");
                }

                if(checkControlFrame(fd, A_ANSWER_RECEIVER, C_UA)) {
                    printf("Received UA frame from receiver.\n");
                    removeAlarm();
                    return 1;
                }
		setAlarm();
            }
	}
	printf("Could not send SET frame to the receiver!\n");
        printf("Reached the maximum of tentatives %d\n", alarmCount);

	return -1;

    } else if(cp.role == LlRx) {
        while(alarmCount < cp.nRetransmissions) {
                if(checkControlFrame(fd, A_FRAME_SENDER, C_SET)) {
                    printf("Received correct SET FRAME from sender.\n");
                    removeAlarm();

                    // Send UA packet
                    Frame uaFrame = createControlFrame(A_ANSWER_RECEIVER, C_UA);
                    if(write(fd, uaFrame.data, uaFrame.size) == -1) {
                        printf("ERROR : sending UA frame exiting...\n");
                        return -1;
                    } else {
                        printf("UA FRAME sent.\n");
                        return 1;
                    }
                }

		if(!alarmEnabled){
		       	setAlarm();
			printf("Ola\n");
		}
            
        }
	printf("ERROR : receiver didn't got SET frame\n");	
	printf("Reached maximum tentatives : %d\n", alarmCount);
    }
    return -1;
}

// Returns 1 if the ACK was sent correctly
// Returns -1 if the ACK was corrupted
// Returns 2 received a REJ
int checkACKResponse(int fd, int expectedSequenceNumber) {
    printf("In the checkACKResponse function\n");
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    while (1) {
        read(fd, recv, 1);
        printf("Received: %x\n", recv[0]);
        switch (rc) {
            case Start: {
                if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case Flag_Rcv: {
                if (recv[0] == A_ANSWER_RECEIVER) {
                    rc = A_Rcv;
                    ac[0] = A_ANSWER_RECEIVER;
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case A_Rcv: {
                if (recv[0] == (0x05 | (expectedSequenceNumber << 7))) {
                    // Check if the received ACK/NACK corresponds to the expected sequence number.
                    rc = C_Rcv;
                    ac[1] = recv[0];
                    break;
                } 
		else if(recv[0] == 0x81){
			//Check if receive a REJ
			return 2;
		}
		else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case C_Rcv: {
                if (recv[0] == (ac[0] ^ ac[1])) {
                    rc = Bcc_OK;
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case Bcc_OK: {
		printf("In the BCCok\n");
                if (recv[0] == FLAG) {
                    return 1;
                }
                return -1;
            }
	   // Just added to stop the warnings, it not possible to reach that case
	   case Data_Rcv : 
		return -1;
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) // TODO: MAKE THIS SEND ONLY ONE PACKET FROM THE APPLICATION WITH THE MAXIMUM SIZE OF 1000 BYTES
{
    if(bufSize > MAX_PAYLOAD_SIZE) {
        printf("Error: bufSize is bigger than the maximum payload size.\n");
        return -1;
    }
    // Create the I-frame
    Frame frame = createInformationFrame(buf, bufSize, sequenceNumber, A_FRAME_SENDER);

    
    // Send frame
    write(fd, frame.data, frame.size);

    setAlarm();

    // Check for ACK response
    if (checkACKResponse(fd, (sequenceNumber+1) %2) == 1) {
        // ACK received
        removeAlarm();
        lastAckReceived = sequenceNumber;
        sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sqcN
        return 1;
    }

    /*
     *Probably exists the case that the receiver sends the ACK, meaning that 
     it alredy 
     */
    // Case the ACK response was invalid or it was a REJ
     while (alarmCount < cp.nRetransmissions) {
         if (alarmEnabled) {
             // Handle retransmission
             printf("Alarm #%d: Resending the packet.\n", alarmCount);
             write(fd, frame.data, frame.size);

             // Check for ACK response
             if (checkACKResponse(fd, sequenceNumber) == 1) {
                 removeAlarm();
                 lastAckReceived = sequenceNumber;
                 sequenceNumber = (sequenceNumber + 1) % 2;// Toggle sqcN
                 return 1;
             }
         }
	 else {
		 setAlarm();
	 }
     }

    // Could not receive the correct ACK response
    return -1; 
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) // packet has 1000bytes size
{
    
    // the expected sequence number is the sequence number we have currenlty
    int dataSize = decodeInformationFrame(fd, sequenceNumber, packet);

    // If the frame is corrupted, send a REJ 
    if (dataSize == -1) {
        printf("Frame corrupted!!!\n");

        // Send rej packet  
        Frame rejFrame = createControlFrame(A_ANSWER_RECEIVER, C_REJ0 | sequenceNumber << 7); // we reject an information frame with sequence number 
        
	// Try 3 times to send the REJ frame
	for(int i = 0; i < 3; i++){
		if(write(fd, rejFrame.data, rejFrame.size) != -1) return 1;
	        printf("Attempt #%d to send REJ FRAME\n", i);
	}
	
	// Case failded to send the REJ frame	
        return -1;
    } else if(dataSize == -2){
	    printf("Frame duplicated! Send RR and drop it...\n");
	    Frame rrFrame = createControlFrame(A_ANSWER_RECEIVER, C_RR0 | (sequenceNumber) << 7);

	    /*Try 3 times, the -2 return will be for the application layer
	     to understand that the packet is a duplicated one, and wait for
	     the next*/
	    for(int i = 0; i < 3; i++){
		    if(write(fd, rrFrame.data, rrFrame.size) != -1) return -2;
	            printf("Attempt #%d to send RR FRAME\n", i);
	    }
	    return -1;
	}

    sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sequence number

    // This is already creating correctly the RR0 and RR1
    Frame rrFrame = createControlFrame(A_ANSWER_RECEIVER, C_RR0 | (sequenceNumber) << 7);

    // Try 3 times to send the RR FRAME
    for(int i = 0; i < 3; i++){
	if(write(fd, rrFrame.data, rrFrame.size) != -1) return dataSize;
	printf("Attempt #%d to send RR FRAME\n", i);
    }

    return -1;

}

// return 1 if the connection was terminated correctly or -1 if not
int terminate_connection(int fd) {
    if(tcsetattr(fd, TCSANOW, &oldtioSave) == -1) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    close(fd);
    return 1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    if(cp.role == LlTx){
        // Send DISC frame if we are the sender, waits for another DISC frame, and finally sends an UA frame
        // Send DISC frame
        Frame discFrame = createControlFrame(A_FRAME_SENDER, C_DISC);
        if(write(fd, discFrame.data, discFrame.size) == -1) {
            printf("Error sending DISC packet.\n");
            return -1;
        } else {
            printf("DISC packet sent.\n");
        }
        // Wait for DISC frame
        if(checkControlFrame(fd, A_ANSWER_RECEIVER, C_DISC)) {
            printf("Received correct DISC packet from receiver.\n");
        } else {
            printf("Error receiving DISC packet from receiver.\n");
            return -1;
        }
        // Send UA frame
        Frame uaFrame = createControlFrame(A_ANSWER_SENDER, C_UA);
        if(write(fd, uaFrame.data, uaFrame.size) == -1) {
            printf("Error sending UA packet.\n");
            return -1;
        } else {
            printf("UA packet sent, terminating connection.\n");
            sleep(1); // wait for the receiver to receive the UA frame
        }
    } else if(cp.role == LlRx) {
        // Wait for DISC frame, send DISC frame, and finally waits for an UA frame
        if(checkControlFrame(fd, A_FRAME_SENDER, C_DISC)) {
            printf("Received correct DISC packet from sender.\n");
            // Send DISC frame
            Frame discFrame = createControlFrame(A_ANSWER_RECEIVER, C_DISC);
            if(write(fd, discFrame.data, discFrame.size) == -1) {
                printf("Error sending DISC packet.\n");
                return -1;
            } else {
                printf("DISC packet sent.\n");
            }
        } else {
            printf("Error receiving DISC packet from sender.\n");
            return -1;
        }
        // Wait for UA frame
        if(checkControlFrame(fd, A_ANSWER_SENDER, C_UA)) {
            printf("Received correct UA packet from sender, terminating connection.\n");
        } else {
            printf("Error receiving UA packet from sender.\n");
            return -1;
        }
    }
    return terminate_connection(fd);
}
