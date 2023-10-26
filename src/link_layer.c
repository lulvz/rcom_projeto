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
#include <stdlib.h>
// include usleep
#include <unistd.h>

// Global variables
volatile int STOP = FALSE;
unsigned int alarmCount = 0;
int alarmEnabled = FALSE;
int fd;
LinkLayer cp = {0};
struct termios oldtio;
struct termios newtio;

static volatile int sequenceNumber = 0;  // Initial sequence number
static volatile int lastAckReceived = -1; // Last acknowledged sequence number


void alarmHandler() {
    alarmCount++;
    alarmEnabled = FALSE;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    cp.baudRate = connectionParameters.baudRate;
    cp.timeout = connectionParameters.timeout;
    cp.nRetransmissions = connectionParameters.nRetransmissions;
    cp.role = connectionParameters.role;
    strcpy(cp.serialPort, connectionParameters.serialPort);
    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(cp.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror("Error opening tty");
        return -1;
    }

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
    newtio.c_cc[VMIN] = 0;  // KEEP AT 0 OMFG IM GOING CRAZY

    // Now clean the line and activate the settings for the port
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        close(fd);
        return -1; // Return an error code
    }

    if(cp.role == LlTx) {
        (void)signal(SIGALRM, alarmHandler); // set alarm only for sender
        enum CheckRecv rc = Start;
        unsigned char recv[1] = {0};

        while(alarmCount < cp.nRetransmissions) {
            // build and send packet 
            Frame setPacket = createControlFrame(A_FRAME_SENDER, C_SET);
            if(write(fd, setPacket.data, setPacket.size) == -1) {
                printf("Error writing SET packet.\n");
                return -1;
            } else {
                printf("SET packet sent.\n");
            }
            alarm(cp.timeout);
            alarmEnabled = TRUE;

            while(alarmEnabled) {
                
                ssize_t recvSize = read(fd, recv, 1);
                if(recvSize > 0) {
                    switch(rc) {
                        default: break;
                        case Start: {
                            if(recv[0] == FLAG) {
                                rc = Flag_Rcv;
                            }
                            break;
                        }
                        case Flag_Rcv: {
                            if(recv[0] == A_ANSWER_SENDER) {
                                rc = A_Rcv;
                            } else if (recv[0] == FLAG) {
                                rc = Flag_Rcv;
                            } else {
                                rc = Start;
                            }
                            break;
                        }
                        case A_Rcv: {
                            if(recv[0] == C_UA) {
                                rc = C_Rcv;
                            } else if (recv[0] == FLAG) {
                                rc = Flag_Rcv;
                            } else {
                                rc = Start;
                            }
                            break;
                        }
                        case C_Rcv: {
                            if(recv[0] == (A_ANSWER_SENDER ^ C_UA)) {
                                rc = Bcc_OK;
                            } else if (recv[0] == FLAG) {
                                rc = Flag_Rcv;
                            } else {
                                rc = Start;
                            }
                            break;
                        }
                        case Bcc_OK: {
                            if(recv[0] == FLAG) {
                                printf("UA packet received, connection established.\n");
                                alarm(0);
                                alarmEnabled = FALSE;
                                return 1;
                            } else {
                                rc = Start;
                            }
                            break;
                        }
                    }
                }
            }
        }
    } else if(cp.role == LlRx) {
        enum CheckRecv rc = Start;
        unsigned char recv[1] = {0};
        while(1) {
            ssize_t recvSize = read(fd, recv, 1);
            if(recvSize > 0) {
                switch(rc) {
                    default: break;
                    case Start: {
                        if(recv[0] == FLAG) {
                            rc = Flag_Rcv;
                        }
                        break;
                    }
                    case Flag_Rcv: {
                        if(recv[0] == A_FRAME_SENDER) {
                            rc = A_Rcv;
                        } else if (recv[0] == FLAG) {
                            rc = Flag_Rcv;
                        } else {
                            rc = Start;
                        }
                        break;
                    }
                    case A_Rcv: {
                        if(recv[0] == C_SET) {
                            rc = C_Rcv;
                        } else if (recv[0] == FLAG) {
                            rc = Flag_Rcv;
                        } else {
                            rc = Start;
                        }
                        break;
                    }
                    case C_Rcv: {
                        if(recv[0] == (A_FRAME_SENDER ^ C_SET)) {
                            rc = Bcc_OK;
                        } else if (recv[0] == FLAG) {
                            rc = Flag_Rcv;
                        } else {
                            rc = Start;
                        }
                        break;
                    }
                    case Bcc_OK: {
                        if(recv[0] == FLAG) {
                            printf("SET packet received, sending UA packet.\n");
                            Frame uaPacket = createControlFrame(A_ANSWER_SENDER, C_UA);
                            if(write(fd, uaPacket.data, uaPacket.size) == -1) {
                                printf("Error writing UA packet.\n");
                                return -1;
                            } else {
                                printf("UA packet sent, connection established.\n");
                                return 1;
                            }
                        } else {
                            rc = Start;
                        }
                        break;
                    }
                }
            }
        }
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

// Decoding the Frames
// Returns length of the data if everything is correct
// Returns -1 if the frame was corrupted
// Returns -2 if the frame is duplicated
int decodeInformationFrame(int fd, int expectedSequenceNumber, unsigned char *data) {
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    int dataSize = 0;
    int stuffedDataSize = 0;
    unsigned char stuffedData[MAX_STUFFED_DATA_SIZE];
    unsigned char tmpData[MAX_PAYLOAD_SIZE+1];
    

    while(1) {
        switch(rc) {
            case Start: {
                read(fd, recv, 1);
                if(recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case Flag_Rcv: { 
                read(fd, recv, 1);
                if(recv[0] == A_FRAME_SENDER) {
                    rc = A_Rcv;
                    ac[0] = A_FRAME_SENDER;
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case A_Rcv: {
                read(fd, recv, 1);
                // check c (0x00 | (expectedSequenceNumber << 6))
		// Simply get the control and pass to next state
                if(recv[0] == FLAG) {
		    rc = Flag_Rcv;
                    break;
                } else {
                    rc = C_Rcv;
                    ac[1] = recv[0];
                    break;
                }
            }
            case C_Rcv: {
                read(fd, recv, 1);
                if(recv[0] == (ac[0] ^ ac[1])) {
                    rc = Bcc_OK;
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case Bcc_OK: {
                // fill data array with data and bcc2
                read(fd, recv, 1);
                if(recv[0] == FLAG) {
                    rc = Data_Rcv;
                    break;
                } else if(stuffedDataSize < MAX_STUFFED_DATA_SIZE) {
                    stuffedData[stuffedDataSize] = recv[0];
                    stuffedDataSize++;
                    break;
                }
                return -1;
            }
            case Data_Rcv: {
                // if flafg was found on BCC_OK, the byte before the flag is bcc2 which is xor of all the databytes
                dataSize = destuffIt(stuffedData, stuffedDataSize, tmpData);

                // calculate xor of data bytes (except bcc2)
                unsigned char dataXor = 0;
                for(int i = 0; i < (dataSize-1); i++) {
                    dataXor ^= tmpData[i];
                }
		
		int sqnIsOk = (ac[1] == (0x00|(expectedSequenceNumber << 6)));

		// Case that everythin is ok, so return the length
		if(dataXor == tmpData[dataSize-1] && sqnIsOk){
                    memcpy(data, tmpData, dataSize-1); // copy all except bcc2
		    return dataSize - 1;
		}
		else if(dataXor == tmpData[dataSize-1]) {
		    /*If everything is okay, except the sequnce number 
		     probably the packet is duplicate, so just send the 
		     RR again*/
                    return -2;
                } else {
                    return -1;
                }
            }
        }
    }
}

// returns TRUE if the defined control frame was received correctly
int checkControlFrame(int fd, unsigned char addressField, unsigned char controlField) {
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    while(1){
        read(fd, recv, 1);
        printf("Received: %x\n", recv[0]);
        switch(rc) {
            case Start: {
                printf("Start\n");
                if(recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return FALSE;
            }
            case Flag_Rcv: {
                printf("Flag_Rcv\n");
                if(recv[0] == addressField) {
                    rc = A_Rcv;
                    ac[0] = addressField;
                    break;
                } else if (recv[1] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return FALSE;
            }
            case A_Rcv: {
                printf("A_Rcv\n");
                if(recv[0] == controlField) {
                    rc = C_Rcv;
                    ac[1] = recv[0];
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return FALSE;
            }
            case C_Rcv: {
                printf("C_Rcv\n");
                if(recv[0] == (ac[0] ^ ac[1])) {
                    rc = Bcc_OK;
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return FALSE;
            }
            case Bcc_OK: {
                printf("Bcc_OK\n");
                if(recv[0] == FLAG) {
                    return TRUE;
                }
                return FALSE;
            }
	    // Just added to stop the warnings, it not possible to reach it
	    case Data_Rcv :
			return FALSE; 
        }
    }
    return FALSE;
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



    // Check for ACK response
    if (checkACKResponse(fd, (sequenceNumber+1) %2) == 1) {
        // ACK received
        alarm(0);
        alarmEnabled = FALSE;
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
                 alarm(0);
                 alarmEnabled = FALSE;
                 lastAckReceived = sequenceNumber;
                 sequenceNumber = (sequenceNumber + 1) % 2;// Toggle sqcN
                 return 1;
             }
         }
	 else {
		alarm(cp.timeout);
        alarmEnabled = TRUE;
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
    if(tcsetattr(fd, TCSANOW, &oldtio) == -1) {
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
