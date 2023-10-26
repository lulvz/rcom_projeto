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
void setAlarm(int seconds) {
    alarm(3); // Set the initial alarm for 3 seconds
    alarmEnabled = TRUE;
}

void removeAlarm() {
    alarmEnabled = FALSE;
    alarm(0);
    alarmCount = 0;
}

void alarmHandler(int signal) {
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
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

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
        if(write(fd, setFrame.data, setFrame.size) == -1) {
            printf("Error sending SET packet.\n");
            return -1;
        } else {
            printf("SET packet sent.\n");
        }

        // set alarm handler
        (void)signal(SIGALRM, alarmHandler);
        // set alarm
        setAlarm(cp.timeout);

        if(checkControlFrame(fd, A_ANSWER_RECEIVER, C_UA)) {
            printf("Received correct UA packet from receiver.\n");
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
                    printf("Error sending SET packet.\n");
                    return -1;
                } else {
                    printf("SET packet sent.\n");
                }

                if(checkControlFrame(fd, A_ANSWER_RECEIVER, C_UA)) {
                    printf("Received correct UA packet from receiver.\n");
                    removeAlarm();
                    return 1;
                }

                // Set the alarm again for the next attempt
                setAlarm(cp.timeout);
            }
        }
    } else if(cp.role == LlRx) {
        while(alarmCount < cp.nRetransmissions) {
            if(!alarmEnabled) {
                if(checkControlFrame(fd, A_FRAME_SENDER, C_SET)) {
                    printf("Received correct SET packet from sender.\n");
                    removeAlarm();

                    // Send UA packet
                    Frame uaFrame = createControlFrame(A_ANSWER_RECEIVER, C_UA);
                    // Frame uaFrame = createControlFrame(0x00, C_UA); // just to test
                    if(write(fd, uaFrame.data, uaFrame.size) == -1) {
                        printf("Error sending UA packet.\n");
                        return -1;
                    } else {
                        printf("UA packet sent.\n");
                        return 1;
                    }
                }
                
                // Set the alarm again for the next attempt
                setAlarm(cp.timeout);
            }
        }
    }
    return -1;
}

// returns TRUE if it's a rr frame or FALSE if it's a rej frame and -1 if it's an error
int checkACKResponse(int fd, int expectedSequenceNumber, LinkLayer cp) {
	printf("In the checkACKResponse function\n");
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    int return_value = -1;
    while (1) {

        int bytes_read = read(fd, recv, 1);
        if(bytes_read == 0) {
            continue;
        }

        printf("Received: %x\n", recv[0]);
        switch (rc) {
            default:
                {return FALSE;}
            case Start: {
                if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                break; 
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
                    // Check if the received ACK corresponds to the expected sequence number.
                    rc = C_Rcv;
                    ac[1] = recv[0];
                    break;
                } else if (recv[0] == (0x01 | ((expectedSequenceNumber+1)%2) << 7)) {
                    // Check if the received NACK corresponds to the next sequence number.
                    rc = C_Rcv;
                    ac[1] = recv[0];
                    break;
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
                    return_value = TRUE;
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    return_value = FALSE;
                    break;
                }
                return -1;
            }
            case Bcc_OK: {
		printf("In the BCCok\n");
                if (recv[0] == FLAG) {
                    return return_value;
                }
                return -1;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) // TODO: MAKE THIS SEND ONLY ONE PACKET FROM THE APPLICATION WITH THE MAXIMUM SIZE OF 1000 BYTES
{
	printf("Inside the llwrite function\n");
    if(bufSize > MAX_PAYLOAD_SIZE) {
        printf("Error: bufSize is bigger than the maximum payload size.\n");
        return -1;
    }
    // Create the I-frame
    Frame frame = createInformationFrame(buf, bufSize, sequenceNumber, A_FRAME_SENDER);

    printf("Created information frame\n");
	for(unsigned i = 0; i < frame.size; i++){
        printf("%02X ", frame.data[i]); // Print each byte in hexadecimal format
	}
    
    // Send frame
    write(fd, frame.data, frame.size);

    // Set alarm
    setAlarm(cp.timeout);

    // Check for ACK response
    int ack_check = checkACKResponse(fd, (sequenceNumber+1) %2, cp);
    if (ack_check == TRUE) {
        // ACK received
        removeAlarm();
        lastAckReceived = sequenceNumber;
        sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sequence number
        return 1;
    } else if (ack_check == FALSE) {
        // NACK received
        removeAlarm();
        // we don't toggle the sequence number since the packet was rejected
        printf("NACK received, packet rejected.\n");
        return -1;
    } else {
        removeAlarm();
        printf("Error in the ACK/NACK frame.\n");
        return -1;
    }

    // // Wait for ACK
    // while (alarmCount < cp.nRetransmissions) {
    //     if (!alarmEnabled) {
    //         // Handle retransmission
    //         printf("Alarm #%d: Resending the packet.\n", alarmCount);
    //         write(fd, frame.data, frame.size);

    //         // Set the alarm again for the next attempt
    //         setAlarm(cp.timeout);

    //         // Check for ACK response
    //         if (checkACKResponse(fd, sequenceNumber)) {
    //             // ACK received
    //             removeAlarm();
    //             lastAckReceived = sequenceNumber;
    //             sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sequence number
    //             break; // Move to the next frame
    //         }
    //     }
    // }
    // if(alarmCount >= cp.nRetransmissions) {
    //     printf("Error sending frame.\n");
    //     return -1;
    // }

    return 1; // Successful transmission
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) // packet has 1000bytes size
{
    
    // the expected sequence number is the sequence number we have currenlty
    int dataSize = decodeInformationFrame(fd, sequenceNumber, packet);

    if (dataSize == -1) {
        // Error decoding frame
        printf("Error decoding frame.\n");

        // Send rej packet  
        Frame rejFrame = createControlFrame(A_ANSWER_RECEIVER, C_REJ0 | sequenceNumber << 7); // we reject an information frame with sequence number 
        
        if(write(fd, rejFrame.data, rejFrame.size) == -1) {
            printf("Error sending Rej packet.\n");
            return -1;
        } else {
            printf("Rej packet sent.\n");// we don't toggle the sequence number
        }
        return -1;
    }

    sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sequence number

    // Create the RR frame, the sequence number we have to send is the one that the i-frame is going to have
    Frame rrFrame = createControlFrame(A_ANSWER_RECEIVER, C_RR0 | (sequenceNumber) << 7);

    printf("Decoded packet\n");
    for(unsigned i = 0; i < dataSize; i++){
    	printf("%02X ", packet[i]); // Print each byte in hexadecimal format
    }

    // Send RR frame
    if(write(fd, rrFrame.data, rrFrame.size) == -1) {
        printf("Error sending RR packet.\n");
        return -1;
    } else {
        printf("RR packet sent.\n");
    }

    return dataSize; // Return the size of the combined data
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
            terminate_connection(fd);
        } else {
            printf("DISC packet sent.\n");
        }
        // Wait for DISC frame
        if(checkControlFrame(fd, A_ANSWER_RECEIVER, C_DISC)) {
            printf("Received correct DISC packet from receiver.\n");
        } else {
            printf("Error receiving DISC packet from receiver.\n");
           terminate_connection(fd); 
        }
        // Send UA frame
        Frame uaFrame = createControlFrame(A_ANSWER_SENDER, C_UA);
        if(write(fd, uaFrame.data, uaFrame.size) == -1) {
            printf("Error sending UA packet.\n");
           terminate_connection(fd); 
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
               terminate_connection(fd); 
            } else {
                printf("DISC packet sent.\n");
            }
        } else {
            printf("Error receiving DISC packet from sender.\n");
           terminate_connection(fd); 
        }
        // Wait for UA frame
        if(checkControlFrame(fd, A_ANSWER_SENDER, C_UA)) {
            printf("Received correct UA packet from sender, terminating connection.\n");
        } else {
            printf("Error receiving UA packet from sender.\n");
           terminate_connection(fd); 
        }
    }
    return terminate_connection(fd);
}

// returns the size of the new stuffed data or -1 on error, should take a pointer to an array with size MAX_STUFFED_DATA_SIZE
int stuffIt(const unsigned char *buf, int bufSize, unsigned char stuffedData[MAX_STUFFED_DATA_SIZE]) {
    int stuffedDataIdx = 0;

    // stuffs all the data, including the bcc2 at the end of the payload
    for(int i = 0; i<bufSize; i++) {
        if(buf[i] == FLAG) {
            stuffedData[stuffedDataIdx] = 0x7d;
            stuffedData[stuffedDataIdx + 1] = 0x5e;
            // 2 byte new size
            stuffedDataIdx += 2;
        } else if (buf[i] == 0x7d) {
            stuffedData[stuffedDataIdx] = 0x7d;
            stuffedData[stuffedDataIdx + 1] = 0x5d;
            //2 byte new size
            stuffedDataIdx += 2;
        } else {
            stuffedData[stuffedDataIdx] = buf[i];
            stuffedDataIdx++;
        }
    }
    // printf("stuffedDataIdx: %i\n", stuffedDataIdx);
    return stuffedDataIdx;
}

// returns the size of the de-stuffed data, this should be a pointer to an array with size MAX_PAYLOAD_SIZE
int destuffIt(unsigned char* buf, int bufSize, unsigned char deStuffedData[MAX_PAYLOAD_SIZE+1]) {
    int deStuffedDataIdx = 0;

    for(int i = 0; i<bufSize; i++) {
        if(buf[i] == 0x7d) {
            if(buf[i+1] == 0x5e) {
                deStuffedData[deStuffedDataIdx] = FLAG;
                i++; // advance to next byte
            } else if(buf[i+1] == 0x5d) {
                deStuffedData[deStuffedDataIdx] = 0x7d;
                i++; // advance to next byte
            }
        } else {
            deStuffedData[deStuffedDataIdx] = buf[i];
        }
        deStuffedDataIdx++;
    }

    return deStuffedDataIdx;
}

// Creation of Frames
Frame createInformationFrame(const unsigned char *data, int dataSize, int sequenceNumber, unsigned char addressField) {
	printf("Inside the createInformationFrame function\n");
    printf("sequenceNumber in createinformation: %d\n", sequenceNumber);
    Frame frame = {0};

    frame.data[0] = FLAG;
    frame.data[1] = addressField;
    frame.data[2] = (sequenceNumber << 6) | 0x00; // 0x00 = I-frame number 0 | 0x40 = I-frame number 1
    printf("frame.data[2] is %x\n", frame.data[2]);
    frame.data[3] = frame.data[1] ^ frame.data[2]; // BCC1

    unsigned char bcc2 = 0;
    // bcc2 is xor of all the data bytes
    for (int i = 0; i < dataSize; i++) {
        bcc2 ^= data[i];
    }

    // add bcc2 to the end of the data to be stuffed
    unsigned char dataWithBcc2[MAX_PAYLOAD_SIZE+1];
    memcpy(dataWithBcc2, data, dataSize);
    dataWithBcc2[dataSize] = bcc2;
    // printf("bcc2 in dataWithBcc2 is %x\nbcc2 in bcc2 is %x\n", dataWithBcc2[dataSize], bcc2);

    unsigned char stuffedData[MAX_STUFFED_DATA_SIZE];
    int stuffedDataSize = stuffIt(dataWithBcc2, dataSize+1, stuffedData);

    memcpy(frame.data + 4, stuffedData, stuffedDataSize);

    // printf("StuffedDataSize is %i\n", stuffedDataSize);
    frame.data[4 + stuffedDataSize] = FLAG;
    frame.size = 4 + stuffedDataSize + 1;
    // print contents of data
    // printf("Frame contents:\n");
    // for(int i = 0; i < frame.size; i++) {
    //     printf("%x ", frame.data[i]);
    // }

    return frame;
}

Frame createControlFrame(const unsigned char addressField, unsigned char controlField) {
    Frame frame = {0};
    frame.size = 5;

    frame.data[0] = FLAG;
    frame.data[1] = addressField;
    frame.data[2] = controlField;
    frame.data[3] = frame.data[1] ^ frame.data[2]; // BCC1
    frame.data[4] = FLAG;

    return frame;
}

// Decoding the Frames
int decodeInformationFrame(int fd, int expectedSequenceNumber, unsigned char *data) {
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    int dataSize = 0;
    int stuffedDataSize = 0;
    unsigned char stuffedData[MAX_STUFFED_DATA_SIZE];
    unsigned char tmpData[MAX_PAYLOAD_SIZE+1];

    printf("State machine of decodeInformationFrame function\n");
    printf("sequenceNumber in decodeinformation: %d\n", expectedSequenceNumber);

    while(1) {
        int read_size = read(fd, recv, 1);
        
        if(read_size < 0) {
            return FALSE;
        }

        switch(rc) {
            case Start: {
                if(read_size == 0) {
                    continue;
                }
                if(read_size < 0) {
                    return FALSE;
                }
                // read(fd, recv, 1);
                // printf("Received: %x\n", recv[0]);
                if(recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                break;
            }
            case Flag_Rcv: { 
                if(read_size == 0) {
                    continue;
                }
                printf("flag_rcf\n");
                if(read_size < 0) {
                    return FALSE;
                }
                // read(fd, recv, 1);
                // printf("Received: %x\n", recv[0]);
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
                if(read_size == 0) {
                    continue;
                }
                // read(fd, recv, 1);
                // printf("Received: %x\n", recv[0]);
                // check c
                if(recv[0] == (0x00 | (expectedSequenceNumber << 6))) {
                    rc = C_Rcv;
                    ac[1] = recv[0];
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                printf("Wrong sequence number\n");
                return -1;
            }
            case C_Rcv: {
                if(read_size == 0) {
                    continue;
                }
                // read(fd, recv, 1);
                // printf("Received: %x\n", recv[0]);
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
                // read(fd, recv, 1);
                // printf("Received: %x\n", recv[0]);
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
                // if flag was found on BCC_OK, the byte before the flag is bcc2 which is xor of all the databytes
                dataSize = destuffIt(stuffedData, stuffedDataSize, tmpData);

                // calculate xor of data bytes (except bcc2)
                unsigned char dataXor = 0;
                for(int i = 0; i < (dataSize-1); i++) {
                    dataXor ^= tmpData[i];
                }

                if(dataXor == tmpData[dataSize-1]) {
                    printf("BCC2 OK\n");
                    memcpy(data, tmpData, dataSize-1); // copy all except bcc2
                    return dataSize-1;
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
        int read_size = read(fd, recv, 1);
        // printf("read_size: %d\n", read_size);
        if(read_size == 0) {
            continue;
        }
        // printf("Received: %x\n", recv[0]);
        switch(rc) {
            default:
                {return FALSE;}
            case Start: {
                printf("Start\n");
                if(recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                break;
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
        }
    }
    return FALSE;
}
