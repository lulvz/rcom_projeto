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
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received

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

// returns TRUE if the ack packet was received correctly and is right
int checkACKResponse(int fd, int expectedSequenceNumber) {
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
                return FALSE;
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
                return FALSE;
            }
            case A_Rcv: {
                if (recv[0] == (C_RR0 | expectedSequenceNumber << 7) || recv[0] == (C_RR1 | expectedSequenceNumber << 7)) {
                    // Check if the received ACK/NACK corresponds to the expected sequence number.
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
                if (recv[0] == (ac[0] ^ ac[1])) {
                    rc = Bcc_OK;
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return FALSE;
            }
            case Bcc_OK: {
                if (recv[0] == FLAG) {
                    return TRUE;
                }
                return FALSE;
            }
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

    // Set alarm
    setAlarm(cp.timeout);

    // Wait for ACK
    while (alarmCount < cp.nRetransmissions) {
        if (!alarmEnabled) {
            // Handle retransmission
            printf("Alarm #%d: Resending the packet.\n", alarmCount);
            write(fd, frame.data, frame.size);

            // Set the alarm again for the next attempt
            setAlarm(cp.timeout);
        }

        // Check for ACK response
        if (checkACKResponse(fd, sequenceNumber)) {
            // ACK received
            removeAlarm();
            lastAckReceived = sequenceNumber;
            sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sequence number
            break; // Move to the next frame
        }
    }
    if(alarmCount >= cp.nRetransmissions) {
        printf("Error sending frame.\n");
        return -1;
    }

    return 1; // Successful transmission
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) // packet has 1000bytes size
{    
    // packet = malloc(MAX_FRAME_SIZE);

    // the expected sequence number is the sequence number we have currenlty
    int dataSize = decodeInformationFrame(fd, sequenceNumber, packet);

    if (dataSize == -1) {
        // Error decoding frame
        printf("Error decoding frame.\n");
        return -1;
    }

    // Create the RR frame, the sequence number we have to send is the one that the i-frame is going to have
    Frame rrFrame = createControlFrame(A_ANSWER_RECEIVER, C_RR0 | ((sequenceNumber+1)%2) << 7);

    printf("Received data: %s", packet);
    // Send RR frame
    if(write(fd, rrFrame.data, rrFrame.size) == -1) {
        printf("Error sending RR packet.\n");
        return -1;
    } else {
        printf("RR packet sent.\n");
        sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sequence number
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
