// Link layer protocol implementation

#include "link_layer.h"
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

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// Maximum Frame Size = Maximum Payload Size + Overhead (header + trailer)
#define MAX_FRAME_SIZE (MAX_PAYLOAD_SIZE + 4 + 2)

#define FLAG 0x7E

#define A_FRAME_SENDER 0x03
#define A_ANSWER_RECEIVER 0x03
#define A_FRAME_RECEIVER 0X01
#define A_ANSWER_SENDER 0X01

// control frame control fields
#define C_SET 0X03
#define C_UA 0X07
#define C_RR0 0x05 // sent by the Receiver when it is ready to receive an information frame number 0
#define C_RR1 0x85 // sent by the Receiver when it is ready to receive an information frame number 1
#define C_REJ0 0x01 // sent by the Receiver when it rejects an information frame number 0 (detected an error)
#define C_REJ1 0x81 // sent by the Receiver when it rejects an information frame number 1 (detected an error)
#define C_DISC 0X0B

// information frame control fields
#define C_I0 0x00 // information frame number 0
#define C_I1 0x40 // information frame number 1

// Global variables
volatile int STOP;

unsigned int alarmCount = 0;
int alarmEnabled = FALSE;
int fd;

unsigned char buf[MAX_FRAME_SIZE] = {0};

LinkLayer cp;
struct termios oldtioSave; // Save current port settings

enum CheckRecv {
    Start,
    Flag_Rcv,
    A_Rcv,
    C_Rcv,
    Bcc_OK,
    Data_Rcv,
};

enum FrameType {
    I,
    C,
};

typedef struct {
    unsigned char data[MAX_FRAME_SIZE];
    int size;
} Frame;

static volatile int sequenceNumber = 0;  // Initial sequence number
static volatile int lastAckReceived = -1; // Last acknowledged sequence number

Frame createInformationFrame(const unsigned char *data, int dataSize, int sequenceNumber, unsigned char addressField) {
    Frame frame = {0};
    frame.size = dataSize + 4 + 2;

    frame.data[0] = FLAG;
    frame.data[1] = addressField;
    frame.data[2] = (sequenceNumber << 6) | 0x00; // 0x00 = I-frame number 0 | 0x40 = I-frame number 1
    frame.data[3] = frame.data[1] ^ frame.data[2]; // BCC1

    memcpy(&frame.data[4], data, dataSize);
    
    unsigned char bcc2 = 0;
    // bcc2 is xor of all the data bytes
    for (int i = 0; i < dataSize; i++) {
        bcc2 ^= data[i];
    }

    frame.data[4 + dataSize] = bcc2;
    frame.data[4 + dataSize + 1] = FLAG;

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

// returns -1 on error or the size of the data field of the frame
int decodeInformationFrame(int fd, int expectedSequenceNumber, unsigned char *data) {
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    unsigned char lastByte = 0;
    unsigned char dataXor = 0;
    int dataSize = 0;
    while(1) {
        switch(rc) {
            case Start: {
                read(fd, recv, 1);
                printf("Received: %x\n", recv[0]);
                if(recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case Flag_Rcv: { 
                read(fd, recv, 1);
                printf("Received: %x\n", recv[0]);
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
                printf("Received: %x\n", recv[0]);
                // check c
                if(recv[0] == (C_I0 | expectedSequenceNumber << 6) || recv[0] == (C_I1 | expectedSequenceNumber << 6)) {
                    // Check if the received ACK/NACK corresponds to the expected sequence number.
                    rc = C_Rcv;
                    ac[1] = recv[0];
                    break;
                } else if (recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case C_Rcv: {
                read(fd, recv, 1);
                printf("Received: %x\n", recv[0]);
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
                // fill data array with data and bcc2 dont xor
                read(fd, recv, 1);
                printf("Received: %x\n", recv[0]);
                if(recv[0] == FLAG) {
                    dataSize--; // remove bcc2 from data size
                    rc = Data_Rcv;
                    break;
                } else {
                    data[dataSize] = recv[0];
                    dataSize++;
                    break;
                }
                return -1;
            }
            case Data_Rcv: {
                // if flag was found on BCC_OK, the byte before the flag is bcc2 which is xor of all the databytes
                // calculate xor of data bytes (data array minus byte after dataSize)
                for(int i = 0; i < dataSize; i++) {
                    dataXor ^= data[i];
                }
                if(dataXor == data[dataSize]) {
                    data[dataSize] = '\0'; // remove bcc2 from data array
                    printf("BCC2 OK\n");
                    return dataSize;
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
        }
    }
    return FALSE;
}

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
