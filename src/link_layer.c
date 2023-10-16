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
#define C_SET 0X03
#define C_UA 0X07
#define C_RR0 0x05 // sent by the Receiver when it is ready to receive an information frame number 0
#define C_RR1 0x85 // sent by the Receiver when it is ready to receive an information frame number 1
#define C_REJ0 0x01 // sent by the Receiver when it rejects an information frame number 0 (detected an error)
#define C_REJ1 0x81 // sent by the Receiver when it rejects an information frame number 1 (detected an error)
#define C_DISC 0X0B
#define C_I0 0x00 // information frame number 0
#define C_I1 0x40 // information frame number 1

// Global variables
volatile int STOP;

unsigned int alarmCount = 0;
int alarmEnabled = FALSE;
int fd;

unsigned char buf[MAX_FRAME_SIZE] = {0};

LinkLayer cp;

enum CheckRecv {
    Start,
    Flag_Rcv,
    A_Rcv,
    C_Rcv,
    Bcc_OK,
    Data_Rcv,
};

// returns TRUE if the SET packet was received correctly
int checkSETFrame(int fd) {
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    while(1){
        read(fd, recv, 1);
        printf("Received: %x\n", recv[0]);
        switch(rc) {
            case Start: {
                if(recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                } 
                return FALSE;
            }
            case Flag_Rcv: {
                if(recv[0] == A_FRAME_SENDER) {
                    rc = A_Rcv;
                    ac[0] = recv[0];
                    break;
                } else if (recv[1] == FLAG) {
                    rc = Flag_Rcv;
                }
                return FALSE;
            }
            case A_Rcv: {
                if(recv[0] == C_SET) {
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
                if(recv[0] == FLAG) {
                    return TRUE;
                }
            }
        }
    }
    return FALSE;
}
// returns TRUE if the UA packet was received correctly
int check_UA_Response(int fd) {
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
                if(recv[0] == A_ANSWER_RECEIVER) {
                    rc = A_Rcv;
                    ac[0] = A_ANSWER_RECEIVER;
                    break;
                } else if (recv[1] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return FALSE;
            }
            case A_Rcv: {
                printf("A_Rcv\n");
                if(recv[0] == C_UA) {
                    rc = C_Rcv;
                    ac[1] = C_UA;
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

int send_SET_Frame(int fd, LinkLayer connectionParameters) {
    unsigned char SET_Packet[5];
    // open the connection by writing a SET control packet
    SET_Packet[0] = FLAG;
    SET_Packet[1] = A_FRAME_SENDER;
    SET_Packet[2] = C_SET;
    SET_Packet[3] = (A_FRAME_SENDER ^ C_SET);
    SET_Packet[4] = FLAG;

    // write packet to cable
    write(fd, SET_Packet, 5);
    // printf("SET packet sent.\n");

    return TRUE;
}

int send_UA_Frame(int fd, LinkLayer connectionParameters) {
    unsigned char UA_Packet[5];
    // open the connection by writing a SET control packet
    UA_Packet[0] = FLAG;
    UA_Packet[1] = A_ANSWER_RECEIVER;
    UA_Packet[2] = C_UA;
    UA_Packet[3] = (A_ANSWER_RECEIVER ^ C_UA);
    UA_Packet[4] = FLAG;

    // write packet to cable
    write(fd, UA_Packet, 5);
    // printf("UA packet sent.\n");

    return TRUE;
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
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    cp = connectionParameters;
    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror("Error opening tty");
        return -1;
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
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

    if(connectionParameters.role == LlTx) {
        if(send_SET_Frame(fd, connectionParameters) == -1) {
            printf("Error sending SET packet.\n");
            return -1;
        } else {
            printf("SET packet sent.\n");
        }
        // set alarm handler
        (void)signal(SIGALRM, alarmHandler);
        // set alarm
        setAlarm(connectionParameters.timeout);

        if(check_UA_Response(fd)) {
            printf("Received correct UA packet from receiver.\n");
            removeAlarm();
            return 1;
        }

        // send SET packet and wait for UA response nRetransmissions times
        while(alarmCount < connectionParameters.nRetransmissions) {
            if(!alarmEnabled) {
                printf("Alarm #%d: Resending the packet.\n", alarmCount);

                if(send_SET_Frame(fd, connectionParameters) == -1) {
                    printf("Error sending SET packet.\n");
                    return -1;
                } else {
                    printf("SET packet sent.\n");
                }

                if(check_UA_Response(fd)) {
                    printf("Received correct UA packet from receiver.\n");
                    removeAlarm();
                    return 1;
                }

                // Set the alarm again for the next attempt
                setAlarm(connectionParameters.timeout);
            }
        }
    } else if(connectionParameters.role == LlRx) {
        // set alarm handler
        (void)signal(SIGALRM, alarmHandler);
        // set alarm
        setAlarm(connectionParameters.timeout);

        // wait for SET packet and send UA response
        while(alarmCount < connectionParameters.nRetransmissions) {
            if(!alarmEnabled) {
                printf("Alarm #%d: Resending the packet.\n", alarmCount);
                // Set the alarm again for the next attempt
                setAlarm(connectionParameters.timeout);
            }
            if(checkSETFrame(fd)) {
                printf("Received correct SET packet from sender.\nSending UA packet.\n");
                removeAlarm();
                if(send_UA_Frame(fd, connectionParameters) == -1) {
                    printf("Error sending UA packet.\n");
                    return -1;
                } else {
                    printf("UA packet sent.\n");
                }
                return 1;
            }
        }
    }
    return -1;
}

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
                    printf("BCC2 OK\n");
                    return dataSize;
                } else {
                    return -1;
                }
            }
        }
    }
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
int llwrite(const unsigned char *buf, int bufSize)
{
    int numFrames = (bufSize + MAX_FRAME_SIZE - 1) / MAX_FRAME_SIZE; // Calculate the number of frames

    for (int i = 0; i < numFrames; i++) {
        // Calculate the size of this frame, which is the minimum between the remaining data size and the maximum frame size
        int frameSize = (bufSize >= MAX_FRAME_SIZE) ? MAX_FRAME_SIZE : bufSize;

        // Create the I-frame
        Frame frame = createInformationFrame(buf, frameSize, sequenceNumber, A_FRAME_SENDER);
        buf += frameSize;
        bufSize -= frameSize;

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
                sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sequence number
                break; // Move to the next frame
            }
        }
        if(alarmCount >= cp.nRetransmissions) {
            printf("Error sending frame.\n");
            return -1;
        }
    }

    return 1; // Successful transmission
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
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
    Frame rrFrame = createControlFrame(A_ANSWER_RECEIVER, C_RR0 | (sequenceNumber+1)%2 << 7);

    printf("Received data: %s", packet);
    // Send RR frame
    write(fd, rrFrame.data, rrFrame.size);

    return dataSize; // Return the size of the combined data
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
