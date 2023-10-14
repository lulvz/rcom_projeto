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

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define BUF_SIZE 256

#define FLAG 0x7E
#define A_FRAME_SENDER 0x03
#define A_ANSWER_RECEIVER 0x03
#define A_FRAME_RECEIVER 0X01
#define A_ANSWER_SENDER 0X01
#define C_SET 0X03
#define C_UA 0X07

// Global variables
volatile int STOP;

unsigned int alarmCount = 0;
int alarmEnabled = FALSE;
int fd;

unsigned char buf[BUF_SIZE] = {0};

enum CheckRecv {
    Start,
    Flag_Rcv,
    A_Rcv,
    C_Rcv,
    Bcc_OK,
};

int check_UA_Response(int fd) {
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    while(1){
        read(fd, recv, 1);
        switch(rc) {
            case Start: {
                if(recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                } 
                return FALSE;
            }
            case Flag_Rcv: {
                if(recv[0] == A_ANSWER_RECEIVER) {
                    rc = A_Rcv;
                    ac[0] = recv[0];
                    break;
                } else if (recv[1] == FLAG) {
                    rc = Flag_Rcv;
                }
                return FALSE;
            }
            case A_Rcv: {
                if(recv[0] == C_UA) {
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

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d: Resending the packet.\n", alarmCount);

    unsigned char buf[BUF_SIZE] = {0};
    buf[0] = 0x7E;
    buf[1] = 0x03;
    buf[2] = 0x03;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = 0x7E;

    int bytes = write(fd, buf, BUF_SIZE);
    printf("%d bytes written\n", bytes);

        // Set the alarm again for the next attempt
    alarm(3);
    alarmEnabled = TRUE;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
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

    unsigned char SET_Packet[BUF_SIZE];
    // open the connection by writing a SET control packet
    SET_Packet[0] = FLAG;
    SET_Packet[1] = A_FRAME_SENDER;
    SET_Packet[2] = C_SET;
    SET_Packet[3] = (buf[1] ^ buf[2]);
    SET_Packet[4] = FLAG;

    (void)signal(SIGALRM, alarmHandler);
    alarm(3); // Set the initial alarm for 3 seconds
    alarmEnabled = TRUE;

    // write packet to cable
    write(fd, SET_Packet, BUF_SIZE);

    // try to send SET packets every 3 seconds until timeout reached
    while(alarmCount < connectionParameters.timeout) {
        if(check_UA_Response(fd)) {
            printf("Received correct UA response from receiver.\nOpening connection.\n");
            alarmEnabled = FALSE;
            alarm(0);
            alarmCount = 0;
            return 0;
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    return 1;
}
