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

static volatile int sequenceNumber = 0;   // Initial sequence number
static volatile int lastAckReceived = -1; // Last acknowledged sequence number

void alarmHandler()
{
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

    if (fd < 0)
    {
        perror("Error opening tty");
        return -1;
    }

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
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
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        close(fd);
        return -1; // Return an error code
    }

    if (cp.role == LlTx)
    {
        enum CheckRecv rc = Start;
        unsigned char recv[1] = {0};

        while (alarmCount < cp.nRetransmissions)
        {
            // build and send packet
            Frame setPacket = createControlFrame(A_FRAME_SENDER, C_SET);
            if (write(fd, setPacket.data, setPacket.size) == -1)
            {
                printf("Error writing SET packet.\n");
                return -1;
            }
            else
            {
                printf("SET packet sent.\n");
            }
            (void)signal(SIGALRM, alarmHandler); // set alarm only for sender
            alarm(cp.timeout);
            alarmEnabled = TRUE;

            while (alarmEnabled)
            {

                ssize_t recvSize = read(fd, recv, 1);
                if (recvSize > 0)
                {
                    switch (rc)
                    {
                    default:
                        break;
                    case Start:
                    {
                        if (recv[0] == FLAG)
                        {
                            rc = Flag_Rcv;
                        }
                        break;
                    }
                    case Flag_Rcv:
                    {
                        if (recv[0] == A_ANSWER_SENDER)
                        {
                            rc = A_Rcv;
                        }
                        else if (recv[0] == FLAG)
                        {
                            rc = Flag_Rcv;
                        }
                        else
                        {
                            rc = Start;
                        }
                        break;
                    }
                    case A_Rcv:
                    {
                        if (recv[0] == C_UA)
                        {
                            rc = C_Rcv;
                        }
                        else if (recv[0] == FLAG)
                        {
                            rc = Flag_Rcv;
                        }
                        else
                        {
                            rc = Start;
                        }
                        break;
                    }
                    case C_Rcv:
                    {
                        if (recv[0] == (A_ANSWER_SENDER ^ C_UA))
                        {
                            rc = Bcc_OK;
                        }
                        else if (recv[0] == FLAG)
                        {
                            rc = Flag_Rcv;
                        }
                        else
                        {
                            rc = Start;
                        }
                        break;
                    }
                    case Bcc_OK:
                    {
                        if (recv[0] == FLAG)
                        {
                            printf("UA packet received, connection established.\n");
                            alarm(0);
                            alarmEnabled = FALSE;
                            return 1;
                        }
                        else
                        {
                            rc = Start;
                        }
                        break;
                    }
                    }
                }
            }
        }
    }
    else if (cp.role == LlRx)
    {
        enum CheckRecv rc = Start;
        unsigned char recv[1] = {0};
        while (1)
        {
            ssize_t recvSize = read(fd, recv, 1);
            if (recvSize > 0)
            {
                switch (rc)
                {
                default:
                    break;
                case Start:
                {
                    if (recv[0] == FLAG)
                    {
                        rc = Flag_Rcv;
                    }
                    break;
                }
                case Flag_Rcv:
                {
                    if (recv[0] == A_FRAME_SENDER)
                    {
                        rc = A_Rcv;
                    }
                    else if (recv[0] == FLAG)
                    {
                        rc = Flag_Rcv;
                    }
                    else
                    {
                        rc = Start;
                    }
                    break;
                }
                case A_Rcv:
                {
                    if (recv[0] == C_SET)
                    {
                        rc = C_Rcv;
                    }
                    else if (recv[0] == FLAG)
                    {
                        rc = Flag_Rcv;
                    }
                    else
                    {
                        rc = Start;
                    }
                    break;
                }
                case C_Rcv:
                {
                    if (recv[0] == (A_FRAME_SENDER ^ C_SET))
                    {
                        rc = Bcc_OK;
                    }
                    else if (recv[0] == FLAG)
                    {
                        rc = Flag_Rcv;
                    }
                    else
                    {
                        rc = Start;
                    }
                    break;
                }
                case Bcc_OK:
                {
                    if (recv[0] == FLAG)
                    {
                        printf("SET packet received, sending UA packet.\n");
                        Frame uaPacket = createControlFrame(A_ANSWER_SENDER, C_UA);
                        if (write(fd, uaPacket.data, uaPacket.size) == -1)
                        {
                            printf("Error writing UA packet.\n");
                            return -1;
                        }
                        else
                        {
                            printf("UA packet sent, connection established.\n");
                            return 1;
                        }
                    }
                    else
                    {
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

// Returns 1 (TRUE) if the ACK was sent correctly
// Returns -1 if the ACK was corrupted
// Returns 0 (FALSE) received a REJ
int checkACKResponse(int fd, int expectedSequenceNumber)
{
    printf("In the checkACKResponse function\n");
    enum CheckRecv rc = Start;
    int ret = -1;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};

    while (alarmEnabled && (rc != Stop))
    {
        read(fd, recv, 1);
        if (recv[0] > 0)
        {
            printf("Received byte: %x\n", recv[0]);
            switch (rc)
            {
            default:
                break;
            case Start:
            {
                if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                    break;
                }
                break;
            }
            case Flag_Rcv:
            {
                if (recv[0] == A_ANSWER_RECEIVER)
                {
                    rc = A_Rcv;
                    ac[0] = recv[0];
                }
                else if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            case A_Rcv:
            {
                if ((recv[0] == C_RR0) || (recv[0] == C_RR1))
                {
                    rc = C_Rcv;
                    ret = TRUE;
                    ac[1] = recv[0];
                }
                else if ((recv[0] == C_REJ0) || (recv[0] == C_REJ1))
                {
                    rc = C_Rcv;
                    ret = FALSE;
                }
                else if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            case C_Rcv:
            {
                if (recv[0] == (ac[0] ^ ac[1]))
                {
                    rc = Bcc_OK;
                }
                else if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            case Bcc_OK:
            {
                if (recv[0] == FLAG)
                {
                    rc = Stop;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            }
        }
    }
    return ret;
}

// returns the control field of the control frame if it was received correctly
// returns 0 if the control frame was corrupted
unsigned char checkControlFrame(int fd, unsigned char addressField)
{
    printf("In the checkControlFrame function\n");
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    unsigned char controlField = 0;

    while (alarmEnabled && (rc != Stop))
    {
        read(fd, recv, 1);
        if (recv[0] > 0)
        {
            switch (rc)
            {
            default:
                break;
            case Start:
            {
                if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                    break;
                }
                break;
            }
            case Flag_Rcv:
            {
                if (recv[0] == addressField)
                {
                    rc = A_Rcv;
                    ac[0] = recv[0];
                }
                else if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            case A_Rcv:
            {
                if ((recv[0] == C_SET) || (recv[0] == C_UA) || (recv[0] == C_DISC) || (recv[0] == C_RR0) || (recv[0] == C_RR1) || (recv[0] == C_REJ0) || (recv[0] == C_REJ1))
                {
                    rc = C_Rcv;
                    controlField = recv[0];
                    ac[1] = recv[0];
                }
                else if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            case C_Rcv:
            {
                if (recv[0] == (ac[0] ^ ac[1]))
                {
                    rc = Bcc_OK;
                }
                else if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            case Bcc_OK:
            {
                if (recv[0] == FLAG)
                {
                    rc = Stop;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            }
        }
    }
    return controlField;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int llwrite(const unsigned char *buf, int bufSize)
{
    Frame frame = createInformationFrame(buf, bufSize, sequenceNumber, A_FRAME_SENDER);
    alarmCount = 0;
    while (alarmCount < cp.nRetransmissions)
    {
        if (write(fd, frame.data, frame.size) == -1)
        {
            printf("Error writing data packet.\n");
            return -1;
        }
        else
        {
            printf("Data packet sent.\n");
        }
        (void)signal(SIGALRM, alarmHandler); // set alarm only for sender
        alarm(cp.timeout);
        alarmEnabled = TRUE;

        while (alarmEnabled)
        {
            unsigned char r = checkControlFrame(fd, A_ANSWER_RECEIVER); // todo: probably implement state machine inside this function
            if ((r == C_RR0) || (r == C_RR1))
            {
                printf("ACK packet received.\n");
                alarm(0);
                alarmEnabled = FALSE;
                alarmCount = 0;
                sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sequence number
                return frame.size;
            }
            else if ((r == C_REJ0) || (r == C_REJ1))
            {
                alarm(0);
                alarmEnabled = FALSE;
                break;
            }
            else
            {
                printf("The ack packet was corrupted.\n");
                printf("Control field: %x\n", r);
                alarm(0);
                alarmEnabled = FALSE;
                break;
            }
        }
    }
    return -1;
}

// return 1 if the connection was terminated correctly or -1 if not
int terminate_connection(int fd)
{
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    close(fd);
    return 1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) // packet has 1000bytes size
{
    enum CheckRecv rc = Start;
    unsigned char recv[1] = {0};
    unsigned char ac[2] = {0};
    int dataSize = 0;
    int stuffedDataSize = 0;
    unsigned char stuffedData[MAX_STUFFED_DATA_SIZE];
    unsigned char tmpData[MAX_PAYLOAD_SIZE + 1];

    while (rc != Data_Rcv)
    {
        if (read(fd, recv, 1) > 0)
        {
            printf("Received byte: %x\n", recv[0]);
            switch (rc)
            {
            default:
                break;
            case Start:
            {
                if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                break;
            }
            case Flag_Rcv:
            {
                if (recv[0] == A_FRAME_SENDER)
                {
                    rc = A_Rcv;
                    ac[0] = recv[0];
                }
                else if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                else if ((recv[0] == C_DISC)) {
                    printf("Got disc frame\n");
                    Frame uaPacket = createControlFrame(A_ANSWER_RECEIVER, C_DISC);
                    if (write(fd, uaPacket.data, uaPacket.size) == -1)
                    {
                        printf("Error writing UA packet.\n");
                        return -1;
                    }
                    else
                    {
                        printf("UA packet sent.\n");
                    }
                    terminate_connection(fd);
                    return 0; // return 0 to indicate that the connection was terminated
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            case A_Rcv:
            {
                // check c (0x00 | (expectedSequenceNumber << 6))
                // Simply get the control and pass to next state
                if (recv[0] == (0x00 | (sequenceNumber) << 6))
                {
                    rc = C_Rcv;
                    ac[1] = recv[0];
                }
                else if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                else if (recv[0] == C_DISC)
                {
                    printf("Got disc frame\n");
                    Frame uaPacket = createControlFrame(A_ANSWER_RECEIVER, C_UA);
                    if (write(fd, uaPacket.data, uaPacket.size) == -1)
                    {
                        printf("Error writing UA packet.\n");
                        return -1;
                    }
                    else
                    {
                        printf("UA packet sent.\n");
                    }
                    terminate_connection(fd);
                    return -1;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            case C_Rcv:
            {
                if (recv[0] == (ac[0] ^ ac[1]))
                {
                    rc = Bcc_OK;
                }
                else if (recv[0] == FLAG)
                {
                    rc = Flag_Rcv;
                }
                else
                {
                    rc = Start;
                }
                break;
            }
            case Bcc_OK:
            {
                // fill data array with data and bcc2
                if (recv[0] == FLAG)
                {
                    rc = Data_Rcv;
                }
                else if (stuffedDataSize < MAX_STUFFED_DATA_SIZE)
                {
                    stuffedData[stuffedDataSize] = recv[0];
                    stuffedDataSize++;
                }
                break;
            }
            }
        }
    }
    // if flag was found on BCC_OK, the byte before the flag is bcc2 which is xor of all the databytes
    dataSize = destuffIt(stuffedData, stuffedDataSize, tmpData);

    // calculate xor of data bytes (except bcc2)
    unsigned char dataXor = 0;
    for (int i = 0; i < (dataSize - 1); i++)
    {
        dataXor ^= tmpData[i];
    }
    
    // check if bcc2 is correct
    if (dataXor == tmpData[dataSize - 1])
    {
        memcpy(packet, tmpData, dataSize-1);
        // send ACK
        Frame ackFrame = createControlFrame(A_ANSWER_RECEIVER, C_RR0 | (((sequenceNumber+1)%2) << 7));
        if (write(fd, ackFrame.data, ackFrame.size) == -1)
        {
            printf("Error writing ACK packet.\n");
            return -1;
        }
        else
        {
            printf("ACK packet sent.\n");
        }
        sequenceNumber = (sequenceNumber + 1) % 2; // Toggle sequence number
        return dataSize-1;
    }
    printf("The data packet was corrupted.\n");
    // send NACK
    Frame nackFrame = createControlFrame(A_ANSWER_RECEIVER, C_REJ0 | (sequenceNumber << 7));
    if (write(fd, nackFrame.data, nackFrame.size) == -1)
    {
        printf("Error writing NACK packet.\n");
        return -1;
    }
    else
    {
        printf("NACK packet sent.\n");
        return -1;
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    if(cp.role == LlTx) {    
        enum CheckRecv rc = Start;
        unsigned char recv[1] = {0};

        alarmCount = 0;
        while ((alarmCount < cp.nRetransmissions) && (rc != Stop))
        {
            // build and send packet
            Frame discPacket = createControlFrame(A_FRAME_SENDER, C_DISC);
            if (write(fd, discPacket.data, discPacket.size) == -1)
            {
                printf("Error writing DISC packet.\n");
                return -1;
            }
            else
            {
                printf("DISC packet sent.\n");
            }
            (void)signal(SIGALRM, alarmHandler);
            alarm(cp.timeout);
            alarmEnabled = TRUE;

            while (alarmEnabled && (rc != Stop))
            {
                ssize_t recvSize = read(fd, recv, 1);
                if (recvSize > 0)
                {
                    switch (rc)
                    {
                    default:
                        break;
                    case Start:
                    {
                        if (recv[0] == FLAG)
                        {
                            rc = Flag_Rcv;
                        }
                        break;
                    }
                    case Flag_Rcv:
                    {
                        if (recv[0] == A_ANSWER_RECEIVER)
                        {
                            rc = A_Rcv;
                        }
                        else if (recv[0] == FLAG)
                        {
                            rc = Flag_Rcv;
                        }
                        else
                        {
                            rc = Start;
                        }
                        break;
                    }
                    case A_Rcv:
                    {
                        if (recv[0] == C_DISC)
                        {
                            rc = C_Rcv;
                        }
                        else if (recv[0] == FLAG)
                        {
                            rc = Flag_Rcv;
                        }
                        else
                        {
                            rc = Start;
                        }
                        break;
                    }
                    case C_Rcv:
                    {
                        if (recv[0] == (A_ANSWER_RECEIVER ^ C_DISC))
                        {
                            rc = Bcc_OK;
                        }
                        else if (recv[0] == FLAG)
                        {
                            rc = Flag_Rcv;
                        }
                        else
                        {
                            rc = Start;
                        }
                        break;
                    }
                    case Bcc_OK:
                    {
                        if (recv[0] == FLAG)
                        {
                            rc = Stop;
                        }
                        else
                        {
                            rc = Start;
                        }
                        break;
                    }
                    }
                }
            }
        }
        if(rc != Stop) {
            printf("Failed to receive DISC packet.\n");
            terminate_connection(fd);
            return -1;
        }
        
        // send UA
        Frame uaPacket = createControlFrame(A_ANSWER_SENDER, C_UA);
        if (write(fd, uaPacket.data, uaPacket.size) == -1)
        {
            printf("Error writing UA packet.\n");
            terminate_connection(fd);
            return -1;
        }
        else
        {
            printf("UA packet sent.\n");
        }

        return terminate_connection(fd);

    } else if(cp.role == LlRx) {
        // wait for DISC and write DISC
        alarmCount = 0;

        (void)signal(SIGALRM, alarmHandler);
        alarm(cp.timeout);
        alarmEnabled = TRUE;
        if(checkControlFrame(fd, A_FRAME_SENDER) == C_DISC) {
            Frame uaPacket = createControlFrame(A_ANSWER_RECEIVER, C_DISC);
            if (write(fd, uaPacket.data, uaPacket.size) == -1)
            {
                printf("Error writing DISC packet.\n");
                terminate_connection(fd);
                return -1;
            }
            else
            {
                printf("DISC packet sent.\n");
            }
            return terminate_connection(fd);
        }
        printf("Failed to receive DISC packet.\n");
        terminate_connection(fd);
        return -1;
    }
    return -1;
}
