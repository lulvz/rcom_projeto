#include "frame.h"

// Creation of Frames
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

// Decoding the Frames
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



// The send by the application layer must be stuffed before it is Framed

/*
int stuffIt(unsigned char *buf, int startingByte, int length, unsigned char *stuffedData){
	int msgSize = 0;

	for(int i = 0; i < startingByte; i++) stuffedData[msgSize++] = buf[i];

	for()
*/
