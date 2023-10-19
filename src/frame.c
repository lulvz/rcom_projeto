#include "frame.h"

// returns the size of the new stuffed data or -1 on error, should take a pointer to an array with size MAX_STUFFED_DATA_SIZE
int stuffIt(const unsigned char *buf, int bufSize, unsigned char *stuffedData) {
    int stuffedDataIdx = 0;

    for(int i = 0; i<bufSize; i++) {
        if(buf[i] == FLAG) {
            stuffedData[stuffedDataIdx] = 0x7d;
            stuffedData[stuffedDataIdx + 1] = 0x5e;
            // 2 byte new size
            stuffedDataIdx += 2;
        } else if (buf[i] == 0x7d) {
            stuffedData[stuffedDataIdx] = 0x7d;
            stuffedData[stuffedDataIdx] = 0x5d;
            //2 byte new size
            stuffedDataIdx += 2;
        } else {
            stuffedData[stuffedDataIdx] = buf[i];
            stuffedDataIdx++;
        }
    }

    return stuffedDataIdx;
}

// returns the size of the de-stuffed data, this should be a pointer to an array with size MAX_PAYLOAD_SIZE
int destuffIt(unsigned char* buf, int bufSize, unsigned char *deStuffedData) {
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
    Frame frame = {0};
    frame.size = dataSize + 4 + 2;

    frame.data[0] = FLAG;
    frame.data[1] = addressField;
    frame.data[2] = (sequenceNumber << 6) | 0x00; // 0x00 = I-frame number 0 | 0x40 = I-frame number 1
    frame.data[3] = frame.data[1] ^ frame.data[2]; // BCC1

    unsigned char stuffedData[MAX_STUFFED_DATA_SIZE];
    int stuffedDataSize = stuffIt(data, dataSize, stuffedData);

    memcpy(&frame.data[4], stuffedData, stuffedDataSize);

    unsigned char bcc2 = 0;
    // bcc2 is xor of all the data bytes
    for (int i = 0; i < dataSize; i++) {
        bcc2 ^= data[i];
    }

    frame.data[4 + stuffedDataSize] = bcc2;
    frame.data[4 + stuffedDataSize + 1] = FLAG;

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
    unsigned char bcc2 = 0x0;
    unsigned char dataXor = 0;
    int dataSize = 0;
    int stuffedDataSize = 0;
    unsigned char stuffedData[MAX_STUFFED_DATA_SIZE];
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
                    stuffedDataSize--; // remove bcc2 from data size
                    bcc2 = stuffedData[stuffedDataSize];
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
                dataSize = destuffIt(stuffedData, stuffedDataSize, data);

                // calculate xor of data bytes (data array minus byte after dataSize)
                for(int i = 0; i < dataSize; i++) {
                    dataXor ^= data[i];
                }
                if(dataXor == bcc2) {
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
