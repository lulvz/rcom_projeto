#include "frame.h"

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
        switch(rc) {
            case Start: {
                read(fd, recv, 1);
                // printf("Received: %x\n", recv[0]);
                if(recv[0] == FLAG) {
                    rc = Flag_Rcv;
                    break;
                }
                return -1;
            }
            case Flag_Rcv: { 
                read(fd, recv, 1);
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
                read(fd, recv, 1);
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
                read(fd, recv, 1);
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
                read(fd, recv, 1);
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
        read(fd, recv, 1);
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
