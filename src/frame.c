#include "frame.h"


int FER_BCC1 = 0;
int FER_BCC2 = 0;

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

// returns the size of the de-stuffed data, this should be a pointer to an array with size MAX_PAYLOAD_SIZE+1 for the bcc2
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
    Frame frame = {0};

    frame.data[0] = FLAG;
    frame.data[1] = addressField;
    frame.data[2] = (sequenceNumber << 6) | 0x00; // 0x00 = I-frame number 0 | 0x40 = I-frame number 1
    printf("In createInformationFrame func frame.data[2] is %x\n", frame.data[2]);
    frame.data[3] = frame.data[1] ^ frame.data[2]; // BCC1
						  
    int x = rand() % 100;
    if(x < FER_BCC1 ){
	    printf("RAND ---> %d\n", x);
	    frame.data[3] = frame.data[3] ^ 0xFF;
    }
    printf("Control frame %x\n", frame.data[3]);

    unsigned char bcc2 = 0;
    // bcc2 is xor of all the data bytes
    for (int i = 0; i < dataSize; i++) {
        bcc2 ^= data[i];
    }

    // add bcc2 to the end of the data to be stuffed
    unsigned char dataWithBcc2[MAX_PAYLOAD_SIZE+1];
    memcpy(dataWithBcc2, data, dataSize);
    dataWithBcc2[dataSize] = bcc2;
    if(rand() % 100 < FER_BCC2) dataWithBcc2[dataSize] ^= dataWithBcc2[dataSize] ^ 0xFF;
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
