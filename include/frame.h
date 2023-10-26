#ifndef _FRAME_
#define _FRAME_

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "link_layer.h"
#include "constants.h"

// Types of Frames
enum FrameType {
    I,
    C,
};

// Structure of a Frame
typedef struct {
        unsigned char data[MAX_FRAME_SIZE];
        int size;
} Frame;

int stuffIt(const unsigned char *buf, int bufSize, unsigned char stuffedData[MAX_STUFFED_DATA_SIZE]);

int destuffIt(unsigned char* buf, int bufSize, unsigned char deStuffedData[MAX_PAYLOAD_SIZE+1]);

// Creation of Frames
Frame createInformationFrame(const unsigned char *data, int dataSize, int sequenceNumber, unsigned char addressField);

Frame createControlFrame(const unsigned char addressField, unsigned char controlField);


// Decoding the Frames
int decodeInformationFrame(int fd, int expectedSequenceNumber, unsigned char *data);

int checkControlFrame(int fd, unsigned char addressField, unsigned char controlField);

#endif 
