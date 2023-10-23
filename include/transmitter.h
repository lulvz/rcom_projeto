#ifndef _TRANSMITTER_
#define _TRANSMITTER_

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "link_layer.h"

// Format
// | C | T1 | L1 | V1 | T2 | L2 | V2 | ... | Tn | Ln | Vn |
int sendControlPacket(unsigned ctrl, unsigned fileSize, char *fileName);

// Format 
// | C | L2 | L1 | P1 | P2 | ... | Pn |
int sendDataPacket(int fileFd);

int mainTransmitter(const char *path);
#endif 
