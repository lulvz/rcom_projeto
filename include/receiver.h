#ifndef _RECEIVER_
#define _RECEIVER_

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "constants.h"
#include "link_layer.h"

int mainReceiver();


int handlePacket(unsigned char *packet, int numBytes);


void parseStartEnd(unsigned char * buffer, int lenght, char* path, int* filesize);

#endif
