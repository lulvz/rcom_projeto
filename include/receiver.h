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

int mainReceiver(const char *path);


int handlePacket(unsigned char *packet, int numBytes, const char *filename);


void parseStartEnd(unsigned char * buffer, int lenght, const char* path, int* filesize);

#endif
