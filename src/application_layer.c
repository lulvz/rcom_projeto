// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    if (strcmp(role, "tx") == 0)
    {
        LinkLayer ll;
        strcpy(ll.serialPort, serialPort);
        ll.role = LlTx;
        ll.baudRate = baudRate;
        ll.nRetransmissions = nTries;
        ll.timeout = timeout;

        if (llopen(ll) == -1)
        {
            printf("Error opening serial port\n");
            exit(-1);
        }

        // open file and read it into a buffer
        int fd = open(filename, O_RDONLY);
        if (fd == -1)
        {
            printf("Error opening file\n");
            exit(-1);
        }

        struct stat st;
        if (stat(filename, &st) == -1)
        {
            printf("Error getting file size\n");
            exit(-1);
        }

        int fileSize = st.st_size;
        unsigned char *fileBuffer = malloc(fileSize);
        if (fileBuffer == NULL)
        {
            printf("Error allocating memory for file buffer\n");
            exit(-1);
        }

        int bytesRead = read(fd, fileBuffer, fileSize);
        if (bytesRead == -1)
        {
            printf("Error reading file\n");
            exit(-1);
        }
        if (bytesRead != fileSize)
        {
            printf("Error reading file\n");
            exit(-1);
        }

        close(fd);

        // send file
        if (llwrite(fileBuffer, fileSize) == -1)
        {
            printf("Error sending file\n");
            exit(-1);
        }
    }
    else if (strcmp(role, "rx") == 0)
    {
        LinkLayer ll = {0};
        strcpy(ll.serialPort, serialPort);
        ll.role = LlRx;
        ll.baudRate = baudRate;
        ll.nRetransmissions = nTries;
        ll.timeout = timeout;

        if (llopen(ll) == -1)
        {
            printf("Error opening serial port\n");
            exit(-1);
        }

        // receive file
        unsigned char *fileBuffer; // will be dynamic memory allocated
        int fileSize = llread(fileBuffer);
        if (fileSize == -1)
        {
            printf("Error receiving file\n");
            exit(-1);
        }

        // write file
        int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (fd == -1)
        {
            printf("Error opening file\n");
            exit(-1);
        }

        int bytesWritten = write(fd, fileBuffer, fileSize);
        if (bytesWritten == -1)
        {
            printf("Error writing file\n");
            exit(-1);
        }
        if (bytesWritten != fileSize)
        {
            printf("Error writing file\n");
            exit(-1);
        }

        close(fd);
    }
    else
    {
        printf("Invalid role\n");
        exit(-1);
    }
    if (llclose(FALSE) == -1)
    {
        printf("Error closing serial port\n");
        exit(-1);
    }
}
