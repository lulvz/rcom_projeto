#include "receiver.h"

// Use in the Type fields of the CONTROL packets
#define FILESIZE 0
#define FILENAME 1

// Use in the control fields of the packets
#define DATA 1
#define START 2
#define END 3


int mainReceiver(const char *path){
	int ret;
	int numBytes = 0;
	const char *filename;

	const char *temp = path + strlen(path) - 1;
	while(*temp != '/'){
		if(temp == path) break;	
		temp--;
	}
	filename = temp;

	while(1){
		printf("In the beggining\n");
		unsigned char packet[MAX_PAYLOAD_SIZE];
		if((numBytes = llread(packet)) < 0){
			fprintf(stderr, "Error receiving packet.\n");
			return -1;
		}

		printf("Packet received with %d bytes ", numBytes);
		for(unsigned i = 0; i < numBytes; i++){
    			printf("%02X ", packet[i]); // Print each byte in hexadecimal format
			if(i == numBytes - 1) printf("\n");
		}

		// make copy of filename into filename_cpy
		char filename_cpy[strlen(filename)];
		strcpy(filename_cpy, filename);
		// With the packet in hands, need to handle it
		if((ret = (handlePacket(packet, numBytes, filename_cpy))) < 0){
			printf("Error!!!\n");
			return -1;
		}
		else if(ret == END){
			break;
		}

	}
	
	printf("Received all the packets!!!\n");
	return 0;
}


int handlePacket(unsigned char *packet, int numBytes, const char *filename){
	int filesize = 0;
	char sourceFile[50];
	// This will be the fd that will represent the file to be written
	static int fd;

	switch(packet[0]){
		case START :
			{
			printf("Parsing start packet\n");
			parseStartEnd(packet, numBytes, sourceFile, &filesize);
			if((fd = open(filename, O_WRONLY | O_CREAT, 0777)) < 0){
				perror("Error opening the file in handlePacket function\n");
				return -1;
			}
			return 0;}

		case DATA :
			{// I think I should probably check the sqNum
			int bytes2write = packet[2]*256 + packet[3];
			if(write(fd,&packet[4], bytes2write) < 0){
				perror("Error writing the bytes to the fd\n");
				return -1;
			}
			printf("Data packet parsed!\n");
			return 0;}

		case END :
			{
			printf("Parsing end packet\n");
			parseStartEnd(packet, numBytes, filename, &filesize);
			if(close(fd) < 0){
				perror("Error closing the file in handlePacket\n");
				return -1;
			}
			//He checks the size of the file here...
			return END;}	
		
		default :
			{printf("Packet structure not existent\n");
			for (int i = 0; i < numBytes; i++) {
            			printf("0x%02x ", packet[i]);
        		}
			return -1;}
	}
}


void parseStartEnd(unsigned char * buffer, int length, const char* path, int* filesize) {

    // filesize ---> holds the length of the file that was sent	
    // path ---> holds the name of the file that was sent
    // It starts from 1, cause the 0 is simply the packet identifier(start/end)
    for (int i = 1; i < length; i++) {
        if (buffer[i] == FILESIZE) {
            i++; // i is now in the byte with information about the number of bytes
            for (int j = 0; j < buffer[i]; j++) {
                *filesize |= (buffer[i+j+1] << (8*j));
            }
            i += buffer[i];
        }

        if (buffer[i] == FILENAME) {
            i++; // i is now in the byte with information about the number of bytes
            char fileName[buffer[i] + 1];
            for (int j = 0; j < buffer[i]; j++) {
                fileName[j] = buffer[i + j + 1];
            }
            fileName[buffer[i]] = '\0';
			//make copy of path
			char path_cpy[strlen(path)];
			strcpy(path_cpy, path);
            strcat(path_cpy, fileName);
            i += buffer[i];
        }
    }
}

