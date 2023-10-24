#include "transmitter.h"

// CONTROL PACKETS
// | C | T1 | L1 | V1 | T2 | L2 | V2 | ... | Tn | Ln | Vn |

// DATA PACKETS
// | C | L2 | L1 | P1 | P2 | ... | Pn |

// Use in the Type fields of the CONTROL packets
#define FILESIZE 0
#define FILENAME 1

// Use in the control fields of the packets
#define DATA 1
#define START 2
#define END 3

int mainTransmitter(const char *path){
	struct stat st;
	int fileFd;
	
	// Reads file info using stat
    	if (stat(path, &st) < 0){
    	    perror("Error with command stat, in the mainTransmitter function.");
    	    return -1;
    	}

	// Opens file to transmit
    	if ((fileFd = open(path, O_RDONLY)) < 0){
    	    perror("Error opening file in mainTransmitterfunction.");
    	    return -1;
    	}

	char *fileName;
	char *temp = path + strlen(path) - 1;
	while(*temp != '/'){
		if(temp == path) break;	
		temp--;
	}
	fileName = temp;

	printf("Filename got : %s\n", fileName);

	// Send START packet
	if(sendControlPacket(START, st.st_size, fileName) < 0 ){
		fprintf(stderr, "Error sending START packet.\n");
		return -1;
	}
	printf("Start packet sent!!!\n");
	
	// Send DATA packet
	if(sendDataPacket(fileFd) < 0){
		fprintf(stderr, "Error sending DATA packet's.\n");
		return -1;
	}
	printf("Data packets sent!!!\n");

	// Send END packet
	if(sendControlPacket(END, st.st_size, fileName) < 0){
		fprintf(stderr, "Error sending END packet.\n");
		return -1;
	}
	printf("End packet sent!!!\n");

	return close(fileFd);
}

int sendDataPacket(int fileFd){
	unsigned char buf[MAX_PAYLOAD_SIZE];
	unsigned bytes2send;
	unsigned sqcNum = 0; 

	while((bytes2send = read(fileFd, buf, MAX_PAYLOAD_SIZE - 4 )) > 0 ){
		unsigned char dataPacket[MAX_PAYLOAD_SIZE];

		dataPacket[0] = DATA;
		dataPacket[1] = sqcNum % 255;
        	dataPacket[2] = (bytes2send / 256);
        	dataPacket[3] = (bytes2send % 256);
        	memcpy(&dataPacket[4], buf, bytes2send);

		printf("Application layer: sending data packet with %d bytes\n", bytes2send+4);
		for(unsigned i = 0; i < bytes2send+4; i++){
			printf("%02X ", dataPacket[i]); // Print each byte in hexadecimal format
		}
		if(llwrite(dataPacket, bytes2send+4) == -1){
			fprintf(stderr, "Error with llwrite in function sendDataPacket\n");
			return -1;
		}
		sqcNum++;
	}
	// Success
	return 1;
}
		

int sendControlPacket(unsigned ctrl, unsigned fileSize, char *fileName){
	unsigned L1 = sizeof(fileSize);
	unsigned L2 = strlen(fileName);	
	// 5 stands for [C, T1, L1, ..., T2, L2 ] 
	// C = control; T = Type(size or name of the file); L = size
	unsigned packetSize = 5 + L1 + L2;

	printf("L1 %u\n", L1);
	printf("L2 %u\n", L2);
	printf("packetSize %u\n", packetSize);

	unsigned char packet[packetSize];
	packet[0] = ctrl;

	packet[1] = FILESIZE;
	packet[2] = L1;
	for (unsigned i = 0; i < L1; i++) {
        	packet[3 + i] = (fileSize >> (8 * i)) & 0xFF;
    	}

	packet[3+L1] = FILENAME;
	packet[4+L1] = L2;
	for(unsigned i = 0; i < L2; i++){
		packet[5 + L1 + i] = fileName[i];
	}

	for (unsigned i = 0; i < packetSize; i++) {
    		printf("%02X ", packet[i]); // Print each byte in hexadecimal format
	}


	return llwrite(packet, packetSize);
}
