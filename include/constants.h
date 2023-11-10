#ifndef _CONSTANTS_
#define _CONSTANTS_

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_STUFFED_DATA_SIZE MAX_PAYLOAD_SIZE * 2 + 2

// Maximum Frame Size = Maximum Payload Size times 2 (in case every byte is a flag or an escape sequence) + Overhead (header + trailer)
#define MAX_FRAME_SIZE (MAX_STUFFED_DATA_SIZE + 4 + 2)

#define FLAG 0x7E
#define A_FRAME_SENDER 0x03
#define A_ANSWER_RECEIVER 0x03
#define A_FRAME_RECEIVER 0X01
#define A_ANSWER_SENDER 0X01
#define C_SET 0X03
#define C_UA 0X07
#define C_RR0 0x05 // sent by the Receiver when it is ready to receive an information frame number 0
#define C_RR1 0x85 // sent by the Receiver when it is ready to receive an information frame number 1
#define C_REJ0 0x01 // sent by the Receiver when it rejects an information frame number 0 (detected an error)
#define C_REJ1 0x81 // sent by the Receiver when it rejects an information frame number 1 (detected an error)
#define C_DISC 0X0B
#define C_I0 0x00 // information frame number 0
#define C_I1 0x40 // information frame number 1
		  
#define TRUE 1
#define FALSE 0

enum CheckRecv {
    Start,
    Flag_Rcv,
    A_Rcv,
    C_Rcv,
    Bcc_OK,
    Data_Rcv,
    Stop,
};

#endif
