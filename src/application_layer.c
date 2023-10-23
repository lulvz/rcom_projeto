// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "transmitter.h"
#include "receiver.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The application control packets look like this:
// | C | T1 | L1 | V1 | T2 | L2 | V2 | ... | Tn | Ln | Vn |

// The application data packets look like this:
// | C | L2 | L1 | P1 | P2 | ... | Pn |

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
	
	mainTransmitter(filename);

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

	mainReceiver();

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
