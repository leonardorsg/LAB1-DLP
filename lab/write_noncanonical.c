// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 256
#define FLAG 0x7E

volatile int STOP = FALSE;

int ua_received = FALSE;
int fd = -1;
unsigned char buf[BUF_SIZE] = {0};

int alarmEnabled = FALSE;
int alarmCount = 0;

void llopen(){
    
    int bytes = write(fd, buf, BUF_SIZE);
    printf("%d bytes written\n", bytes);

    // // Wait until all bytes have been written to the serial port
    
    unsigned char responseBuf[BUF_SIZE];
    int responseBytes = read(fd, responseBuf, BUF_SIZE);

    if (responseBytes > 0) {
        unsigned char a = buf[1];
        unsigned char c = buf[2];
        unsigned char check = a ^ c;
        unsigned char bcc = buf[3];

        if(bcc != check){
            printf(" A ^ C != BCC1\n");
            printf(":%s:%d\n", buf, bytes);
            for (int i = 0; i < 5; i++) 
                printf("0x%02X ", responseBuf[i]);
            
        } else {
            printf("UA received!\n");
            printf(":%s:%d\n", buf, bytes);
            for (int i = 0; i < 5; i++) 
                printf("0x%02X ", responseBuf[i]);
            
            alarm(0); // Disable alarm
            ua_received = TRUE;
        }
    } else {
        printf("No response received.\n");
    }

}

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);

    llopen();

}

int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 30; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    // Create string to send
    buf[0] = FLAG; 
    buf[1] = 0x03; // A
    buf[2] = 0x03; // C
    buf[3] = 0x03 ^ 0x03; // bcc1
    buf[4] = FLAG; 

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.
    buf[5] = '\n';

    llopen();
    while (alarmCount < 3 && ua_received == FALSE)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
        }
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
