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
#define ESC 0x7D

volatile int STOP = FALSE;

unsigned char rr = 0x00;

// renamed to i_value to avoid confusion with i for-loop variable
unsigned char i_value = 0x00; 

int fd = -1;
unsigned char buf[BUF_SIZE] = {0};

int alarmEnabled = FALSE;
int alarmCount = 0;

enum GeneralState {SET_UA, I_RR};
enum GeneralState general_state = SET_UA;

unsigned char frame0[] = {0x01,0x04,0x29};
unsigned char frame1[] = {0x02,0x04,0x29};

unsigned char *frames[] = {frame0, frame1};
size_t frame_sizes[] = {sizeof(frame0), sizeof(frame1)};

void llopen(){
    
    int bytes = write(fd, buf, BUF_SIZE);
    printf("%d bytes written\n", bytes);

    // Wait until all bytes have been written to the serial port
    
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
                printf("0x%02X \n", responseBuf[i]);
            
        } else {
            printf("UA received!\n");
            printf(":%s:%d\n", buf, bytes);

            for (int i = 0; i < 5; i++) 
                printf("0x%02X \n", responseBuf[i]);
            
            general_state = I_RR;
            alarmCount = 0; // Reset alarm count
            alarm(0);
        }
    } else {
        printf("No response received.\n");
    }

}

void llwrite(unsigned char i_value){

    // Create string to send
    buf[0] = FLAG; 
    buf[1] = 0x03; // A
    buf[2] = i_value; // C
    buf[3] = 0x03 ^ i_value; // bcc1

    unsigned char bcc2 = 0x00;

    unsigned char *selectedFrame = frames[i_value];
    size_t frameSize = frame_sizes[i_value]; 

    size_t pos = 4;

    for (int j = 0; j < frameSize; j++, pos++) {

        unsigned char value = selectedFrame[j];

        if (value == FLAG){
            buf[pos] = ESC;
            buf[++pos] = 0x5E;
        } else if (value == ESC){
            buf[pos] = ESC;
            buf[++pos] = 0x5D;
        } else
            buf[pos] = selectedFrame[j]; 

        bcc2 ^= selectedFrame[j];
    }

// frame0 = {0x01,0x04,0x29};
// frame1 = {0x02,0x04,0x29};

    buf[pos] = bcc2;
    buf[pos+1] = FLAG;

    printf("buf to send: \n");
    for (int i = 0; i < frameSize + 6; i++) 
        printf("buf[%d]: 0x%02X \n", i, buf[i]);

    int bytes = write(fd, buf, BUF_SIZE);
    printf("%d bytes written during llwrite\n", bytes);
    
    // Wait until all bytes have been written to the serial port
    unsigned char responseBuf[BUF_SIZE];
    int responseBytes = read(fd, responseBuf, BUF_SIZE);

    if (responseBytes > 0) {
        unsigned char a = responseBuf[1];
        unsigned char c = responseBuf[2];
        unsigned char check = a ^ c;
        unsigned char bcc = responseBuf[3];

        if(bcc != check){
            printf(" A ^ C != BCC1\n");
            printf(":%s:%d\n", responseBuf, bytes);
            for (int i = 0; i < 5; i++) 
                printf("0x%02X \n", responseBuf[i]);
            
        } else {
            printf(" bcc==check! \n");
            printf(":%s:%d\n", responseBuf, bytes);

            for (int i = 0; i < 5; i++) 
                printf("0x%02X \n", responseBuf[i]);  

            switch (c) {
                case 0x54:
                    rr = 0;
                    printf("REJ0\n");
                    break;
                case 0xAA:
                    rr = 0;
                    printf("RR0\n");
                    alarmCount = 0; // Reset alarm count   
                    alarm(0); // Disable alarm          
                    break;
                case 0x55:
                    rr =1;
                    printf("REJ1\n");
                    break;
                case 0xAB:
                    rr = 1;
                    printf("RR1\n");
                    alarmCount = 0; // Reset alarm count    
                    alarm(0); // Disable alarm            
                    break;
                default:
                    break;
            }

            if (rr == i_value){ // If the received RR is the same as the sent I, we retry the transmission
                printf("Retransmitting frame, i = %d, alarmCount = %d\n", i_value, alarmCount);

                if (alarmEnabled == FALSE) {
                    printf("Setting alarm, alarmCount = %d\n", alarmCount);
                    alarm(3); // Set alarm to be triggered in 3s
                    alarmEnabled = TRUE;
                }

            } else       // If the received RR is different from the sent I,
                i_value = !i_value;  // change the I value
            
            // by default, the next I is the same as the previous one
            printf("next i = %d\n", i_value); 
            
        }
    } else {
        printf("No response received,  %d  alarmCount = %d \n", i_value, alarmCount);

        // If no response is received, we retry the transmission
        if (alarmEnabled == FALSE) {
            printf("Setting alarm, alarmCount = %d\n", alarmCount);
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
        }
    }
}

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);

    printf("General state: %d\n", general_state);

    switch (general_state){
        case SET_UA:
            llopen();
            break;
        case I_RR:    
            printf("Frame not received, retransmitting i = %d\n", i_value);
            llwrite(i_value);
            break;
        default:
            printf("Invalid general state, %d\n", general_state);

            // TODO: put it in the end
            alarm(0); // Disable alarm 
            break;
    } 
    
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
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

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
    // buf[3]=0xFF;
    buf[4] = FLAG; 

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.
    buf[5] = '\n';

    llopen();

    while (alarmCount < 3 && general_state==SET_UA)
    {
        if (!alarmEnabled)
        {
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
        }
    }

    while (alarmCount < 3 && general_state == I_RR){
        if (!alarmEnabled){
            printf("Alarm disabled, writing frame %d\n", i_value);
            llwrite(i_value);
        }
    }

    printf("Exausted all attempts\n");

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
