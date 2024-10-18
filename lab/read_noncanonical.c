// Read from serial port in non-canonical mode
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
#include <stdbool.h>
// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 256
#define FLAG 0x7E
#define A 0x03
#define C 0x03
#define ESC 0x7D

volatile int STOP = FALSE;

enum State {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_};
enum State state = START;

int fd = -1;
unsigned char buf[BUF_SIZE + 1] = {0}; // +1: Save space for the final '\0' char

unsigned char i_signal = 0x12;
unsigned char rr_signal = 0x12;
unsigned char counter = ESC;

enum State_ACK {START_ack, FLAG_RCV_ack, A_RCV_ack, C_RCV_ack, BCC1_OK_ack, STOP_ack};
enum State_ACK state_ack = START_ack;

enum GeneralState {SET_UA, I_RR, CLOSE};
enum GeneralState general_state = SET_UA;

int llread(){

    STOP = FALSE;
    unsigned char bcc2_value = 0x00;
    unsigned char is_esc = false;

    while ((STOP == FALSE) && (general_state == I_RR))
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, buf, 1);
        buf[bytes] = '\0'; // Set end of string to '\0', so we can printf
        switch(state_ack){
            case START_ack:
                if(buf[0] == FLAG){
                    state_ack = FLAG_RCV_ack;
                }
                //printf("start");
            break;
            case FLAG_RCV_ack:
                if(buf[0] == A){
                    state_ack  = A_RCV_ack;
                    printf("flag to a\n");
                } else if (buf[0] != FLAG) {
                    state_ack  = START_ack;
                    printf("flag to start\n");
                }
                break;

            case A_RCV_ack:
                if((buf[0] == 0x00) || (buf[0] == 0x80)){
                    state_ack = C_RCV_ack;
                    i_signal = buf[0];
                    printf("A_RCV_ack to C_RCV_ack, i_signal = %x\n", i_signal);
                } else if (buf[0] == FLAG) {
                    state_ack = FLAG_RCV_ack;
                    printf("a to flag t\n");
                } else {
                    state_ack = START_ack;
                    printf("a to start\n");
                }
                break;
            case C_RCV:
                if(buf[0] == (A ^ i_signal)){
                    state_ack  = BCC1_OK_ack;
                    printf("c to bcc1\n");
                } else if (buf[0] == FLAG) {
                    state_ack  = FLAG_RCV_ack;
                    printf("c to flag\n");
                } else {
                    state_ack = START_ack;
                    printf("c to to start\n");
                }
                break;
            case BCC1_OK_ack:
                if(buf[0] == FLAG){
                    bcc2_value ^= counter;
                    if(bcc2_value == counter){
                        state_ack = STOP_ack;
                        printf("bcc1 to stop\n");
                        if(i_signal == 0x00){
                            rr_signal = 0xAB;
                        } else if(i_signal == 0x80){
                            rr_signal = 0xAA;
                        }
                    } else{
                        state_ack = STOP_ack;
                        printf("bcc1 to stop\n");
                        if(i_signal == 0x00){
                            rr_signal = 0x54;
                        } else if(i_signal == 0x80){
                            rr_signal = 0x55;
                        }
                    }
                } else {
                    if(!is_esc){
                        if(buf[0]==ESC){
                            is_esc = true;
                        }else{
                            counter = buf[0];
                            bcc2_value ^= buf[0];
                        }
                    }
                    else {
                        is_esc = false;
                        if (buf[0] == 0x5e){
                            bcc2_value ^= 0x7e;

                        } else if(buf[0] == 0x5d){
                            bcc2_value ^= 0x7d;
                        }
                    }
                }
                break;
            case STOP_ack:
                printf("FRAME acknowledged!\n");
            

                buf[0] = FLAG; 
                buf[1] = 0x01; // A
                buf[2] = rr_signal; // C
                buf[3] = 0x01 ^ rr_signal; // bcc1
                buf[4] = FLAG; 
                buf[5] = '\n';
                for (int i = 0; i < 1; i++)
                    printf("0x%02X ", buf[i]);
                int bytes = write(fd, buf, BUF_SIZE);
                printf("%d bytes written\n", bytes);
                printf("stop\n");
                //STOP = TRUE;
                general_state = I_RR;
                bcc2_value = 0x00;
                is_esc = false;
                i_signal = 0x12;
                rr_signal = 0x12;
                counter = ESC;
                state_ack = START_ack;
                break;
        }
    }

    return 0;
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

    // Open serial port device for reading and writing and not as controlling tty
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

    // Loop for input
    

    while (STOP == FALSE && general_state == SET_UA)
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, buf, 1);
        buf[bytes] = '\0'; // Set end of string to '\0', so we can printf

        switch(state){
            case START:
                if(buf[0] == FLAG){
                    state = FLAG_RCV;
                }
            break;
            case FLAG_RCV:
                if(buf[0] == A){
                    state = A_RCV;
                } else if (buf[0] != FLAG) {
                    state = START;
                }
                break;

            case A_RCV:
                if(buf[0] == C){
                    state = C_RCV;
                } else if (buf[0] == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;
            case C_RCV:
                if(buf[0] == (A ^ C)){
                    state = BCC_OK;
                } else if (buf[0] == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;
            case BCC_OK:
                if(buf[0] == FLAG){
                    state = STOP_;
                } else {
                    state = START;
                }
                break;
            case STOP_:
                printf("SET acknowledged!\n");
                printf(":%s:%d\n", buf, bytes);

                for (int i = 0; i < 5; i++)
                    printf("0x%02X ", buf[i]);

                buf[0] = FLAG; 
                buf[1] = 0x01; // A
                buf[2] = 0x07; // C
                buf[3] = 0x01 ^ 0x07; // bcc1
                buf[4] = FLAG; 
                buf[5] = '\n';
                int bytes = write(fd, buf, BUF_SIZE);
                printf("%d bytes written\n", bytes);

                STOP = TRUE;
                general_state = I_RR;
                break;
        }
    }

    llread();

    // The while() cycle should be changed in order to respect the specifications
    // of the protocol indicated in the Lab guide

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
