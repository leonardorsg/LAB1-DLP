// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

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
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 256
#define FLAG 0x7E
#define ESC 0x7D
#define A 0x03
#define C 0x03

volatile int STOP = FALSE;

enum State {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_};
enum State state = START;

int fd = -1;
unsigned char bufc[BUF_SIZE + 1] = {0}; // +1: Save space for the final '\0' char

unsigned char i_signal = 0x12;
unsigned char rr_signal = 0x12;
unsigned char counter = ESC;

enum State_ACK {START_ack, FLAG_RCV_ack, A_RCV_ack, C_RCV_ack, BCC1_OK_ack, STOP_ack};
enum State_ACK state_ack = START_ack;

unsigned char rr = 0x00;

// renamed to i_value to avoid confusion with i for-loop variable
unsigned char i_value = 0x00; 

int alarmEnabled = FALSE;
int alarmCount = 0;

enum GeneralState {SET_UA, I_RR, DISC};
enum GeneralState general_state = SET_UA;

unsigned char frame0[] = {0x01,0x04,0x29};
unsigned char frame1[] = {0x02,0x04,0x29};

unsigned char *frames[] = {frame0, frame1};
size_t frame_sizes[] = {sizeof(frame0), sizeof(frame1)};


void send_set(){
    
    int bytes = write(fd, bufc, BUF_SIZE);
    printf("%d bytes written\n", bytes);

    // Wait until all bytes have been written to the serial port
    
    unsigned char responseBuf[BUF_SIZE];
    int responseBytes = read(fd, responseBuf, BUF_SIZE);

    if (responseBytes > 0) {
        unsigned char a = bufc[1];
        unsigned char c = bufc[2];
        unsigned char check = a ^ c;
        unsigned char bcc = bufc[3];

        if(bcc != check){
            printf(" A ^ C != BCC1\n");
            printf(":%s:%d\n", bufc, bytes);

            for (int i = 0; i < 5; i++) 
                printf("0x%02X \n", responseBuf[i]);
            
        } else {
            printf("UA received!\n");
            printf(":%s:%d\n", bufc, bytes);

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

void setOffAlarm(){
    alarm(0); // Disable alarm
    alarmEnabled = FALSE;
    alarmCount = 0;
}

void send_information(const unsigned char *selectedFrame, int frameSize){

    // Create string to send
    bufc[0] = FLAG; 
    bufc[1] = A; 

    // C

    if (i_value)
        bufc[2] = 0x80;
    else
        bufc[2] = 0x00;

    bufc[3] = A ^ bufc[2]; // bcc1

    unsigned char bcc2 = 0x00;
    
    //unsigned char *selectedFrame = frames[i_value];
    //size_t frameSize = frame_sizes[i_value]; 
    
    size_t pos = 4;

    for (int j = 0; j < frameSize; j++, pos++) {

        unsigned char value = selectedFrame[j];

        if (value == FLAG){
            bufc[pos] = ESC;
            bufc[++pos] = 0x5E;
        } else if (value == ESC){
            bufc[pos] = ESC;
            bufc[++pos] = 0x5D;
        } else
            bufc[pos] = selectedFrame[j]; 

        bcc2 ^= selectedFrame[j];
    }

// frame0 = {0x01,0x04,0x29};
// frame1 = {0x02,0x04,0x29};

    bufc[pos] = bcc2;
    bufc[pos+1] = FLAG;

    printf("buf to send: \n");
    for (int i = 0; i < frameSize + 6; i++) 
        printf("buf[%d]: 0x%02X \n", i, bufc[i]);

    int bytes = write(fd, bufc, BUF_SIZE);
    printf("%d bytes written during llwrite\n", bytes);

    if (alarmEnabled == FALSE) {
        printf("Setting alarm, alarmCount = %d\n", alarmCount);
        alarm(3); // Set alarm to be triggered in 3s
        alarmEnabled = TRUE;
    }
    
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
                    setOffAlarm();
                    printf("REJ0\n");
                    break;
                case 0xAA:
                    rr = 0;
                    setOffAlarm();
                    printf("RR0\n");     
                    break;
                case 0x55:
                    rr = 1;
                    setOffAlarm();
                    printf("REJ1\n");
                    break;
                case 0xAB:
                    rr = 1;
                    setOffAlarm();
                    printf("RR1\n");      
                    break;
                default:
                    break;
            }

            if (rr != i_value){  // If the received RR is different from the sent I,
                i_value = !i_value;  // change the I value
                printf("just changed i_value to %d\n", i_value);

            } 
            // else    // If the received RR is the same as the sent I, we keep i_value
            //     printf("Keeping i = %d, alarmCount = %d\n", i_value, alarmCount);

            // by default, the next I is the same as the previous one
            printf("next i = %d\n", i_value); 
            
        }
    } else {
        printf("No response received,  %d  alarmCount = %d \n", i_value, alarmCount);
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
            send_set();
            break;
        case I_RR:    
            printf("Frame not received, retransmitting i = %d\n", i_value);
            llwrite();
            break;
        default:
            printf("Invalid general state, %d\n", general_state);

            // TODO: put it in the end
            alarm(0); // Disable alarm 
            break;
    } 
    
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    fd = openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate);
    if (fd < 0)
    {
        return -1;
    }

    printf("New termios structure set\n");

    if(connectionParameters.role == LlTx){
        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);

        // Create string to send
        bufc[0] = FLAG; 
        bufc[1] = A;
        bufc[2] = 0x03; // C
        bufc[3] = A ^ 0x03; // bcc1
        // buf[3]=0xFF;
        bufc[4] = FLAG; 
        bufc[5] = '\n';

        send_set();

        while (alarmCount < 3 && general_state==SET_UA)
        {
            if (!alarmEnabled)
            {
                alarm(3); // Set alarm to be triggered in 3s
                alarmEnabled = TRUE;
            }
        }


        // TODO

        if(general_state == SET_UA) return -1;
        else
            return fd;
    }

    if(connectionParameters.role == LlRx){

    while (STOP == FALSE && general_state == SET_UA)
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, bufc, 1);
        bufc[bytes] = '\0'; // Set end of string to '\0', so we can printf

        switch(state){
            case START:
                if(bufc[0] == FLAG){
                    state = FLAG_RCV;
                }
            break;
            case FLAG_RCV:
                if(bufc[0] == A){
                    state = A_RCV;
                } else if (bufc[0] != FLAG) {
                    state = START;
                }
                break;

            case A_RCV:
                if(bufc[0] == C){
                    state = C_RCV;
                } else if (bufc[0] == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;
            case C_RCV:
                if(bufc[0] == (A ^ C)){
                    state = BCC_OK;
                } else if (bufc[0] == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;
            case BCC_OK:
                if(bufc[0] == FLAG){
                    state = STOP_;
                } else {
                    state = START;
                }
                break;
            case STOP_:
                printf("SET acknowledged!\n");
                printf(":%s:%d\n", bufc, bytes);

                for (int i = 0; i < 5; i++)
                    printf("0x%02X ", bufc[i]);

                bufc[0] = FLAG; 
                bufc[1] = 0x01; // A
                bufc[2] = 0x07; // C
                bufc[3] = 0x01 ^ 0x07; // bcc1
                bufc[4] = FLAG; 
                bufc[5] = '\n';
                int bytes = write(fd, bufc, BUF_SIZE);
                printf("%d bytes written\n", bytes);

                STOP = TRUE;
                general_state = I_RR;
                break;
        }
    }
    return fd;
    }
}




////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    //copy value of buf and bufsize as a global variable so that alarm handler can use it
    while (alarmCount < 3 && general_state == I_RR){
        if (!alarmEnabled){
            printf("Alarm disabled, writing frame %d\n", i_value);
            send_information(buf, bufSize);
        }
    }

    //send bytes written is success and -1 if failure
    printf("Exausted all attempts\n");

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    STOP = FALSE;
    unsigned char bcc2_value = 0x00;
    unsigned char is_esc = FALSE;
    bcc2_value = 0x00;
    is_esc = FALSE;
    i_signal = 0x12;
    rr_signal = 0x12;
    counter = ESC;
    state_ack = START_ack;

    while ((STOP == FALSE) && (general_state == I_RR))
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, bufc, 1);
        bufc[bytes] = '\0'; // Set end of string to '\0', so we can printf
        switch(state_ack){
            case START_ack:
                if(bufc[0] == FLAG){
                    state_ack = FLAG_RCV_ack;
                }
                //printf("start");
            break;
            case FLAG_RCV_ack:
                if(bufc[0] == A){
                    state_ack  = A_RCV_ack;
                    printf("flag to a\n");
                } else if (bufc[0] != FLAG) {
                    state_ack  = START_ack;
                    printf("flag to start\n");
                }
                break;

            case A_RCV_ack:
                if((bufc[0] == 0x00) || (bufc[0] == 0x80)){
                    state_ack = C_RCV_ack;
                    i_signal = bufc[0];
                    printf("A_RCV_ack to C_RCV_ack, i_signal = %x\n", i_signal);
                } else if (bufc[0] == FLAG) {
                    state_ack = FLAG_RCV_ack;
                    printf("a to flag t\n");
                } else {
                    state_ack = START_ack;
                    printf("a to start\n");
                }
                break;
            case C_RCV:
                if(bufc[0] == (A ^ i_signal)){
                    state_ack  = BCC1_OK_ack;
                    printf("c to bcc1\n");
                } else if (bufc[0] == FLAG) {
                    state_ack  = FLAG_RCV_ack;
                    printf("c to flag\n");
                } else {
                    state_ack = START_ack;
                    printf("c to to start\n");
                }
                break;
            case BCC1_OK_ack:
                if(bufc[0] == FLAG){
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
                        if(bufc[0]==ESC){
                            is_esc = TRUE;
                        }else{
                            counter = bufc[0];
                            bcc2_value ^= bufc[0];
                        }
                    }
                    else {
                        is_esc = FALSE;
                        if (bufc[0] == 0x5e){
                            bcc2_value ^= 0x7e;

                        } else if(bufc[0] == 0x5d){
                            bcc2_value ^= 0x7d;
                        }
                    }
                }
                break;
            case STOP_ack:
                printf("FRAME acknowledged!\n");
            

                bufc[0] = FLAG; 
                bufc[1] = 0x01; // A
                bufc[2] = rr_signal; // C
                bufc[3] = 0x01 ^ rr_signal; // bcc1
                bufc[4] = FLAG; 
                bufc[5] = '\n';
                for (int i = 0; i < 1; i++)
                    printf("0x%02X ", bufc[i]);
                int bytes = write(fd, bufc, BUF_SIZE);
                printf("%d bytes written\n", bytes);
                printf("stop\n");
                STOP = TRUE;
                general_state = DISC;
                break;
        }
    }

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
