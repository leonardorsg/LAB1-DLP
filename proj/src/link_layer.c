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

int fdd = -1;
unsigned char bufc[BUF_SIZE + 1] = {0}; // +1: Save space for the final '\0' char
unsigned char curr_buf[BUF_SIZE + 1] = {0};
int curr_buf_size = 0;

enum State_ACK {START_ack, FLAG_RCV_ack, A_RCV_ack, C_RCV_ack, BCC1_OK_ack, STOP_ack};
enum State_ACK state_ack = START_ack;

unsigned char rr = 0x00;

// renamed to i_value to avoid confusion with i for-loop variable
unsigned char i_value = 0x00; 

int alarmEnabled = FALSE;
int alarmCount = 0;

enum GeneralState {SET_UA, I_RR, DISC, END};
enum GeneralState general_state = SET_UA;

unsigned char frame0[] = {0x01,0x04,0x29};
unsigned char frame1[] = {0x02,0x04,0x29};

unsigned char *frames[] = {frame0, frame1};
size_t frame_sizes[] = {sizeof(frame0), sizeof(frame1)};

LinkLayerRole curr_role;

int changed_i = FALSE;

void send_supervision_frame(){
    
    int bytes = write(fdd, bufc, 5);
    printf("%d bytes written in supervision frame\n", bytes);

    if(general_state == END) return;

    // Wait until all bytes have been written to the serial port
    
    unsigned char responseBuf[5];

    printf("Waiting for RX response...\n");
    int responseBytes = read(fdd, responseBuf, 5);
    
    if (responseBytes > 0) {
        unsigned char a = responseBuf[1];
        unsigned char c = responseBuf[2];
        unsigned char check = a ^ c;
        unsigned char bcc = responseBuf[3];

        if(bcc != check){
            printf(" A ^ C != BCC1\n");
            printf("responseBytes %d: ", responseBytes);

            for (int i = 0; i < 5; i++) 
                printf("0x%02X ", responseBuf[i]);
            printf("\n");

        } else {
            if(general_state == SET_UA){
                printf("UA received! %d responseBytes: ", responseBytes);

                for (int i = 0; i < 5; i++) 
                    printf("0x%02X ", responseBuf[i]);
                printf("\n");

                general_state = I_RR;
    
                printf("\n Successfully opened SET UA connection with llopen() \n\n");
    
            }else if(general_state == DISC){
                printf("DISC received! (%d bytes) \n", responseBytes);

                for (int i = 0; i < 5; i++) 
                    printf("0x%02X \n", responseBuf[i]);
                
                general_state = END;
            }
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

int send_information(const unsigned char *selectedFrame, int frameSize){

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

    bufc[pos] = bcc2;
    bufc[pos+1] = FLAG;

    int bytes = write(fdd, bufc, pos+2);
    // printf("%d bytes written\n", bytes);
    
    if (alarmEnabled == FALSE) {
        // printf("Setting alarm, alarmCount = %d\n", alarmCount);
        alarm(3); // Set alarm to be triggered 
        alarmEnabled = TRUE;
    }
    
    // Wait until all bytes have been written to the serial port
    unsigned char responseBuf[5];
    int responseBytes = read(fdd, responseBuf, 5);
    
    if (responseBytes > 0) {
        unsigned char a = responseBuf[1];
        unsigned char c = responseBuf[2];
        unsigned char check = a ^ c;
        unsigned char bcc = responseBuf[3];

        if(bcc != check){
            printf(" A ^ C != BCC1\n");
            
        } else {
            // printf(" bcc==check! \n");

            if (c!=0x00){
                // printf("We just read %d bytes: ", responseBytes);
                // for (int i = 0; i < 6; i++) 
                //     printf("[%d]: 0x%02X ", i, responseBuf[i]);
                // printf("\n");
            } else
                printf("empty C field... :(\n");

            switch (c) {
                case 0x54:
                    rr = 0;
                    setOffAlarm();
                    printf("REJ0\n");
                    break;
                case 0xAA:
                    rr = 0;
                    setOffAlarm();
                    // printf("RR0\n");     
                    break;
                case 0x55:
                    rr = 1;
                    setOffAlarm();
                    printf("REJ1\n");
                    break;
                case 0xAB:
                    rr = 1;
                    setOffAlarm();
                    // printf("RR1\n");      
                    break;
                default:
                    break;
            }

            if (rr != i_value){  // If the received RR is different from the sent I,
                i_value = !i_value;  // change the I value
                changed_i = TRUE;
                // printf("Switching i (%d)\n", i_value);
                return 0;
            } 
            // else printf("kept i (%d)\n\n", i_value);    
        }
    } 
    // else {
    //     printf("No response received (i=%d)\n", i_value);
    // }
    return 1;
}

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);

    // printf("General state: %d\n", general_state);

    switch (general_state){
        case SET_UA:
            send_supervision_frame();
            break;
        case I_RR:    
            send_information(curr_buf, curr_buf_size);
            break;
        case DISC:
            send_supervision_frame();
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
    fdd = openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate);

    if (fdd < 0) return -1;

    printf("New termios structure set\n");

    if(connectionParameters.role == LlTx){
        curr_role = LlTx;
        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);
        // Create string to send
        bufc[0] = FLAG; 
        bufc[1] = A;
        bufc[2] = 0x03; // C
        bufc[3] = A ^ 0x03; // bcc1
        // buf[3]=0xFF;
        bufc[4] = FLAG; 
        // bufc[5] = '\n';

        send_supervision_frame();

        while (alarmCount < 3 && general_state==SET_UA)
        {
            if (!alarmEnabled)
            {
                alarm(3); // Set alarm to be triggered 
                alarmEnabled = TRUE;
            }
        }

        // TODO

        if(general_state == SET_UA) return -1;
        else
            return fdd;
    }

    if(connectionParameters.role == LlRx){
        curr_role = LlRx;

        while (STOP == FALSE && general_state == SET_UA)
        {
            int bytes = read(fdd, bufc, 1);
            // bufc[bytes] = '\0'; // Set end of string to '\0', so we can printf

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
                    printf("bcc ok received\n");
                    if(bufc[0] == FLAG){
                        state = STOP_;
                    } else {
                        state = START;
                    }
                    break;
                case STOP_:

                    printf("SET acknowledged!\n");
            

                    bufc[0] = FLAG; 
                    bufc[1] = 0x01; // A
                    bufc[2] = 0x07; // C
                    bufc[3] = 0x01 ^ 0x07; // bcc1
                    bufc[4] = FLAG; 
                    // bufc[5] = '\n';

                    int bytes = write(fdd, bufc, 5);
                    printf("Sent UA, %d bytes written: ", bytes);
                    for (int i = 0; i < 5; i++)
                        printf("0x%02X ", bufc[i]);
                    printf("\n");

                    STOP = TRUE;
                    general_state = I_RR;
                    break;
            }
        }
    }
    
    return fdd;
}




////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);
    
    //copy value of buf and bufsize as a global variable so that alarm handler can use it
    curr_buf_size = bufSize;
    for(int i=0; i < bufSize; i++){
        curr_buf[i] = buf[i];
    }
    
    changed_i = FALSE;

    // printf("general state: %d\n", general_state);
    
    while (alarmCount < 3 && general_state == I_RR && changed_i==FALSE){
        if (!alarmEnabled){
            // printf("Alarm disabled (i=%d)\n", i_value);            
            send_information(buf, bufSize);
        }
    }

    for(int i=0; i < bufSize; i++){
        curr_buf[i] = 0;
    }
    curr_buf_size = 0;

    if (alarmCount>=3){
        printf("Exausted all attempts\n");
        return -1; 
    } 
    
    // printf("llwrite finished\n");
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
    unsigned char i_signal = 0x12;
    unsigned char rr_signal = 0x12;
    unsigned char counter = ESC;
    state_ack = START_ack;
    unsigned char aux_buf[BUF_SIZE + 1] = {0};
    int count = 0;

    while ((STOP == FALSE) && (general_state == I_RR)) {
        int bytes = read(fdd, bufc, 1);
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
                    // printf("flag to a\n");
                } else if (bufc[0] != FLAG) {
                    state_ack  = START_ack;
                    // printf("flag to start\n");
                }
                break;

            case A_RCV_ack:
                if((bufc[0] == 0x00) || (bufc[0] == 0x80)){
                    state_ack = C_RCV_ack;
                    i_signal = bufc[0];
                } else if (bufc[0] == FLAG) {
                    state_ack = FLAG_RCV_ack;
                    // printf("a to flag t\n");
                } else {
                    state_ack = START_ack;
                    // printf("a to start\n");
                }
                break;
            case C_RCV:
                if(bufc[0] == (A ^ i_signal)){
                    state_ack  = BCC1_OK_ack;
                    // printf("c to bcc1\n");
                } else if (bufc[0] == FLAG) {
                    state_ack  = FLAG_RCV_ack;
                    // printf("c to flag\n");
                } else {
                    state_ack = START_ack;
                    // printf("c to to start\n");
                }
                break;
            case BCC1_OK_ack:
                if(bufc[0] == FLAG){
                    bcc2_value ^= counter;
                    if(bcc2_value == counter){
                        state_ack = STOP_ack;

                        if (i_signal == 0x00) rr_signal = 0xAB;
                        else if (i_signal == 0x80) rr_signal = 0xAA;

                        // printf("RIGHT BCC2 0X%02X, i_signal 0x%02X, so rr= 0x%02X\n", bcc2_value, i_signal, rr_signal);
                    } else{
                        state_ack = STOP_ack;
                        if (i_signal == 0x00) rr_signal = 0x54;
                        else if (i_signal == 0x80) rr_signal = 0x55;

                        printf("WRONG BCC2, i_signal 0x%02X, so rr= 0x%02X\n", i_signal, rr_signal);
                    }
                } else {
                    if(!is_esc){
                        if(bufc[0]==ESC){
                            is_esc = TRUE;
                        }else{
                            counter = bufc[0];
                            bcc2_value ^= bufc[0];
                            aux_buf[count] = bufc[0];
                            // printf("auxbuf[%d] = 0x%02X\n", count, aux_buf[count]);
                            count++;
                        }
                    }
                    else {
                        is_esc = FALSE;
                        if (bufc[0] == 0x5e){
                            bcc2_value ^= FLAG;
                            aux_buf[count] = FLAG;
                            // printf(" FLAG auxbuf[%d] = 0x%02X\n", count, aux_buf[count]);
                            count++;

                        } else if(bufc[0] == 0x5d){
                            bcc2_value ^= ESC;
                            aux_buf[count] = ESC;
                            // printf("ESC auxbuf[%d] = 0x%02X\n", count, aux_buf[count]);
                            count++;

                        }
                    }
                }
                break;
            case STOP_ack:
                // printf("FRAME acknowledged!\n");

                bufc[0] = FLAG; 
                bufc[1] = 0x01; // A
                bufc[2] = rr_signal; // C
                bufc[3] = 0x01 ^ rr_signal; // bcc1
                bufc[4] = FLAG; 
                // bufc[5] = '\n'; // nao sei bem pra que serviria isso
 
                int bytes = write(fdd, bufc, 5);
                
                STOP = TRUE;
                break;
        }
    }

    for(int i = 0; i < count-1;i++){
        packet[i] = aux_buf[i];
    }

    return count-1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    //TODO collect statistics
    if(curr_role == LlTx){
        // Set alarm function handler
        (void)signal(SIGALRM, alarmHandler);
        // Create string to send
        bufc[0] = FLAG; 
        bufc[1] = A;
        bufc[2] = 0x0B; // C
        bufc[3] = A ^ 0x0B; // bcc1
        // buf[3]=0xFF;
        bufc[4] = FLAG; 
        // bufc[5] = '\n';

        send_supervision_frame();
        general_state = DISC;

        while (alarmCount < 3 && general_state==DISC)
        {
            if (!alarmEnabled)
            {
                alarm(3); // Set alarm to be triggered 
                alarmEnabled = TRUE;
            }
        }

        bufc[0] = FLAG; 
        bufc[1] = A;
        bufc[2] = 0x07; // C
        bufc[3] = A ^ 0x07; // bcc1
        // buf[3]=0xFF;
        bufc[4] = FLAG; 
        // bufc[5] = '\n';

        send_supervision_frame();

    } else if(curr_role == LlRx){
        general_state = DISC;
        while (STOP == FALSE && general_state == DISC){
            int bytes = read(fdd, bufc, 1);
            // bufc[bytes] = '\0'; // Set end of string to '\0', so we can printf

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
                    if(bufc[0] == (A ^ 0x0B)){
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
                    printf("DISC acknowledged! ");
                    printf("%d\n", bytes);

                    for (int i = 0; i < 5; i++)
                        printf("0x%02X ", bufc[i]);

                    bufc[0] = FLAG; 
                    bufc[1] = 0x01; // A
                    bufc[2] = 0x0B; // C
                    bufc[3] = 0x01 ^ 0x0B; // bcc1
                    bufc[4] = FLAG; 
                    // bufc[5] = '\n';
                    int bytes = write(fdd, bufc, 5);

                    STOP = TRUE;
                    general_state = END;
                    break;
            }
        }

        STOP = TRUE;
        state = START;

        while (STOP == FALSE && general_state == END){
            // Returns after 5 chars have been input
            int bytes = read(fdd, bufc, 1);
            // bufc[bytes] = '\0'; // Set end of string to '\0', so we can printf

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
                    if(bufc[0] == (A ^ 0x07)){
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
                    printf("final UA acknowledged! ");
                    printf("%d\n", bytes);

                    for (int i = 0; i < 5; i++)
                        printf("0x%02X ", bufc[i]);

                    STOP = TRUE;
                    break;
            }
        }
        }
    int clstat = closeSerialPort();
    return clstat;
}
