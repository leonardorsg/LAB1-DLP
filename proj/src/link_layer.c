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

volatile int STOP = FALSE;

enum State {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK};
enum State state = START;

int fdd = -1;
unsigned char bufc[BUF_SIZE + 1] = {0}; // +1: Save space for the final '\0' char
unsigned char curr_buf[BUF_SIZE + 1] = {0};
int curr_buf_size = 0;

enum State_ACK {START_ack, FLAG_RCV_ack, A_RCV_ack, C_RCV_ack, BCC1_OK_ack, STOP_ack};
enum State_ACK state_ack = START_ack;

unsigned char rr = 0x00;
unsigned char i_signal = 0x12;
unsigned char rr_signal = 0x12;

// renamed to i_value to avoid confusion with i for-loop variable
unsigned char i_value = 0x00; 

int alarmEnabled = FALSE;
int alarmCount = 0;

enum GeneralState {SET_UA, I_RR, DISC, END};
enum GeneralState general_state = SET_UA;


LinkLayerRole curr_role;

int changed_i = FALSE;

int n_tries;
int timeout;

void setOffAlarm(){
    alarm(0); // Disable alarm
    alarmEnabled = FALSE;
    alarmCount = 0;
}

void setAlarm(){
    alarm(timeout);
    alarmEnabled = TRUE;    
    printf("Alarm was set! alarmCount: %d\n", alarmCount);
}

void send_supervision_frame(){
    
    write(fdd, bufc, 5);
    printf("Supervision frame was written.\n");
    
    //setAlarm();

    if(general_state == END) return;
    
    unsigned char responseBuf[1];

    STOP = FALSE;
    state = START;
    unsigned char c = 0x00;
    
    printf("Waiting for RX response...\n");
    
    while (STOP == FALSE ){ 

        if(read(fdd, responseBuf, 1)>0){
            switch(state){
                case START:
                    if(responseBuf[0] == FLAG){
                        state = FLAG_RCV;
                        printf(" changed to FLAG_RCV\n");
                    } 
                break;
                case FLAG_RCV:
                    
                    if(responseBuf[0] == 0x01){
                        printf(" changed to A_RCV\n");
                        state = A_RCV;
                    } else if (responseBuf[0] != FLAG) {
                        state = START;
                    } 
                    break;

                case A_RCV:
                    if((responseBuf[0] == 0x07) || (responseBuf[0] == 0x0B)){
                        printf(" changed to C_RCV\n");
                        state = C_RCV;
                        c = responseBuf[0];
                    } else if (responseBuf[0] == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(responseBuf[0] == (0x01 ^ c)){
                        printf(" changed to BCC_OK\n");
                        state = BCC_OK;
                    } else if (responseBuf[0] == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        STOP = TRUE;
                    }
                    break;
                case BCC_OK:
                    if(responseBuf[0] == FLAG){
                        printf(" changed to STOP_\n");

                        if(general_state == SET_UA){

                            general_state = I_RR;
            
                            printf("\n Successfully opened SET UA connection with llopen() \n\n");
            
                        }else if(general_state == DISC){
                            printf("DISC received! \n");
                        
                            general_state = END;
                        }
                        setOffAlarm();            

                        STOP = TRUE;
                    } 
                    break;
                }
            } else {
            STOP = TRUE;
            printf("no response received\n");
        }
    }
        
    printf(" will return from send_supervision_frame. alarmenabled? %d\n", alarmEnabled);
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
 
    write(fdd, bufc, pos+2);
    
    if (alarmEnabled == FALSE) {
        setAlarm();
    }
    
    unsigned char responseBuf[1];
    state = START;
    STOP = FALSE;
    unsigned char c = 0x00;
    while (STOP == FALSE){
        if(read(fdd, responseBuf, 1)<0){

            switch(state){
                case START:
                    if(responseBuf[0] == FLAG){
                        state = FLAG_RCV;
                    } 
                break;
                case FLAG_RCV:
                    if(responseBuf[0] == 0x01){
                        state = A_RCV;
                    } else if (responseBuf[0] != FLAG) {
                        state = START;
                    }
                    break;

                case A_RCV:
                    if((responseBuf[0] == 0xAA) || (responseBuf[0] == 0xAB) || (responseBuf[0] == 0x54) || (responseBuf[0] == 0x55)){
                        state = C_RCV;
                        c = responseBuf[0];
                    } else if (responseBuf[0] == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case C_RCV:
                    if(responseBuf[0] == (0x01 ^ c)){
                        state = BCC_OK;
                    } else if (responseBuf[0] == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;
                case BCC_OK:
                    if(responseBuf[0] == FLAG){
                        STOP = TRUE;
                    } else {
                        state = START;
                    }
                    break;
            }
        } else {
            STOP = TRUE;
        }
    }


    switch (c) {
        case 0x54:
            rr = 0;
            setOffAlarm();
            printf("REJ0\n");
            break;
        case 0xAA:
            rr = 0;
            setOffAlarm();
            //printf("RR0\n");       
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
        return 0;
    } 
    return -1;
}

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);

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

    n_tries = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    
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
        bufc[4] = FLAG;
        
        send_supervision_frame();

        while (alarmCount < n_tries && general_state==SET_UA)
        {
            if (!alarmEnabled) {
                //printf("alarmCount: %d, alarm disabled, will send supervision frame. \n", alarmCount);
                setAlarm();
                //printf("alarmCount: %d, sent supervision frame in llopen. \n", alarmCount);
            } 
        }

        if(general_state == SET_UA) return -1;
        else return fdd;
    }

    if(connectionParameters.role == LlRx){
        curr_role = LlRx;

        while (STOP == FALSE && general_state == SET_UA)
        {
            read(fdd, bufc, 1);

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
                        printf("SET acknowledged!\n");
            

                        bufc[0] = FLAG; 
                        bufc[1] = 0x01; // A
                        bufc[2] = 0x07; // C
                        bufc[3] = 0x01 ^ 0x07; // bcc1
                        bufc[4] = FLAG; 

                        write(fdd, bufc, 5);
                        

                        STOP = TRUE;
                        general_state = I_RR;
                    } else {
                        state = START;
                    }
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
    
    // Copy value of buf and bufsize as a global variable, so that alarm handler can use it
    curr_buf_size = bufSize;
    for(int i=0; i < bufSize; i++){
        curr_buf[i] = buf[i];
    }
    
    changed_i = FALSE;

    while (alarmCount < n_tries && general_state == I_RR && changed_i==FALSE){
        if (!alarmEnabled){
            send_information(buf, bufSize);
        }
    }

    for(int i=0; i < bufSize; i++){
        curr_buf[i] = 0;
    }
    curr_buf_size = 0;

    if (alarmCount>=n_tries){
        printf("Exausted all attempts\n");
        return -1; 
    } 
    
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
    unsigned char counter = ESC;
    state_ack = START_ack;
    unsigned char aux_buf[BUF_SIZE + 1] = {0};
    int count = 0;

    while ((STOP == FALSE) && (general_state == I_RR)) {
        
        read(fdd, bufc, 1);

        switch(state_ack){
            case START_ack:
                if(bufc[0] == FLAG){
                    state_ack = FLAG_RCV_ack;
                }
                break;
            case FLAG_RCV_ack:
                if(bufc[0] == A){
                    state_ack  = A_RCV_ack;
                } else if (bufc[0] != FLAG) {
                    state_ack  = START_ack;
                }
                break;

            case A_RCV_ack:
                if((bufc[0] == 0x00) || (bufc[0] == 0x80)){
                    state_ack = C_RCV_ack;
                    i_signal = bufc[0];
                } else if (bufc[0] == FLAG) {
                    state_ack = FLAG_RCV_ack;
                } else {
                    state_ack = START_ack;
                }
                break;
            case C_RCV:
                if(bufc[0] == (A ^ i_signal)){
                    state_ack  = BCC1_OK_ack;
                } else if (bufc[0] == FLAG) {
                    state_ack  = FLAG_RCV_ack;
                } else {
                    state_ack = START_ack;
                }
                break;
            case BCC1_OK_ack:
                if(bufc[0] == FLAG){
                    bcc2_value ^= counter;
                    if(bcc2_value == counter){
                        state_ack = STOP_ack;

                        if (i_signal == 0x00) rr_signal = 0xAB;
                        else if (i_signal == 0x80) rr_signal = 0xAA;

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
                            count++;
                        }
                    }
                    else {
                        is_esc = FALSE;
                        if (bufc[0] == 0x5e){
                            bcc2_value ^= FLAG;
                            aux_buf[count] = FLAG;
                            count++;

                        } else if(bufc[0] == 0x5d){
                            bcc2_value ^= ESC;
                            aux_buf[count] = ESC;
                            count++;

                        }
                    }
                }
                break;
            case STOP_ack:

                bufc[0] = FLAG; 
                bufc[1] = 0x01; // A
                bufc[2] = rr_signal; // C
                bufc[3] = 0x01 ^ rr_signal; // bcc1
                bufc[4] = FLAG; 
 
                write(fdd, bufc, 5);
                
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
        // (void)signal(SIGALRM, alarmHandler);

        // Create string to send
        bufc[0] = FLAG; 
        bufc[1] = A;
        bufc[2] = 0x0B; // C
        bufc[3] = A ^ 0x0B; // bcc1
        bufc[4] = FLAG; 

        general_state = DISC;
        send_supervision_frame();
        
        while (alarmCount < n_tries && general_state==DISC)
        {
            if (!alarmEnabled)
            {
                setAlarm();
            }
        }

        if(general_state == END){

            bufc[0] = FLAG; 
            bufc[1] = A;
            bufc[2] = 0x07; // C
            bufc[3] = A ^ 0x07; // bcc1
            bufc[4] = FLAG; 

            send_supervision_frame();

            while (alarmCount < n_tries && general_state==DISC)
        {
            if (!alarmEnabled)
            {
                setAlarm();
            }
        }
        }
    } else if(curr_role == LlRx){
        general_state = DISC;
        state = START;
        STOP = FALSE;
        while (STOP == FALSE && general_state == DISC){
            read(fdd, bufc, 1);
            
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
                    if(bufc[0] == 0X0B){
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
                        printf("DISC acknowledged!\n");
                    
                        bufc[0] = FLAG; 
                        bufc[1] = 0x01; // A
                        bufc[2] = 0x0B; // C
                        bufc[3] = 0x01 ^ 0x0B; // bcc1
                        bufc[4] = FLAG; 
                        write(fdd, bufc, 5);

                        STOP = TRUE;
                        general_state = END;
                    } else {
                        state = START;
                    }
                    break;
            }
        }

        STOP = FALSE;
        state = START;

        while (STOP == FALSE && general_state == END){
            
            read(fdd, bufc, 1);

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
                    if(bufc[0] == 0x07){
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
                        printf("Final UA acknowledged!\n");
                        STOP = TRUE;
                    } else {
                        state = START;
                    }
                    break;
                
            }
        }
    }
    int clstat = closeSerialPort();

    return clstat;
}
