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
#include <unistd.h>
#include <sys/time.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 1024
#define FLAG 0x7E
#define ESC 0x7D

#define A 0x03
#define C 0x03

#define SET 0x03
#define DISC 0x0B
#define UA 0x07

volatile int STOP = FALSE;

typedef enum {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_} State;
typedef enum {SET_UA_STATE, I_RR_STATE, DISC_STATE, UA_STATE, END_STATE} GeneralState;

// RX VARIABLES
State rx_state = START;

struct timeval begin, end;


unsigned char i_signal = 0x12;
unsigned char rr_signal = 0x12;

unsigned char c = 0x00;

// Used in llread()
unsigned char bcc2_value;
unsigned char is_esc;
unsigned char counter;
unsigned char aux_buf[BUF_SIZE + 1];
int count;

// TX VARIABLES
GeneralState general_state = SET_UA_STATE;
State tx_state = START;

unsigned char rr = 0x00;
unsigned char i_value = 0x00; // renamed to i_value to avoid confusion with i for-loop variable
int changed_i = FALSE;
int changed_i_rcv = TRUE;

int alarmEnabled = FALSE;
int alarmCount = 0;

int llclose_retransmissions = 0;

int llclose_frames = 0;

int fdd = -1;
unsigned char bufc[BUF_SIZE + 1] = {0}; // +1: Save space for the final '\0' char
unsigned char curr_buf[BUF_SIZE + 1] = {0};
int curr_buf_size = 0;

int is_valid_packet = FALSE;

LinkLayer connectionParams;

LinkLayerRole curr_role;


void setAlarm(){
    alarm(connectionParams.timeout);
    alarmEnabled = TRUE;
}

void setOffAlarm(){
    alarm(0); 
    alarmEnabled = FALSE;
    alarmCount = 0;
}

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    //statistics variable
    llclose_retransmissions++;

    printf("Alarm #%d\n", alarmCount);  
}

// Track the number of frames sent
void increment_llclose_frames(){
    llclose_frames++;
}

//Function that implements state machine used to procress receptor supervision bytes
void processRXSupervisionByte(unsigned char supervisionByte){
    switch(tx_state){
        case START:
            if(supervisionByte == FLAG){
                tx_state = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if(supervisionByte == 0x01){
                tx_state = A_RCV;
            } else if (supervisionByte != FLAG) {
                tx_state = START;
            }
            break;

        case A_RCV:
            if ((general_state == SET_UA_STATE && supervisionByte == UA) || 
                (general_state == DISC_STATE && supervisionByte == DISC)){ 
                tx_state = C_RCV;
            } else if (supervisionByte == FLAG) {
                tx_state = FLAG_RCV;
            } else {
                tx_state = START;
            }
            break;
        case C_RCV:
            if((general_state == SET_UA_STATE && supervisionByte == (0x01^ UA)) ||
                (general_state == DISC_STATE && supervisionByte == (0x01^ DISC))){
                tx_state = BCC_OK;
            } else if (supervisionByte == FLAG) {
                tx_state = FLAG_RCV;
            } else {
                tx_state = START;
            }
            break;
        case BCC_OK:
            // printf("bcc ok received\n");
            if(supervisionByte == FLAG){
                tx_state = STOP_;

                if(general_state == SET_UA_STATE){
                    printf("UA received! \n\n");

                    general_state = I_RR_STATE;
                }else if(general_state == DISC_STATE){
                    printf("DISC received! \n\n");
                    general_state = UA_STATE;
                }

                setOffAlarm();

            } else {
                tx_state = START;
            }
        default:
            break;
    }
}

void sendSupervisionFrame(){
    
    write(fdd, bufc, 5);
    printf("Sent supervision frame.\n");
    increment_llclose_frames();
    if (!alarmEnabled) setAlarm();

}

void receiveSupervisionAnswer(){
    
    unsigned char responseBuf[1];
    
    printf("Waiting for RX response...\n");

    tx_state = START;
    while (alarmEnabled && tx_state != STOP_){
        if (read(fdd, responseBuf, 1)){
            processRXSupervisionByte(responseBuf[0]);
        } 
    }
    
}

//Function that implements state machine used to procress receptor information bytes
void processRXInformationByte(unsigned char informationByte){
    switch(tx_state){
        case START:
            if(informationByte == FLAG){
                tx_state = FLAG_RCV;
            }
        break;
        case FLAG_RCV:
            if(informationByte == 0x01){
                tx_state = A_RCV;
            } else if (informationByte != FLAG) {
                tx_state = START;
            }
            break;

        case A_RCV:
            if((informationByte == 0xAA) || (informationByte == 0xAB) || (informationByte == 0x54) || (informationByte == 0x55)){
                tx_state = C_RCV;
                c = informationByte;
            } else if (informationByte == FLAG) {
                tx_state = FLAG_RCV;
            } else {
                tx_state = START;
            }
            break;
        case C_RCV:
            if(informationByte == (0x01 ^ c)){
                tx_state = BCC_OK;
            } else if (informationByte == FLAG) {
                tx_state = FLAG_RCV;
            } else {
                tx_state = START;
            }
            break;
        case BCC_OK:
            // printf("bcc ok received\n");
            if(informationByte == FLAG){
                tx_state = STOP_;
            } else {
                tx_state = START;
            }
            break;
        default:
            break;
    }
}
    
void receiveInformationAnswer(){

    unsigned char responseBuf[1];

    tx_state = START;
    while (alarmEnabled && tx_state != STOP_){
        if (read(fdd, responseBuf, 1)){
            processRXInformationByte(responseBuf[0]);
        } 
    }

}

int send_information(const unsigned char *selectedFrame, int frameSize){

    c = 0x00;
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

    int bytesWritten = write(fdd, bufc, pos+2);
    increment_llclose_frames();
    if (alarmEnabled == FALSE) {
        setAlarm();
    }

    receiveInformationAnswer();

    switch (c) {
        case 0x54:
            rr = 0;
            setOffAlarm();
            llclose_retransmissions++;
            printf("REJ0. ");
            break;
        case 0xAA:
            rr = 0;
            setOffAlarm();
            // printf("RR0\n");       
            break;
        case 0x55:
            rr = 1;
            setOffAlarm();
            llclose_retransmissions++;
            printf("REJ1. "); 
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
    } 

    return bytesWritten;
}

//Function that implements state machine used to procress transmitor supervision bytes
void processTXSupervisionByte(unsigned char TXSupervisionByte){
    switch(rx_state){
        case START:
            if(TXSupervisionByte == FLAG){
                rx_state = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if(TXSupervisionByte == A){
                rx_state = A_RCV;
            } else if (TXSupervisionByte != FLAG) {
                rx_state = START;
            }
            break;
        case A_RCV:
            // Probably the receiver DISC ACK was lost, and the transmitter is sending a new DISC
            if (general_state == UA_STATE && TXSupervisionByte == DISC){
                printf("Receiving a late DISC signal. \n");
                general_state = DISC_STATE;
                rx_state = C_RCV;
            }
            else if((general_state == SET_UA_STATE && TXSupervisionByte == SET) || 
                (general_state == DISC_STATE && TXSupervisionByte == DISC) || 
                (general_state == UA_STATE && TXSupervisionByte == UA)){ 
                rx_state = C_RCV;
            } else if (TXSupervisionByte == FLAG) {
                rx_state = FLAG_RCV;
            } else {
                rx_state = START;
            }
            break;
        case C_RCV:
            if((general_state == SET_UA_STATE && TXSupervisionByte== (A ^ SET)) || 
                (general_state == DISC_STATE && TXSupervisionByte == (A ^ DISC)) || 
                (general_state == UA_STATE && TXSupervisionByte == (A ^ UA))){ 
                rx_state = BCC_OK;
            } else if (TXSupervisionByte == FLAG) {
                rx_state = FLAG_RCV;
            } else {
                rx_state = START;
            }
            break;
        case BCC_OK:
            if(TXSupervisionByte == FLAG){
                rx_state = STOP_;

                if(general_state == SET_UA_STATE){
                    printf("SET received!\n");
                    general_state = I_RR_STATE;
                        
                    bufc[0] = FLAG; 
                    bufc[1] = 0x01; // A
                    bufc[2] = UA; // C
                    bufc[3] = 0x01 ^ UA; // bcc1
                    bufc[4] = FLAG; 
                    
                    write(fdd, bufc, 5);
                    increment_llclose_frames();;

                } else if (general_state == DISC_STATE){
                    printf("DISC received!\n");
                    // Setting state to UA_STATE, waiting for UA
                    general_state = UA_STATE;

                    bufc[0] = FLAG; 
                    bufc[1] = 0x01; // A
                    bufc[2] = DISC; // C
                    bufc[3] = 0x01 ^ DISC; // bcc1
                    bufc[4] = FLAG; 

                    write(fdd, bufc, 5);
                    increment_llclose_frames();;
                } else if (general_state == UA_STATE){
                    printf("UA received!\n");
                    general_state = END_STATE;
                    increment_llclose_frames();;
                }
            } else {
                rx_state = START;
            }
            break;
        default:
            break;
    }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters){
    fdd = openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate);

    if (fdd < 0) return -1;

    gettimeofday(&begin, NULL);


    connectionParams = connectionParameters;

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

        while (alarmCount <= connectionParams.nRetransmissions && general_state==SET_UA_STATE) {
            if (!alarmEnabled) {
                sendSupervisionFrame();
                receiveSupervisionAnswer();
            }
        }

        //If the transmitor tried to retransmits supervision frame N times and is still in SET_UA_STATE, returns -1
        if(general_state == SET_UA_STATE) return -1;
        else return fdd;
    }

    if(connectionParameters.role == LlRx){
        curr_role = LlRx;

        while (general_state != I_RR_STATE){
            if (read(fdd, bufc, 1)){
                processTXSupervisionByte(bufc[0]);
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
    
    // Copy value of buf and bufsize as a global variable so that alarm handler can use it
    curr_buf_size = bufSize;
    for(int i=0; i < bufSize; i++){
        curr_buf[i] = buf[i];
    }
    
    changed_i = FALSE;
    int bytesWritten = 0;
    
    while (alarmCount <= connectionParams.nRetransmissions && general_state == I_RR_STATE && changed_i==FALSE){
        if (!alarmEnabled){        
           bytesWritten = send_information(buf, bufSize);
        }
    }

    for(int i=0; i < bufSize; i++){
        curr_buf[i] = 0;
    }
    curr_buf_size = 0;

    if (alarmCount>connectionParams.nRetransmissions){
        printf("Exausted all attempts\n");
        return -1; 
    } 
    
    return bytesWritten;
}

void processTXInformationByte(unsigned char TXInformationByte){
    switch(rx_state){
        case START:
            if(TXInformationByte == FLAG){
                rx_state = FLAG_RCV;
            }
            break;
        case FLAG_RCV:
            if(TXInformationByte == A){
                rx_state  = A_RCV;
            } else if (TXInformationByte != FLAG) {
                rx_state  = START;
            }
            break;

        case A_RCV:
            if((TXInformationByte == 0x00) || (TXInformationByte == 0x80)){
                rx_state = C_RCV;

                if (i_signal == TXInformationByte) changed_i_rcv = FALSE;
                else changed_i_rcv = TRUE;

                i_signal = TXInformationByte;
            } else if (TXInformationByte == FLAG) {
                rx_state = FLAG_RCV;
            } else {
                rx_state = START;
            }
            break;
        case C_RCV:
            if(TXInformationByte == (A ^ i_signal)){
                rx_state  = BCC_OK;
            } else if (TXInformationByte == FLAG) {
                rx_state  = FLAG_RCV;
            } else {
                rx_state = START;
            }
            break;
        case BCC_OK:
            if(TXInformationByte == FLAG){
                bcc2_value ^= counter;
                if(bcc2_value == counter){
                    rx_state = STOP_;

                    if (changed_i_rcv){
                        is_valid_packet = TRUE;
                    } else if ((rr_signal == 0x54) || (rr_signal == 0x55)){
                        // if the previous rr_signal was REJ and the i kept the same, then we want to write it
                        is_valid_packet = TRUE;
                    } else { 
                        // if the i did not change, and we accepted the previous one
                        is_valid_packet = FALSE;
                    }

                    if (i_signal == 0x00) rr_signal = 0xAB;
                    else if (i_signal == 0x80) rr_signal = 0xAA;

                    // printf("RIGHT BCC2 0X%02X, i_signal 0x%02X, so rr= 0x%02X\n", bcc2_value, i_signal, rr_signal);
                    
                } else{
                    rx_state = STOP_;
                    if (i_signal == 0x00) rr_signal = 0x54;
                    else if (i_signal == 0x80) rr_signal = 0x55;

                    is_valid_packet = FALSE;

                    printf("WRONG BCC2 %0x02X, i_signal 0x%02X, so rr= 0x%02X\n", bcc2_value, i_signal, rr_signal);
                }
            } else {
                if(!is_esc){
                    if(TXInformationByte==ESC){
                        is_esc = TRUE;
                    }else{
                        counter = TXInformationByte;
                        bcc2_value ^= TXInformationByte;
                        aux_buf[count] = TXInformationByte;
                        count++;
                    }
                }
                else {
                    is_esc = FALSE;
                    if (TXInformationByte == 0x5e){
                        bcc2_value ^= FLAG;
                        aux_buf[count] = FLAG;
                        count++;

                    } else if(TXInformationByte == 0x5d){
                        bcc2_value ^= ESC;
                        aux_buf[count] = ESC;
                        count++;

                    }
                }
            }
            break;
        default:
            break;
    }
}


////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet){
    bcc2_value = 0x00;
    is_esc = FALSE;
    counter = ESC;

    memset(aux_buf, 0, sizeof(aux_buf));
    count = 0;

    rx_state = START;
    while (rx_state != STOP_){ 
        if (read(fdd, bufc, 1)){
            processTXInformationByte(bufc[0]);
        } 
    }

    if (rx_state == STOP_){

        bufc[0] = FLAG; 
        bufc[1] = 0x01; // A
        bufc[2] = rr_signal; // C
        bufc[3] = 0x01 ^ rr_signal; // bcc1
        bufc[4] = FLAG; 

        write(fdd, bufc, 5);
        increment_llclose_frames();;
    }
    if(is_valid_packet){
        for(int i = 0; i < count-1;i++){
            packet[i] = aux_buf[i];
        }
        return count-1;
    } else {
        return -1;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {
    //TODO collect statistics
    if(curr_role == LlTx){
        // Set alarm function handler
        // (void)signal(SIGALRM, alarmHandler);

        // Create string to send
        bufc[0] = FLAG; 
        bufc[1] = A;
        bufc[2] = DISC; // C
        bufc[3] = A ^ DISC; // bcc1
        bufc[4] = FLAG; 

        printf("Sending DISC...\n");
        general_state = DISC_STATE;

        while (alarmCount <= connectionParams.nRetransmissions && general_state==DISC_STATE){
            if (!alarmEnabled){
                sendSupervisionFrame();
                receiveSupervisionAnswer();
            }
        }

        if (alarmCount>connectionParams.nRetransmissions){
            printf("Exausted all attempts\n");
            return -1; 
        } 

        bufc[0] = FLAG; 
        bufc[1] = A;
        bufc[2] = UA; // C
        bufc[3] = A ^ UA; // bcc1
        bufc[4] = FLAG; 

        printf("Sending UA...\n");
        sendSupervisionFrame();
        general_state = END_STATE;
        
    } else {
        general_state = DISC_STATE;
        rx_state = START;
        
        printf("Waiting for DISC...\n");
        
        while (general_state!=UA_STATE){
            if (read(fdd, bufc, 1)){
                processTXSupervisionByte(bufc[0]);
            }     
        }

        printf("Waiting for UA...\n");
        rx_state = START;
        while (general_state!=END_STATE){
            if (read(fdd, bufc, 1)){
                processTXSupervisionByte(bufc[0]);
            }
        }
    }


    if(showStatistics){
        gettimeofday(&end, NULL);
        if(curr_role == LlTx){
            printf("\n==================== Transmitter Statistics ====================\n");
            printf("Total number of frames sent            : %d\n", llclose_frames);
            printf("Total number of retransmissions        : %d\n", llclose_retransmissions);
            printf("FER                                    : %.2f%%\n", (double) llclose_retransmissions / llclose_frames);

            
        } else {
            printf("\n==================== Receiver Statistics ====================\n");
            printf("Total number of frames received        : %d\n", llclose_frames);
            
        }
        double tframe = (double) BUF_SIZE*8 / connectionParams.baudRate;
        printf("Time Frame                             : %.6f\n", tframe);
        
        long time = end.tv_sec - begin.tv_sec;
        printf("Total time                             : %ld seconds\n", time);
        
        // calculated baudrate = filesize (bits) / time (s)
        double calculated_baudrate = (double) llclose_frames * 8 * BUF_SIZE / time;
        printf("Calculated Baudrate                    : %d\n", (int) calculated_baudrate);
        printf("Nominal Baudrate                       : %d\n", connectionParams.baudRate);
        printf("Efficency                             : %.6f\n", (double) (calculated_baudrate / connectionParams.baudRate));
        printf("=================================================================\n\n");
    }
    sleep(3);
    int clstat = closeSerialPort();

    return clstat;
}
