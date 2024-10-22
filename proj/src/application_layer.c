// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_FRAME_SIZE 1024

unsigned char *readFile(const char *filename, size_t *fileSize) {
    FILE *file = fopen(filename, "rb"); // Open file in binary mode
    if (!file) {
        perror("Error opening file");
        return NULL;
    }

    // Obtain file size
    fseek(file, 0, SEEK_END);
    *fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    // Allocate memory for the file content
    unsigned char *buffer = (unsigned char *)malloc(*fileSize);
    if (!buffer) {
        perror("Error allocating memory");
        fclose(file);
        return NULL;
    }

    // Read file content into buffer
    fread(buffer, 1, *fileSize, file);
    fclose(file);

    return buffer;
}

// Function to send frames to the receiver
void sendFrames(unsigned char *fileData, size_t fileSize) {
    
    int length = 0;
    int currentFileSize = fileSize;
    int sequence_number;

    unsigned char controlPacket[MAX_FRAME_SIZE + 4]; 
    controlPacket[0] = 0x01;
    controlPacket[1] = 0x00; 

    // Calculate the length of the file in bytes
    while (currentFileSize > 0)
    {
        int rest = currentFileSize % 256;
        int div = currentFileSize / 256;
        length++;

        // Reorganize the control packet, shifting to the right
        for (unsigned int i = 2 + length; i > 3; i--)
            controlPacket[i] = controlPacket[i - 1];

        controlPacket[3] = (unsigned char)rest;

        currentFileSize = div;
    }
    
    
    controlPacket[2] = (unsigned char)length;

    int controlPacketSize = 3 + length;

    llwrite(controlPacket, controlPacketSize);
    
    size_t bytesSent = 0;
    while (bytesSent < fileSize) {
        // Calculate the size of the frame to send
        size_t frameSize = (fileSize - bytesSent > MAX_FRAME_SIZE) ? MAX_FRAME_SIZE : (fileSize - bytesSent);

        // Copy the frame data from the file data
        unsigned char frame[MAX_FRAME_SIZE];
        frame[0] = 0x10; //C = 2 -> send data
        sequence_number = (sequence_number + 1) % 100;
        frame[1] = (unsigned char)sequence_number;
        int l1 = frameSize % 256;
        int l2 = frameSize / 256;
        frame[2] = (unsigned char)l1;
        frame[3] = (unsigned char)l2;

        //ta certo isso??
        for (size_t i = 0; i < frameSize; i++) {
            frame[i+4] = fileData[bytesSent + i];
        }

        printf("Transmitting frame: %zu bytes\n", frameSize);
        llwrite(frame, frameSize+4);

        // Update the number of bytes sent
        bytesSent += frameSize;
    }


    //Send end control packet
    controlPacket[0] = 0x11;
    
    llwrite(controlPacket, controlPacketSize);

}

void receiveFrames(const char *filename) {
    FILE *file = fopen(filename, "wb"); // Open file in binary mode
    if (!file) {
        perror("Error opening file");
        return;
    }

    unsigned char frame[MAX_FRAME_SIZE];
    size_t totalSize = 0;
    int fileSize = 0;
    int receiving = TRUE;
    int expectedSequenceNumber = 0;

    while (receiving) {
        int bytesRead = llread(frame);
        if (bytesRead < 0) {
            printf("Error reading frame\n");
            return;
        } 
        //talvez nao precise deste elif
        else if (bytesRead == 0) {
            printf("End of transmission\n");
            receiving = FALSE;
            break;
        } else { //
            if(frame[0] == 0x01){
                int length = (int)frame[2];


                for (int i = 0; i < length; i++)
                {
                    fileSize = fileSize * 256 + (int)frame[3 + i];
                }
            } else if(frame[0] == 0x10){
                int sequenceNumber = (int)frame[1];
                if(sequenceNumber != expectedSequenceNumber) {
                    printf("Unexpected sequence number\n");
                    break;
                } else {
                    expectedSequenceNumber = (expectedSequenceNumber + 1) % 100;
                    int l1 = (int)frame[2];
                    int l2 = (int)frame[3];
                    size_t frameSize = 256 * l2 + l1;

                    unsigned char data[MAX_FRAME_SIZE];

                    for(int i = 0; i < frameSize; i++){
                        data[i] = frame[i + 4];
                    }

                    int dataLength = bytesRead - 4;

                    if (fwrite(data, sizeof(unsigned char), dataLength, filename) != dataLength)
                    {
                        return -1;
                    }
                }

            } else if (frame[0] == 0x11){
                receiving = FALSE;
            }

        }

        totalSize += bytesRead ;
    }


    if(fileSize == totalSize) printf("Sucessful read! All bytes were read.\n");

    printf("Received %zu bytes\n", totalSize);
}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole this_role;

    if(strcmp("tx", role) == 0) 
        this_role = LlTx;
    else if(strcmp("rx", role) == 0)
        this_role = LlRx;
    
    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.role = this_role;
    connectionParameters.timeout = timeout;
    strcpy(connectionParameters.serialPort, serialPort);

    // *connectionParameters.serialPort = serialPort;

    if(llopen(connectionParameters) < 0) printf("Error in open\n");
    
    if (this_role == LlTx) {
        size_t fileSize;
        unsigned char *fileData = readFile(filename, &fileSize);

        if (!fileData) return 1; // Error reading file

        sendFrames(fileData, fileSize);
        
        free(fileData);
    } else {
        receiveFrames(filename);
    }

    if(llclose(1) < 0) printf("Error in close\n");

}
