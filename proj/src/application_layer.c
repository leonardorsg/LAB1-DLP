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

    unsigned char controlPacket[MAX_FRAME_SIZE + 4]; 
    controlPacket[0] = 0x01;
    controlPacket[1] = 0x00; 

    // Calculate the length of the control packet in bytes
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
        for (size_t i = 0; i < frameSize; i++) {
            frame[i] = fileData[bytesSent + i];
        }

        printf("Transmitting frame: %zu bytes\n", frameSize);
        llwrite(frame, frameSize);

        // Update the number of bytes sent
        bytesSent += frameSize;
    }
}

void receiveFrames(const char *filename) {
    FILE *file = fopen(filename, "wb"); // Open file in binary mode
    if (!file) {
        perror("Error opening file");
        return;
    }

    unsigned char frame[MAX_FRAME_SIZE];
    size_t frameSize;
    bool receiving = true;

    while (receiving) {
        int bytesRead = llread(frame);
        if (bytesRead < 0) {
            printf("Error reading frame\n");
            return;
        } else if (bytesRead == 0) {
            printf("End of transmission\n");
            break;
        } else { //
            int fileSize = frame[3];

        }

        frameSize += bytesRead;
    }

    printf("Received %zu bytes\n", frameSize);
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

}
