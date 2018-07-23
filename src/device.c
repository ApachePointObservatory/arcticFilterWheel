#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "ArcusPerformaxDriver.h"
#include "evgpio.h"

AR_HANDLE       Handle; //usb handle
AR_DWORD        num;
int             retVal;
char            usbCmd[64];
char            usbResponse[64];

// function signatures
int commFlush();
int sendCmd(char *cmdStr);
int disconnect();
char* getUSBReply();
void evgpioinit();

// Set data output value, 1 high, 0 low
void evsetdata(int dio, int value);

// Set DIO direction, 1 output, 0 input
void evsetddr(int dio, int value);

// Set or clear the watch mask for a dio
void evsetmask(int dio, int value);

// Returns the input value for a dio
int evgetin(int dio);

// Clears all pending events
void evclrwatch();


char* getUSBReply(){

    return usbResponse;
}

int commFlush(){
    if (!fnPerformaxComFlush(Handle)){
        printf("Error flushing comms\n");
        return -1;
    }
    else{
        printf("Comms flushed\n");
    }
    return 1;
}

int sendCmd(char *cmdStr){
    strcpy(usbCmd, cmdStr);
    if(!fnPerformaxComSendRecv(Handle, usbCmd, 64,64, usbResponse)){
        printf("Command %s failed\n", cmdStr);
        return -1;
    }
    printf("Command Succeeded: %s, response: %s\n", cmdStr, usbResponse);
    return 1;
}

int disconnect(){
    if(!fnPerformaxComClose(Handle)){
        printf( "Error disconnecting device\n");
        return -1;
    }
    return 1;
}

int connect(){
    if(!fnPerformaxComGetNumDevices(&num)){
        printf("error in fnPerformaxComGetNumDevices\n");
        return -1;
    }
    else{
        printf("found %ld devices\n", num);
    }
    if(!fnPerformaxComOpen(0,&Handle)){
        printf( "Error opening device\n");
        return -1;
    }
    else{
        printf("device opened\n");
    }
    if(!fnPerformaxComSetTimeouts(500,500)){
        printf("Error setting timeouts\n");
        return -1;
    }
    else{
        printf("set timeouts\n");
    }


    return 1;
}