/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */
 
#ifndef UARTIF_H
#define	UARTIF_H

#include <xc.h> // include processor files - each processor file is guarded. 
#include "general.h"

#define OUTPUT_BUFFER_SIZE 20
#define INPUT_BUFFER_SIZE 20

void setupUART1(void);
//TX
errorCode transmitASCII(const char *buffer);
errorCode transmitData(const char *buffer, unsigned int len);
errorCode transmitStatus(void);

//RX
errorCode StopReceiving();
errorCode StartReceiving();
errorCode GetNextMessage(char* buffer);
#endif
