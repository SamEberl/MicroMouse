/* 
 * File:   IOconfig.h
 * Author: adamp
 *
 * Created on 13 December 2019, 09:38
 */

#ifndef IOCONFIG_H
#define	IOCONFIG_H

#include "io_public.h"
#include "general.h"
#include "xc.h"

#define SEN1 LATAbits.LATA7
#define SEN2 LATAbits.LATA10
#define SEN3 LATAbits.LATA9
#define SEN4 LATAbits.LATA4
#define SEN5 LATAbits.LATA8

#define LED1 LATCbits.LATC6
#define LED2 LATCbits.LATC7
#define LED3 LATCbits.LATC0

#define SW1 PORTCbits.RC1

#define SW_PRESSED 0
#define SW_RELEASED 1

#define LEDOFF 0
#define LEDON 1
#define SENON 1
#define SENOFF 0

void setupIO();
void IO_10ms();
errorCode changeLEDState(uint8_t ledNr, uint8_t state);
errorCode IO_registerButtonPressedCB(ButtonPressedCB callback);
errorCode IO_registerButtonReleasedCB(ButtonReleasedCB callback);

#endif	/* IOCONFIG_H */

