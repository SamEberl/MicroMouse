#ifndef UART_TP_H
#define UART_TP_H

#include <xc.h>
#include "general.h"

//RX Service Masks
#define TP_MASK 0b00010000
#define DIAG_MASK 0b00100000

//TP Services
#define START_MSG 0x01
#define STOP_MSG 0x02

/* Message Setup:
 * 10 Bytes Cyclice Measurements:
 *  0 Sensor 0
 *  1 Sensor 1
 *  2 Sensor 2
 *  3 Sensor 3
 *  4 Sensor 4
 *  5 Motor left desired velocity
 *  6 Motor left acutual velocity
 *  7 Motor right desired velocity
 *  8 Motor right actual velocity
 *  9 Application SW Status Byte
 * 10 Bytes Diagnostic Service Message
 */

typedef errorCode (*updateMsgContentCB)(uint8_t *data);
typedef errorCode (*diagServiceHandlerCB)(uint8_t len, uint8_t *data);

typedef enum uartMsgContent{
            sensor1 = 0,
            sensor2 = 1,
            sensor3 = 2,
            sensor4 = 3,
            sensor5 = 4,
            motLDesVel = 5,
            motLActVel = 6,
            motRDesVel = 7,
            motRActVel = 8,
            aswSts = 9,
            diagMsg = 10
}uartMsgContent;

typedef struct uartMsg{
    uint8_t sensor1;
    uint8_t sensor2;
    uint8_t sensor3;
    uint8_t sensor4;
    uint8_t sensor5;
    uint8_t motLDesVel;
    uint8_t motLActVel;
    uint8_t motRDesVel;
    uint8_t motRActVel;
    uint8_t aswSts;
    uint8_t diagMsg[10];
}uartMsg;

errorCode uartTp_10ms(void);
errorCode uartTp_init(void);
errorCode uartTp_startReceiving(void);

errorCode uartTp_updateMessageContent(uartMsgContent content, uint8_t *data);
errorCode uartTp_registerUpdateFunc(uartMsgContent content, updateMsgContentCB callbackFunc);
errorCode uartTp_registerdiagServiceHandlerFunc(diagServiceHandlerCB callbackFunc);
#endif