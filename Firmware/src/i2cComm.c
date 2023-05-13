/*
 * File:   i2cComm.c
 * Author: louis
 *
 * Created on January 21, 2023, 3:36 PM
 */


#include <string.h>

#include "xc.h"
#include "i2cComm.h"
#include "general.h"

I2Cstate state = IDLE;
I2Ccmd cmd;
char I2CTXBuffer[4] = {0};
int I2CDataLen = 0;
char I2CIdx[2] = {0};
uint8_t I2CAddress = 0x00;
uint8_t* I2CRXDestinationPtr;
char done = FALSE;
I2C_receive_CB Rx_callback;
I2C_transmit_CB Tx_callback;
uint8_t callback_flg;

errorCode setupI2C(void)
{
    I2C1BRG = 62;
    I2C1CONbits.I2CEN = 0;
    IPC4bits.MI2C1IP = 5;
    IEC1bits.MI2C1IE = 0;
    return OK;
}

errorCode startI2C(void)
{
    int i;
    I2C1CONbits.I2CEN = 1;
    IEC1bits.MI2C1IE = 1;
    for (i = 0; i < 30000; i++); // short dirty delay for changes to take effect,
    return OK;
}

errorCode I2C_transmit(uint8_t slave_add, uint16_t idx ,const uint8_t *data, uint8_t len, I2C_transmit_CB TxCB, uint8_t cb_flg)
{
    if(state!= IDLE) return BUSY;
    if(len>4 || len<1) return OUT_OF_BOUNDS_ERROR;
    state = START;
    cmd = TX;
    I2CAddress = slave_add;
    I2CIdx[0] = (idx>>8);
    I2CIdx[1] = (idx&0x00FF);
    callback_flg = cb_flg;
    Tx_callback = TxCB;
    memcpy(I2CTXBuffer, data, len);
    I2CDataLen = len;
    I2C1CONbits.SEN = 1;
    return OK;
}

errorCode I2C_transmit_blocking(uint8_t slave_add, uint16_t idx ,const uint8_t *data, uint8_t len){
    errorCode retVal;
    done=FALSE;
    retVal = I2C_transmit(slave_add,idx, data, len, NULL, 0);
    if(retVal == OK) {
        while(!(done==TRUE));
    }
    return retVal;
}

errorCode I2C_receive(uint8_t slave_add, uint16_t idx, uint8_t *data, uint8_t len, I2C_receive_CB RxCB, uint8_t cb_flg)
{
    if(state != IDLE) return BUSY;
    if(len>4 || len<1) return OUT_OF_BOUNDS_ERROR;
    state = START;
    cmd = RX;
    callback_flg = cb_flg;
    Rx_callback = RxCB;
    I2CAddress = slave_add;
    I2CIdx[0] = (idx>>8);
    I2CIdx[1] = (idx&0x00FF);
    I2CRXDestinationPtr = data;
    I2CDataLen = len;
    I2C1CONbits.SEN = 1;
    return OK;
}

errorCode I2C_receive_blocking(uint8_t slave_add, uint16_t idx, uint8_t *data, uint8_t len)
{
    errorCode retVal;
    done=FALSE;
    retVal = I2C_receive(slave_add,idx, data, len, NULL, 0);
    if(retVal == OK) {
        while(!(done==TRUE));
    }
    return retVal;
}

void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt(void)
{
    IFS1bits.MI2C1IF = 0;
    switch(state)
    {
        case IDLE:
            break;
        case START:
            state = ADDR;
            I2C1TRN = WRITE(I2CAddress);
            break;
        case ADDR:
            state = IDX_H;
            I2C1TRN = I2CIdx[0];
            break;
        case IDX_H:
            state = IDX_L;
            I2C1TRN = I2CIdx[1];
            break;
        case IDX_L:
            if(cmd == TX){
                state = SEND;
                I2CDataLen--;
                I2C1TRN = I2CTXBuffer[I2CDataLen];
            }else{
                state = REPEAT_START;
                I2C1CONbits.RSEN = 1;
            }
            break;
        case STOP:
            state = IDLE;
            done = TRUE;
            if(callback_flg == 1) {
                if(cmd == RX) {
                    Rx_callback(OK, I2CRXDestinationPtr, 1);
                }
                else {
                    Tx_callback(OK);
                }
            }
            break;
        case ACK:
            state = RECEIVE;
            I2C1CONbits.RCEN = 1;
            break;
        case NACK:
            state = STOP;
            I2C1CONbits.PEN = 1;
            break;
        case SEND:
            I2CDataLen--;
            if(I2CDataLen<0){
                state = STOP;
                I2C1CONbits.PEN = 1;
            }
            else {
                state = SEND;
                I2C1TRN = I2CTXBuffer[I2CDataLen];
            }
            break;
        case RECEIVE:
            I2CDataLen--;
            I2CRXDestinationPtr[I2CDataLen] = I2C1RCV;
            if(I2CDataLen <= 0){
                state = NACK;
                I2C1CONbits.ACKDT = 1;
                I2C1CONbits.ACKEN = 1;
            }
            else {
                state = ACK;
                I2C1CONbits.ACKDT = 0;
                I2C1CONbits.ACKEN = 1;
            }
            break;
        case REPEAT_START:
            state = ACK;
            I2C1TRN = READ(I2CAddress);
            break;
        default:
            //TODO set I2C Fault
            break;
    }
}



