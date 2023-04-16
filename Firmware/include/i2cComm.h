#ifndef I2CCOMM_H
#define I2CCOMM_H

#include "general.h"

#define WRITE(x) ((x<<1)&0xFE)
#define READ(x) ((x<<1)|0x01)

typedef enum I2Cstate{
    IDLE,
    START,
    STOP,
    ADDR,
    IDX_H,
    IDX_L,
    ACK,
    NACK,
    SEND,
    RECEIVE,
    REPEAT_START
}I2Cstate;

typedef enum I2Ccmd{
    TX,
    RX
}I2Ccmd;

typedef errorCode (*I2C_transmit_CB)(errorCode status);
typedef errorCode (*I2C_receive_CB)(errorCode status, uint8_t *data, uint8_t len);

errorCode setupI2C(void);
errorCode startI2C(void);
errorCode I2C_transmit(uint8_t slave_add, uint16_t idx, const uint8_t *data, uint8_t len, I2C_transmit_CB TxCB, uint8_t cb_flg);
errorCode I2C_transmit_blocking(uint8_t slave_add, uint16_t idx ,const uint8_t *data, uint8_t len);
errorCode I2C_receive(uint8_t slave_add, uint16_t idx, uint8_t *data, uint8_t len, I2C_receive_CB RxCB, uint8_t cb_flg);
errorCode I2C_receive_blocking(uint8_t slave_add, uint16_t idx, uint8_t *data, uint8_t len);

#endif