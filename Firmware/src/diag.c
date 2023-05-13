#include <xc.h>
#include "uartTp.h"
#include "diag.h"
#include "string.h"
#include "params.h"

//DIAG SERVICES
#define WRITE_PARAMETER 0x01
#define READ_ERROR_MEM 0x02
#define START_ROUTINE 0x03

typedef enum diagState{
    INIT,
    HANDLE_SERVICE,
    SEND_ANS
}diagState;

static uint8_t diagInputBuffer[10] = {0};
static uint8_t inputLen = 0;
static uint8_t diagOutputBuffer[10] = {0};

static diagState state = INIT;

errorCode diagRXIndication(uint8_t len, uint8_t *data){
    if(len>10) return OUT_OF_BOUNDS_ERROR;
    if (state != INIT) return BUSY;
    memcpy(diagInputBuffer,data,len);
    inputLen = len;
    state = HANDLE_SERVICE;
    return OK;
}

void createNegativeResponse(errorCode error){
    diagOutputBuffer[0] = 0x02;
    diagOutputBuffer[1] = 0x7F;
    diagOutputBuffer[2] = error;
    state = SEND_ANS;
}

errorCode writeParameterService(void){
    if(inputLen != 4) return INVALID_ARGUMENT_ERROR;
    errorCode retVal;
    parameters param = (parameters)diagInputBuffer[1];
    uint16_t paramValue = (((uint16_t)diagInputBuffer[2])<<8)+diagInputBuffer[3];
    retVal = setParameter(param, paramValue);
    if(retVal != OK) return retVal;
    //Create Positive response
    diagOutputBuffer[0] = 0x02;
    diagOutputBuffer[1] = 0x40+WRITE_PARAMETER;
    diagOutputBuffer[2] = diagInputBuffer[1];
    state = SEND_ANS;
    return OK;
}

void diag_init(void){
    uartTp_registerdiagServiceHandlerFunc(&diagRXIndication);
}

void diag_10ms(void){
    errorCode retVal;
    switch(state) {
        case INIT:
            break;
        case HANDLE_SERVICE:
            switch(diagInputBuffer[0]) {
                case WRITE_PARAMETER:
                    retVal = writeParameterService();
                    if(retVal != OK) createNegativeResponse(retVal);
                    break;
                case READ_ERROR_MEM:
                    break;
                case START_ROUTINE:
                    break;
                default:
                    createNegativeResponse(OUT_OF_BOUNDS_ERROR);
                    break;
            }
            break;
        case SEND_ANS:
            uartTp_updateMessageContent(diagMsg, diagOutputBuffer);
            state = INIT;
            break;
        default:
            break;
    }
}