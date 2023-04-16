#include "uartTp.h"
#include "uartComm.h"
#include "ioconfig.h"
#include <string.h>

static uartMsg msg = {0};

static updateMsgContentCB callbacks[10] = {0};
static uint8_t callbackRegFlg[10] = {0};
static uint8_t outputBuffer[OUTPUT_BUFFER_SIZE] = {0};
static uint8_t inputBuffer[INPUT_BUFFER_SIZE] = {0};
static uint8_t diagMsgFlg = 0;
static uint8_t sencCyclicMsgs = FALSE;
static diagServiceHandlerCB diagServiceHandler;
static uint8_t diagServiceHandlerRegistered = FALSE;

errorCode uartTp_Prv_FillBuffer(){
    //Fill Measurements
    outputBuffer[0] = msg.sensor1;
    outputBuffer[1] = msg.sensor2;
    outputBuffer[2] = msg.sensor3;
    outputBuffer[3] = msg.sensor4;
    outputBuffer[4] = msg.sensor5;
    outputBuffer[5] = msg.motLDesVel;
    outputBuffer[6] = msg.motLActVel;
    outputBuffer[7] = msg.motRDesVel;
    outputBuffer[8] = msg.motRActVel;
    outputBuffer[9] = msg.aswSts;
    memcpy(outputBuffer+10,&msg.diagMsg,10);
    return OK;
}

errorCode uartTp_Prv_Send(void){
    errorCode retVal = OK;
    // Update all Measurements where a callback is registered
    int i = 0;
    for(i=0;i<10;i++){
        if(callbackRegFlg[i]==1u) {
            uint8_t buffer;
            //get Data from Callback
            retVal = callbacks[i](&buffer);
            if(retVal != OK) return retVal;
            //Update Message Content with new value
            retVal = uartTp_updateMessageContent((uartMsgContent)i, &buffer);
            if(retVal != OK) return retVal;
        }
    }
    retVal = uartTp_Prv_FillBuffer();
    if(retVal != OK) return retVal;
    
    //Check if Diagnostic Message is part of the Message
    if(diagMsgFlg == TRUE){
        uint8_t diag_len = outputBuffer[10];
        transmitData((const char*)outputBuffer, 11u+diag_len);
    }
    else{
        outputBuffer[10] = 0;
        transmitData((const char*)outputBuffer, 11u);
    }
    //Reset Diagnostic Message Flag
    diagMsgFlg = FALSE;
    return OK;
}

errorCode uartTp_init(void){
    setupUART1();
    StartReceiving();
    return OK;
}

errorCode uartTp_startReceiving(void){
    return StartReceiving();
}


errorCode uartTp_updateMessageContent(uartMsgContent content, uint8_t *data){
    switch(content) {
        case sensor1:
            msg.sensor1 = *data;
            break;
        case sensor2:
            msg.sensor2 = *data;
            break;
        case sensor3:
            msg.sensor3 = *data;
            break;
        case sensor4:
            msg.sensor4 = *data;
            break;
        case sensor5:
            msg.sensor5 = *data;
            break;
        case motLDesVel:
            msg.motLDesVel = *data;
            break;
        case motLActVel:
            msg.motLActVel = *data;
            break;
        case motRDesVel:
            msg.motRDesVel = *data;
            break;
        case motRActVel:
            msg.motRActVel = *data;
            break;
        case aswSts:
            msg.aswSts = *data;
            break;
        case diagMsg:
            memcpy(&msg.diagMsg,data,10);
            diagMsgFlg = TRUE;
            break;
        default:
            return INVALID_ARGUMENT_ERROR;
    }
    return OK;
}

errorCode uartTp_registerUpdateFunc(uartMsgContent content, updateMsgContentCB callbackFunc){
    callbacks[(int)content] = callbackFunc;
    callbackRegFlg[(int)content] = 1u;
    return OK;
}

errorCode uartTp_serviceHandler(uint8_t len, uint8_t *data){
    uint8_t service = data[0];
    switch(service){
        case START_MSG:
            sencCyclicMsgs = TRUE;
            changeLEDState(3, LEDON);
            break;
        case STOP_MSG:
            sencCyclicMsgs = FALSE;
            changeLEDState(3, LEDOFF);
            break;
        default:
            return INVALID_ARGUMENT_ERROR;
    }
    return OK;
}

errorCode uartTp_registerdiagServiceHandlerFunc(diagServiceHandlerCB callbackFunc){
    diagServiceHandler = callbackFunc;
    diagServiceHandlerRegistered = TRUE;
    return OK;
}

errorCode diag_serviceHandler(uint8_t len, uint8_t *data){
    if(diagServiceHandlerRegistered != TRUE) return UNINITIALIZED;
    return diagServiceHandler(len, data);
}

errorCode uartTp_10ms(void){
    errorCode retVal = OK;
    //TX
    static int cnt = 0;
    if(cnt==10){
        if(sencCyclicMsgs == TRUE) retVal = uartTp_Prv_Send();
        cnt = 0;
    }else cnt++;
    if (retVal != OK) return retVal;
    
    //RX
    retVal = GetNextMessage((char*)&inputBuffer);
    if(retVal == OK){
        uint8_t len = inputBuffer[0];
        if(len<=0) return OUT_OF_BOUNDS_ERROR;
        uint8_t service = inputBuffer[1];
        //Remove TP Header
        inputBuffer[1] = inputBuffer[1]&0x0F;
        if(service&TP_MASK){
            return uartTp_serviceHandler(len, inputBuffer+1);
        }
        if(service&DIAG_MASK){
            return diag_serviceHandler(len, inputBuffer+1);
        }
        return INVALID_ARGUMENT_ERROR;
    }
    else return OK;
}