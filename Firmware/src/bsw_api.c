#include "bsw_api.h"
#include "vl6180x.h"
#include "uartTp.h"
#include "ioconfig.h"
#include "general.h"
#include "xc.h"

errorCode getSensorVal(uint8_t sensorNum, uint8_t *data){
    switch(sensorNum) {
        case 1:
            return getSensor1Data(data);
        case 2:
            return getSensor2Data(data);
        case 3:
            return getSensor3Data(data);
        case 4:
            return getSensor4Data(data);
        case 5:
            return getSensor5Data(data);
        default:
            return INVALID_ARGUMENT_ERROR;
    }
}

errorCode getMotorVelocity(motor mot, uint8_t *data){
    return GENERAL_ERROR;
}

errorCode setMotorVelocity(motor mot, uint8_t data){
    return GENERAL_ERROR;
}

errorCode registerButtonPressedCB(ButtonPressedCB callback){
    return IO_registerButtonPressedCB(callback);
}

errorCode registerButtonReleasedCB(ButtonReleasedCB callback){
    return IO_registerButtonReleasedCB(callback);
}

errorCode setLEDState(uint8_t ledNr, LEDstate state){
    if(ledNr == 3) return INVALID_ARGUMENT_ERROR;
    if(state == ON){
        return changeLEDState(ledNr,LEDON);
    }
    else{
        return changeLEDState(ledNr,LEDOFF);
    }
}

errorCode setASWStatus(uint8_t status){
    return uartTp_updateMessageContent(aswSts, &status);
}