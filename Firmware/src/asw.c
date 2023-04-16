#include "asw.h"
#include "bsw_api.h"
#include "general.h"
#include "xc.h"

static uint8_t statusAsw = 0;

void incrementStatus(void) {
    statusAsw++;
}

void asw_init(void){
    //TODO INITIALIZE ASW PROGRAM
    registerButtonPressedCB(&incrementStatus);
    registerButtonReleasedCB(&incrementStatus);
}

void asw_10ms(void){
    //TODO 10MS PROCESS FOR ASW
    uint8_t sensorVal;
    getSensorVal(2,&sensorVal);
    if(sensorVal < 100){
        setLEDState(1,ON);
    }
    else{
        setLEDState(1,OFF);
    }
    getSensorVal(4,&sensorVal);
    if(sensorVal < 100){
        setLEDState(2,ON);
    }
    else{
        setLEDState(2,OFF);
    }
    setASWStatus(statusAsw);
}