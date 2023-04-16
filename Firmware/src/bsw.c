#include "bsw.h"
#include "general.h"
#include "ioconfig.h"
#include "myTimers.h"
#include "i2cComm.h"
#include "uartTp.h"
#include "vl6180x.h"
//#include "myPWM.h"
#include "diag.h"
//#include "motors.h"
#include "xc.h"

void bsw_init(void) {
    setupIO(); //configures inputs and outputs
    initTimer1(10.0);
    uartTp_init();
    setupI2C();
    startI2C();
    SEN1 = SENOFF;
    SEN2 = SENOFF;
    SEN3 = SENOFF;
    SEN4 = SENOFF;
    SEN5 = SENOFF;
    //init_vl6180x(1);
    init_vl6180x(2);
    //init_vl6180x(3);
    init_vl6180x(4);
    //init_vl6180x(5);
    
    uartTp_registerUpdateFunc(sensor1, &getSensor1Data);
    uartTp_registerUpdateFunc(sensor2, &getSensor2Data);
    uartTp_registerUpdateFunc(sensor3, &getSensor3Data);
    uartTp_registerUpdateFunc(sensor4, &getSensor4Data);
    uartTp_registerUpdateFunc(sensor5, &getSensor5Data);
    
    //initPWM1(1000);
    //motor_init();
    
    diag_init();
}

void init_end(void) {
    startTimer1();
}

void bsw_10ms(void) {
    IO_10ms();
    //motor_10ms();
    uartTp_10ms();
    I2CTp_10ms();
    diag_10ms();
}