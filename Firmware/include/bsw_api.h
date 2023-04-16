#ifndef BSW_API_H
#define BSW_API_H

#include "xc.h"
#include "general.h"
#include "params.h"
#include "io_public.h"

//Sensors
errorCode getSensorVal(uint8_t sensorNum, uint8_t *data);

//Motors
errorCode getMotorVelocity(motor mot, uint8_t *data);
errorCode setMotorVelocity(motor mot, uint8_t data);

errorCode registerButtonPressedCB(ButtonPressedCB callback);
errorCode registerButtonReleasedCB(ButtonReleasedCB callback);

//LEDs
errorCode setLEDState(uint8_t ledNr, LEDstate state);

//StatusByte
errorCode setASWStatus(uint8_t status);

#endif