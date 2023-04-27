#ifndef MOTORS_H
#define MOTORS_H

#include "general.h"
#include "xc.h"

void motor_init(void);
void motor_10ms(void);
errorCode setDesiredVelocity(uint8_t mot, int8_t velocity);

#endif