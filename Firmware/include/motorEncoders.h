/*
 * File:   motorEncoders.h
 * Author: bauma
 *
 * Created on 23. Mai 2019, 11:22
 */

#ifndef __MOTORENCODERS_H__
#define	__MOTORENCODERS_H__

#define TURN_BACKWARD 0
#define TURN_FORWARD 1

#include "general.h"
#include<xc.h>

void initQEI1( unsigned int  startPos);
void initQEI2( unsigned int  startPos);

void motorEncoder_10ms(void);

long getPositionInCounts_1();
long getPositionInCounts_2();

int8_t getMotorRightVelocity(void);
int8_t getMotorLeftVelocity(void);

errorCode getMororRightVelocityData(uint8_t *data);
errorCode getMororLeftVelocityData(uint8_t *data);



#define WHEEL_ROTATIONS_PERROBOT_ROTATION 2.5
#define TICKS_PER_WHEELROTATION (64*33)
#define TICKS_PER_CENTIMETER TICKS_PER_WHEELROTATION/12.566
#define METER_PER_TICkS 0.12566/TICKS_PER_WHEELROTATION
#define DELTATICKS_90_DEGREES  (0.25* WHEEL_ROTATIONS_PERROBOT_ROTATION*TICKS_PER_WHEELROTATION) 
#define DELTATICKS_180_DEGREES (0.5 * WHEEL_ROTATIONS_PERROBOT_ROTATION*TICKS_PER_WHEELROTATION)
#define DELTATICKS_CELL_GAP (11.5*TICKS_PER_CENTIMETER)




#endif	/* __MOTORENCODERS_H__ */