#include "motors.h"
#include "motorEncoders.h"
#include "params.h"
#include "uartTp.h"
#include "myPWM.h"
#include "motor_public.h"
#include <math.h>
#include <xc.h>
#include <stdlib.h>

#define NUM_ERRORS_STORED 30
#define MOTOR_MAX_VELOCITY 100 //From Datasheet calculated. Probably not really reachable but a good starting point

typedef struct motor{
    int8_t desiredVelocity;
    int8_t actualVelocity;
    double kp;
    double ki;
    double kd;
    int8_t errorList[NUM_ERRORS_STORED];
    int16_t errorSum;
    uint8_t idx;
    double dc;
    uint8_t direction;
}motor;

static motor MotorLeft = {0};
static motor MotorRight = {0};

errorCode updateMotorState(){
    uint16_t temp;
    //Update Motor Left
    //Parameter update
    getParameter(PARAM_BSW_L_KP, &temp);
    MotorLeft.kp = ((double)temp)/1000.0;
    getParameter(PARAM_BSW_L_KI, &temp);
    MotorLeft.ki = ((double)temp)/1000.0;
    getParameter(PARAM_BSW_L_KD, &temp);
    MotorLeft.kd = ((double)temp)/1000.0;
    //Measure Velocity
    MotorLeft.actualVelocity = getMotorLeftVelocity();
    //Update Motor Right
    //Parameter update
    getParameter(PARAM_BSW_R_KP, &temp);
    MotorRight.kp = ((double)temp)/1000.0;
    getParameter(PARAM_BSW_R_KI, &temp);
    MotorRight.ki = ((double)temp)/1000.0;
    getParameter(PARAM_BSW_R_KD, &temp);
    MotorRight.kd = ((double)temp)/1000.0;
    //Measure Velocity
    MotorRight.actualVelocity = getMotorRightVelocity();
    
    return OK;
}

double calcuclateForwardingDC(int8_t desiredVelocity) {
    double dc = ((double)desiredVelocity)/((double)MOTOR_MAX_VELOCITY);
    if(dc < -1.0) return -1.0;
    else if(dc > 1.0) return 1.0;
    return dc;
}

void setSpeed(void){
    uartTp_updateMessageContent(motLDesVel, &MotorLeft.desiredVelocity);
    //uartTp_updateMessageContent(motLActVel, &MotorLeft.actualVelocity);
    uartTp_updateMessageContent(motRDesVel, &MotorRight.desiredVelocity);
    //uartTp_updateMessageContent(motRActVel, &MotorRight.actualVelocity);
    
    //Motor Right
    if(MotorRight.direction == TURN_FORWARD)
        setPWM1Mode(2,DISABLE_H_ENABLE_L);
    else
        setPWM1Mode(2,ENABLE_H_DISABLE_L);
    
    setPWM1DC(2,MotorRight.dc);
    
    //Motor Left
    if(MotorLeft.direction == TURN_FORWARD)
        setPWM1Mode(1,ENABLE_H_DISABLE_L);
    else
        setPWM1Mode(1,DISABLE_H_ENABLE_L);
    
    setPWM1DC(1,MotorLeft.dc);
}

errorCode controlLoopStep(motor* mot){
    int8_t error = mot->desiredVelocity - mot->actualVelocity;
    int8_t lastError = mot->errorList[mot->idx];
    double forwarding_dc = calcuclateForwardingDC(mot->desiredVelocity);
    //increment idx
    (mot->idx)++;
    if(mot->idx == NUM_ERRORS_STORED) mot->idx = 0;
    
    //Update error Sum
    mot->errorSum -= mot->errorList[mot->idx];
    mot->errorList[mot->idx] = error;
    mot->errorSum += mot->errorList[mot->idx];
    
    double p = mot->kp * (double)error;
    double i = mot->ki * (double)mot->errorSum;
    double d = mot->kd * (double)(error-lastError);
    
    //double dc = p+i+d+forwarding_dc;
    double dc = p+i+d;
    if((dc<-0.6)||(dc>0.6)) mot->dc = 0.6;
    else mot->dc = fabs(dc);
    
    if(dc<0.0) mot->direction = TURN_BACKWARD;
    else mot->direction = TURN_FORWARD;
    
    return OK;
}

void motor_init(void){
    MotorLeft.desiredVelocity = 0;    
    MotorRight.desiredVelocity = 0;
    
}

void motor_10ms(void){
    updateMotorState();
    controlLoopStep(&MotorLeft);
    controlLoopStep(&MotorRight);
    setSpeed();
}

errorCode setDesiredVelocity(uint8_t mot, int8_t velocity){
    if((velocity<-MOTOR_MAX_VELOCITY)||(velocity>MOTOR_MAX_VELOCITY)) return INVALID_ARGUMENT_ERROR;
    switch(mot){
        case MOTOR_L:
            MotorLeft.desiredVelocity = velocity;
            break;
        case MOTOR_R:
            MotorRight.desiredVelocity = velocity;
            break;
        default:
            return INVALID_ARGUMENT_ERROR;
    }
    return OK;
}