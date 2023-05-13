/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

#ifndef __MYPWM_H__
#define	__MYPWM_H__

#include "general.h"

static const unsigned int PWM1_PRESCALER[] = {1,4,16,64};

typedef enum pwmMode{
    DISABLE_H_DISABLE_L,
    ENABLE_H_DISABLE_L,
    DISABLE_H_ENABLE_L,
    ENABLE_H_ENABLE_L
}pwmMode;

errorCode initPWM1(unsigned int freq);
errorCode setPWM1Mode(unsigned char channel, pwmMode mode);
errorCode setPWM1DC(unsigned char channel, float dc);

#endif	/* __MYPWM_H__ */
