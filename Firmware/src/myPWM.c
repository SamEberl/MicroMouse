#include <xc.h>
#include "myPWM.h"

static unsigned int TPeriod1 = 0;

errorCode initPWM1(unsigned int freq){
    errorCode retVal;
    TPeriod1 = (unsigned long) (1000.0/((float)(freq)*TCYCLE_MS));
    unsigned int i = 0;
    
    P1TCONbits.PTEN = 0; // Switch off PWM generator
    
    for(i=0;i<4;i++) {
        unsigned long temp = (unsigned long) (TPeriod1/PWM1_PRESCALER[i]);
        
        if(temp<= 0x7FFFu) {
            TPeriod1 = temp;
            P1TCONbits.PTCKPS = i;
            P1TPER = TPeriod1;
            break;
        }
    }
    if (i==4) return OUT_OF_BOUNDS_ERROR;
    
    PWM1CON1bits.PMOD1 = 1;
    PWM1CON1bits.PMOD2 = 1;
    PWM1CON1bits.PMOD3 = 1;
    
    for(i=1;i<4;i++) {
        retVal = setPWM1Mode(i, DISABLE_H_DISABLE_L);
        if (retVal != OK) return retVal;
        retVal = setPWM1DC(i, 0.0);
        if (retVal != OK) return retVal;
    }
    P1TCONbits.PTEN = 1; // Switch on PWM generator
    return retVal;
}

errorCode setPWM1Mode(unsigned char channel, pwmMode mode) {
    switch (channel) {
        case 1:
            switch (mode){
                case DISABLE_H_DISABLE_L:
                    PWM1CON1bits.PEN1H = 0;
                    PWM1CON1bits.PEN1L = 0;
                    P1OVDCONbits.POVD1H=0;
                    P1OVDCONbits.POVD1L=0;
                    break;
                case ENABLE_H_DISABLE_L:
                    PWM1CON1bits.PEN1H = 1;
                    PWM1CON1bits.PEN1L = 0;
                    P1OVDCONbits.POVD1H=1;
                    P1OVDCONbits.POVD1L=0;
                    break;
                case DISABLE_H_ENABLE_L:
                    PWM1CON1bits.PEN1H = 0;
                    PWM1CON1bits.PEN1L = 1;
                    P1OVDCONbits.POVD1H=0;
                    P1OVDCONbits.POVD1L=1;
                    break;
                case ENABLE_H_ENABLE_L:
                    PWM1CON1bits.PEN1H = 1;
                    PWM1CON1bits.PEN1L = 1;
                    P1OVDCONbits.POVD1H=1;
                    P1OVDCONbits.POVD1L=1;
                    break;
                default:
                    return INVALID_ARGUMENT_ERROR;
            }
            break;
        case 2:
            switch (mode){
                case DISABLE_H_DISABLE_L:
                    PWM1CON1bits.PEN2H = 0;
                    PWM1CON1bits.PEN2L = 0;
                    P1OVDCONbits.POVD2H=0;
                    P1OVDCONbits.POVD2L=0;
                    break;
                case ENABLE_H_DISABLE_L:
                    PWM1CON1bits.PEN2H = 1;
                    PWM1CON1bits.PEN2L = 0;
                    P1OVDCONbits.POVD2H=1;
                    P1OVDCONbits.POVD2L=0;
                    break;
                case DISABLE_H_ENABLE_L:
                    PWM1CON1bits.PEN2H = 0;
                    PWM1CON1bits.PEN2L = 1;
                    P1OVDCONbits.POVD2H=0;
                    P1OVDCONbits.POVD2L=1;
                    break;
                case ENABLE_H_ENABLE_L:
                    PWM1CON1bits.PEN2H = 1;
                    PWM1CON1bits.PEN2L = 1;
                    P1OVDCONbits.POVD2H=1;
                    P1OVDCONbits.POVD2L=1;
                    break;
                default:
                    return INVALID_ARGUMENT_ERROR;
            }
            break;
        case 3:
            switch (mode){
                case DISABLE_H_DISABLE_L:
                    PWM1CON1bits.PEN3H = 0;
                    PWM1CON1bits.PEN3L = 0;
                    break;
                case ENABLE_H_DISABLE_L:
                    PWM1CON1bits.PEN3H = 1;
                    PWM1CON1bits.PEN3L = 0;
                    break;
                case DISABLE_H_ENABLE_L:
                    PWM1CON1bits.PEN3H = 0;
                    PWM1CON1bits.PEN3L = 1;
                    break;
                case ENABLE_H_ENABLE_L:
                    PWM1CON1bits.PEN3H = 1;
                    PWM1CON1bits.PEN3L = 1;
                    break;
                default:
                    return INVALID_ARGUMENT_ERROR;
            }
            break;
        default:
            return INVALID_ARGUMENT_ERROR;
    }
    return OK;
}

errorCode setPWM1DC(unsigned char channel, float dc) {
    dc = 1.0-dc;
    if ((dc<0.0) || (dc > 1.0)) return OUT_OF_BOUNDS_ERROR;
    switch (channel) {
        case 1:
            P1DC1 = (unsigned int)(dc*TPeriod1*2.0);
            break;
        case 2:
            P1DC2 = (unsigned int)(dc*TPeriod1*2.0);
            break;
        case 3:
            P1DC3 = (unsigned int)(dc*TPeriod1*2.0);
            break;
        default:
            return INVALID_ARGUMENT_ERROR;
    }
    return OK;
}