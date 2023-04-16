#include "myTimers.h"
#include "bsw.h"
#include "asw.h"

errorCode initTimer1(float ms) {
    T1CON = 0;
    unsigned long TPeriod = (unsigned long)(ms/TCYCLE_MS);
    int i = 0;
    for(i=0;i<4;i++){
        unsigned long tmp = (unsigned long)(TPeriod/TMR1_PRESCALER[i]);
        if(tmp<=0xFFFFu){
            TPeriod = tmp;
            T1CONbits.TCKPS = i;
            PR1 = (unsigned int) TPeriod;
            break;
        }
    }
    if(i==4) return OUT_OF_BOUNDS_ERROR;
    T1CONbits.TCS = 0;      // select internal FCY clock source
    T1CONbits.TGATE = 0;    // gated time accumulation disabled
    IFS0bits.T1IF = 0;      // reset Timer 1 interrupt flag
    IPC0bits.T1IP = 4;      // set Timer1 interrupt priority level to 4
    T1CONbits.TON = 0;      // leave timer disabled initially
    TMR1 = 0;
    IEC0bits.T1IE = 1;      // enable Timer 1 interrupt
    return OK;
}

errorCode startTimer1(void) 
{
    T1CONbits.TON = 1;
    return OK;
}

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;      // reset Timer 1 interrupt flag
    
    bsw_10ms();
    asw_10ms();
}
