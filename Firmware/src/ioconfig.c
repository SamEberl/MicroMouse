#include "xc.h"
#include "ioconfig.h"
#include "general.h"
#include "io_public.h"

static uint8_t buttonPressed = FALSE;

static ButtonPressedCB buttonPressedCB;
static uint8_t buttonPressedInitalized = FALSE;

static ButtonReleasedCB buttonReleasedCB;
static uint8_t buttonReleasedInitalized = FALSE;

void setupIO()
{

    int i;
    AD1PCFGL=0xFFFF; //all pins are now digital, by default they are analogue
    AD1PCFGLbits.PCFG0 = 0; //set AN0 to analogue
    
    // set Sensors as output
    TRISAbits.TRISA7 = 0;   //SEN1
    TRISAbits.TRISA10 = 0;  //SEN2
    TRISAbits.TRISA9 = 0;   //SEN3
    TRISAbits.TRISA4 = 0;   //SEN4
    TRISAbits.TRISA8 = 0;   //SEN5
    
    // set LEDs as output
    TRISCbits.TRISC6 = 0;   //LED1
    TRISCbits.TRISC7 = 0;   //LED2
    TRISCbits.TRISC0 = 0;   //LED3
    
    // set SW1 as input
    TRISCbits.TRISC1 = 1;

    //PIN MAPPING
    
    //before we map, we need to unlock
    __builtin_write_OSCCONL(OSCCON & 0xbf); // clear bit 6 (unlock, they are usually write protected)
    
    // PERIPHERAL receives data from which INPUT  
    RPINR18bits.U1RXR = 24; //mapped to RP24 is U1 RX, CHANGE THIS

    //MOTOR RIGHT
    //PERIPHERAL QEA Encoder 1, receives data from RP19
    RPINR14bits.QEA1R = 19; 
    //PERIPHERAL QEB Encoder 1, receives data from RP20
    RPINR14bits.QEB1R = 20;
    
    //MOTOR LEFT
    //PERIPHERAL QEA Encoder 2, receives data from RP21
    RPINR16bits.QEA2R = 21; 
    //PERIPHERAL QEB Encoder 2, receives data from RP4
    RPINR16bits.QEB2R = 4;
    
    
    //OUTPUT PIN receives data from which PERIPHERAL, 
    //see table 11-2 in datasheet to check peripheral codes
    RPOR12bits.RP25R = 0b00011; //output bin RP25 gets data from peripheral U1 TX 

   
    //after mapping we lock again
     __builtin_write_OSCCONL(OSCCON | 0x40); // Lock PPS registers (lock again!)
     
    for (i = 0; i < 30000; i++); // short dirty delay for changes to take effect,

    SEN1 = SENOFF;
    SEN2 = SENOFF;
    SEN3 = SENOFF;
    SEN4 = SENOFF;
    SEN5 = SENOFF;
}

errorCode changeLEDState(uint8_t ledNr, uint8_t state){
    if(!((state==LEDON)||(state==LEDOFF))) return INVALID_ARGUMENT_ERROR;
    switch(ledNr){
        case 1:
            LED1 = state;
            break;
        case 2:
            LED2 = state;
            break;
        case 3:
            LED3 = state;
            break;
        default:
            return INVALID_ARGUMENT_ERROR;
    }
    return OK;
}

errorCode IO_registerButtonPressedCB(ButtonPressedCB callback){
    buttonPressedCB = callback;
    buttonPressedInitalized = TRUE;
    return OK;
}

errorCode IO_registerButtonReleasedCB(ButtonReleasedCB callback){
    buttonReleasedCB = callback;
    buttonReleasedInitalized = TRUE;
    return OK;
}

void IO_10ms(){
    uint8_t newButtonState = SW1;
    
    if(newButtonState != buttonPressed){
        buttonPressed = newButtonState;
        if(buttonPressed == SW_PRESSED){
            if(buttonPressedInitalized == TRUE) buttonPressedCB();
        }
        else{
            if(buttonReleasedInitalized == TRUE) buttonReleasedCB();
        }
    }
}