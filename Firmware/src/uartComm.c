
#include <xc.h>
#include "uartComm.h"
#include <string.h>

/*
*	set-up the serial port
*   here we aim to achieve a data transfer rate of 57.6 kbit/s,
*   based on Fcycle=26.6666Mhz 
*   BaudRate=Fcycle/(16*(BRG+1))
*   ==> BRG=Fcy/(16*BaudRate) - 1 = 26.666Mhz/(16*57600) - 1 = 28.23
*   ==> choose 28 ==> BaudRate= 57.474  kbit/s, which is ~ 1% off.
 * 
 * for standard communication speed of 9600 kbit/s
 * choose 173 (factor 6)
*/

char outputBuffer[OUTPUT_BUFFER_SIZE] = {0};

char inputBuffer[INPUT_BUFFER_SIZE] = {0};

char* nextByteTX;
unsigned int bytesToSend = 0;

char newMsg = TRUE;
errorCode readyToRead = UNINITIALIZED;
unsigned int bytesToReceive = 0;
unsigned int inputByteCnt = 0;

errorCode readyToSend = OK;

void setupUART1(void)
{
	U1MODEbits.UARTEN=0; //switch the uart off during set-up
	U1BRG=173; // baud rate register
	//U1MODEbits.LPBACK=0; // in loopback mode for test! TODO: set to no loop-back (=0) after test 
	
	U1MODEbits.WAKE=0; //do not wake up on serial port activity

	U1MODEbits.ABAUD=0; //no auto baud rate detection
	U1MODEbits.PDSEL=0; //select 8 bits date, no parity
	U1MODEbits.STSEL=0; //one stop bit
    U1MODEbits.BRGH = 0; // No High Speed Mode


	IFS0bits.U1RXIF=0; //reset the receive interrupt flag
	IFS0bits.U1TXIF=0; //reset the transmission interrupt flag
    
	IPC2bits.U1RXIP=3; //set the RX interrupt priority
	IPC3bits.U1TXIP=5; //set the TX interrupt priority

	U1STAbits.URXISEL=0; //generate a receive interrupt as soon as a character has arrived
	U1STAbits.UTXEN=1; //enable the transmission of data

	IEC0bits.U1RXIE=1; //enable the receive interrupt
	IEC0bits.U1TXIE=0; //disable the transmit interrupt

	//FINALLY, 
	U1MODEbits.UARTEN=1; //switch the uart on

  	U1STAbits.UTXEN=1; //enable transmission
	
}



void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
	//Set the UART2 receiving interrupt flag to zero
    char nextByte;
 
	IFS0bits.U1RXIF=0;
    
    nextByte = U1RXREG;
	
    if (newMsg == TRUE)
    {
        //set length of the message
        bytesToReceive = nextByte;
        inputByteCnt = 0;
        
        //check input buffer space
        if (bytesToReceive >= INPUT_BUFFER_SIZE)
        {
            StopReceiving();
            readyToRead = OUT_OF_BOUNDS_ERROR;
            inputByteCnt=0;
            newMsg = TRUE;
            U1STAbits.OERR=0;
            return;
        }
        
        inputBuffer[inputByteCnt] = nextByte;
        inputByteCnt++;
        newMsg = FALSE;
        readyToRead = BUSY;
    }
    else
    {
        inputBuffer[inputByteCnt] = nextByte;
        inputByteCnt++;
        bytesToReceive--;
        if(bytesToReceive <= 0)
        {
            StopReceiving();
            readyToRead = OK;
            newMsg = TRUE;
        }
    }
 
	//we should also clear the overflow bit if it has been set (i.e. if we were to slow to read out the fifo)
	U1STAbits.OERR=0; //we reset it all the time
	//some notes on this from the data sheet
	/*
	If the FIFO is full (four characters) and a fifth character is fully received into the UxRSR register,
	the overrun error bit, OERR (UxSTA<1>), will be set. The word in UxRSR will be kept, but further
	transfers to the receive FIFO are inhibited as long as the OERR bit is set. The user must clear
	the OERR bit in software to allow further data to be received.
	If it is desired to keep the data received prior to the overrun, the user should first read all five
	characters, then clear the OERR bit. If the five characters can be discarded, the user can simply
	clear the OERR bit. This effectively resets the receive FIFO and all prior received data is lost.

	The data in the receive FIFO should be read prior to clearing the OERR bit. The
	FIFO is reset when OERR is cleared, which causes all data in the buffer to be lost.
	*/

}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{	
	//unsigned int rxData; // a local buffer to copy the data into
   // long i;
	/**Set the UART2 receiving interrupt flag to zero*/
 
	IFS0bits.U1TXIF=0;
    
    bytesToSend--;
    
    if (bytesToSend == 0) {
        IEC0bits.U1TXIE=0;
        readyToSend = OK;
    }
    else {
        readyToSend = BUSY;
        U1TXREG = *nextByteTX++;
    }
   // LED7=0;//;
}

errorCode transmitStatus(void){
    return readyToSend;
}

errorCode transmitASCII(const char *buffer){
    const char* temp = buffer;
    int i;
    for(i=0;i<OUTPUT_BUFFER_SIZE;i++) {
        if(*temp == '\0') break;
        temp++;
    }
    if(i==OUTPUT_BUFFER_SIZE) return OUT_OF_BOUNDS_ERROR;
    return transmitData(buffer, i+1);
}

errorCode transmitData(const char *buffer, unsigned int len) {
    if (readyToSend != OK) return readyToSend;
    if(len>OUTPUT_BUFFER_SIZE) return OUT_OF_BOUNDS_ERROR;
    memcpy(outputBuffer, buffer, len);
    readyToSend = BUSY;
    nextByteTX = outputBuffer;
    bytesToSend = len;
    U1TXREG = *nextByteTX++;
    IEC0bits.U1TXIE=1;
    return OK;
}

errorCode StartReceiving()
{
    IEC0bits.U1RXIE=1;
    return OK;
}

errorCode StopReceiving()
{
    IEC0bits.U1RXIE=0;
    return OK;
}

errorCode GetNextMessage(char* buffer)
{
    if (readyToRead != OK) return readyToRead;
    unsigned int len = inputBuffer[0];
    memcpy(buffer, inputBuffer, len+1);
    readyToRead = UNINITIALIZED;
    inputByteCnt=0;
    newMsg = TRUE;
    StartReceiving();
    return OK;
}