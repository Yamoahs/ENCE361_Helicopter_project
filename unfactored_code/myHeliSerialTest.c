//*****************************************************************************
//
// heliSerialTest.c - Test program for the serial output of status data
//  [Based on butV2Test.c and serial_hw#.c]
//
// Repetitively sends a block of status lines via UART0 (including newlines)
//  at a rate which can be adjusted between 10 per second and 1 per 4 secs
//
// Author:  P.J. Bones	UCECE
// Last modified:	23.3.2015
//*****************************************************************************

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "driverlib/debug.h"
#include "drivers/rit128x96x4.h"
#include "driverlib/uart.h"
#include "stdio.h"
#include "buttonSet.h"
#include "button.h"

//*****************************************************************************
// Constants
#define SYSTICK_RATE_HZ 1000ul
#define BAUD_RATE 9600ul

//**********************************************************************
// Initialise the clock
//**********************************************************************
initClock (void)
{
	   //
	   // Set the clock rate to 20 MHz
	   SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
	                   SYSCTL_XTAL_8MHZ);
}

//**********************************************************************
// Initialise UART0 - 8 bits, 1 stop bit, no parity
//**********************************************************************
initConsole (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), BAUD_RATE,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UART0_BASE);
    UARTEnable(UART0_BASE);
}

//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void
UARTSend (char *pucBuffer)
{
    //
    // Loop while there are more characters to send.
    //
    while(*pucBuffer)
    {
        //
        // Write the next character to the UART Tx FIFO.
        //
        UARTCharPut(UART0_BASE, *pucBuffer);
        pucBuffer++;
    }
}

//**********************************************************************
// Transmit a dummy "status" block of helicopter data, with buffering
//**********************************************************************
void
TxStatus (void)
{
	UARTSend ("hellooooooooooo\n");
}


int
main(void)
{

	 initClock ();
	 initConsole ();

	 // Enable interrupts to the processor.
	    IntMasterEnable();

	 while (1)
	    {
		 TxStatus ();
	    }
}
