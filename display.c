/*******************************************************************************
@file			display.c
@author		Samuel Yamoah
@date			05.06.2016
@modified	07.06.2016
@brief		Set up of the UART Display and Stellaris
*******************************************************************************/

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/uart.h"
#include "drivers/rit128x96x4.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "utils/isqrt.h"
#include "utils/circBuf.h"
#include "stdio.h"
#include "stdlib.h"


#include "pid_control.h"
#include "yaw_control.h"
#include "display.h"

//**********************************************************************
// Initialise UART0 - 8 bits, 1 stop bit, no parity
//**********************************************************************
void initConsole (void)
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
void UARTSend (char *pucBuffer)
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

void initDisplay (void)
{
  // intialise the OLED display
  RIT128x96x4Init(1000000);
}


//*****************************************************************************
// Function to display the displays the current height (mili Volts), reference
// height (mili Volts) and the current height as a percentage.
//*****************************************************************************
void displayInfo(int inital, int height, int degrees, long g_ulSampCnt,
  long altError, int desiredHeight, int desiredYaw, int main_duty, int tail_duty)
{
	char string[40];

	sprintf(string, " Main: %d Tail: %d", main_duty, tail_duty);
	RIT128x96x4StringDraw(string, 5, 14, 15);
	sprintf(string,"Alt (%%): %d [%d] {%d}", desiredHeight, height, altError);
	RIT128x96x4StringDraw(string, 5, 24, 15);
	sprintf(string,"altError {%d}", altError);
	RIT128x96x4StringDraw(string, 5, 34, 15);


	sprintf (string," Yaw: %d [%d]",desiredYaw, degrees);
	RIT128x96x4StringDraw(string, 5, 64, 15);

	sprintf (string, "Deg = %4d", degrees);
	RIT128x96x4StringDraw (string, 5, 84, 15);

	if ((g_ulSampCnt % 25) == 0){
		sprintf(string, " Main: %d Tail: %d\n----------\n", main_duty, tail_duty);
		UARTSend (string);
		sprintf(string, "Alt (%%): %d [%d] {%d}\n----------\n", desiredHeight,
                                                            height, altError);
		UARTSend (string);
		sprintf(string, " Yaw: %d [%d]\n----------\n",desiredYaw, degrees);
		UARTSend (string);
	}
}
