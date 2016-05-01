//*****************************************************************************
//
// pulseInrpt.c - Program to monitor the width of pulse in a digital waveform.
//
// Author:  P.J. Bones	UCECE
// Last modified:	17.3.2015
//
//*****************************************************************************
// Lab 3 sheet for ENCE361 2012: Step 6. Write an interrupt service routine to 
// read the SysTick register operating as a decrementing wrap-on-zero counter, 
// using SysTickValueGet()  for every 0->1  or 1->0  transition on an EVK header 
// pin (Pin 27 = PF5).  Maintain a circular buffer of size NUM_TICKS
// with NUM_TICKS = 20, say. Produce a continually updating display 
// of the mean pulse duration in msec over the contents of the buffer.  
// Remember that the 24-bit counter value will wrap once it reaches zero.
//*****************************************************************************

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
#include "drivers/rit128x96x4.h"
#include "stdio.h"
#include "stdlib.h"


//*****************************************************************************
// Constants
//*****************************************************************************
#define MAX_24BIT_VAL 0X0FFFFFF


//*****************************************************************************
// Global variables
//*****************************************************************************
static volatile unsigned long g_ulIntCntA;	// Monitors interrupts on A
static volatile unsigned long g_ulIntCntB;	// Monitors interrupts on B
int currentState;
int previousState;
int movement = 0;

//*****************************************************************************
//
// The interrupt handler for the for the pin change interrupt. Note that
//  the SysTick counter is decrementing.
//
//*****************************************************************************
void
PinChangeIntHandler (void)
{
	/*States:
	 * A 1 = 00
	 * B 2 = 01
	 * C 3 = 11
	 * D 4 = 10
	 */
	//static unsigned long ulLastCnt;	// Retains previous value of SysTick counter
	//unsigned long ulSysTickCnt;
	unsigned long ulPortValA;
	unsigned long ulPortValB;
	
	// 
	// Clear the interrupt (documentation recommends doing this early)
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_5 |  GPIO_PIN_7);
	//
	// Read the pin
	ulPortValA = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_5);
	ulPortValB = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_7);
	//
	// Read the SysTick counter value 
	//ulSysTickCnt = SysTickValueGet ();
	//
	// Calculate pulse width (at trailing edge only)
	if (!ulPortValA)
	{
		if (ulPortValB){
			currentState = 2;
		}
		else
		{
			currentState = 1;
		}
		// Count interrupts
			g_ulIntCntA++;
	}
	else
	{
		if (ulPortValB){
			currentState = 3;
		}
		else
		{
			currentState = 4;
		}
		// Count interrupts
		g_ulIntCntB++;
	}

	/*if (!ulPortValB)
	{
		if (ulPortValA){
					currentState = 3;
				}
				else
				{
					currentState = 2;
				}
		g_ulIntCntB++;
	}*/
	previousState = currentState;
	//
	// Prepare for next interrupt
	//ulLastCnt = ulSysTickCnt;

}

//*****************************************************************************
// Initialisation functions: clock, GPIO pin, display, buffer
//*****************************************************************************
void
initClock (void)
{
  //
  // Set the clock rate @ 3125000 Hz (minimum possible). The wrap-around 
  //  period is then 5.36871 sec.
  SysCtlClockSet (SYSCTL_SYSDIV_64 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);	
  //
  // Set up the period for the SysTick timer to get the maximum span.  
  SysTickPeriodSet (MAX_24BIT_VAL);
  //
  // Enable SysTick device (no interrupt)
  SysTickEnable ();
}

// *******************************************************
void
initPin (void)
{
    //
    // Register the handler for Port F into the vector table
    GPIOPortIntRegister (GPIO_PORTF_BASE, PinChangeIntHandler);
    //
    // Enable and configure the port and pin used:  input on PF5: Pin 27
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF); 
    GPIOPadConfigSet (GPIO_PORTF_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA,
       GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet (GPIO_PORTF_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA,
           GPIO_PIN_TYPE_STD_WPU);
    //
    // Set up the pin change interrupt (both edges)
    GPIOIntTypeSet (GPIO_PORTF_BASE, GPIO_PIN_5, GPIO_BOTH_EDGES);
    GPIOIntTypeSet (GPIO_PORTF_BASE, GPIO_PIN_7, GPIO_BOTH_EDGES);
    // 
    // Enable the pin change interrupt 
    GPIOPinIntEnable (GPIO_PORTF_BASE, GPIO_PIN_5);
    GPIOPinIntEnable (GPIO_PORTF_BASE, GPIO_PIN_7);
    IntEnable (INT_GPIOF);	// Note: INT_GPIOF defined in inc/hw_ints.h
}

// *******************************************************
void
initDisplay (void)
{
  // intialise the OLED display
  RIT128x96x4Init(1000000);	
}


//*****************************************************************************
//
// Function to display the mean interval in usec
//
//*****************************************************************************
void
displayMeanVal (void)
{
    //unsigned long ulClkRate = SysCtlClockGet();
	
    RIT128x96x4StringDraw ("Monitor Pin 27 & 29", 5, 24, 15);
}

//*****************************************************************************
//
// Function to display the interrupt count 
//*****************************************************************************
void
displayIntCnt (void)
{
   char string[30];

   sprintf (string, "state", currentState);
   RIT128x96x4StringDraw (string, 5, 44, 15);
   sprintf (string, "Count A = %d", g_ulIntCntA);
   RIT128x96x4StringDraw (string, 5, 54, 15);
   sprintf (string, "Count B = %d", g_ulIntCntB);
   RIT128x96x4StringDraw (string, 5, 64, 15);
   sprintf (string, "curr State = %d", currentState);
   RIT128x96x4StringDraw (string, 5, 74, 15);
   sprintf (string, "prev State = %d", previousState);
   RIT128x96x4StringDraw (string, 5, 84, 15);
}
//*****************************************************************************

int
main(void)
{

	initClock ();
	initPin ();
	initDisplay ();

	//
	// Enable interrupts to the processor.
	IntMasterEnable ();
    
	while (1)
	{
		//
		// Background task: calculate the mean of the intervals in the 
		//  circular buffer and display it

		displayMeanVal ();
		displayIntCnt ();
	}
}

