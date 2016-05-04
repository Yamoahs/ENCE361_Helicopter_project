/* YAW TO DEREES INTEGER DIVISION BUG FIX - 02/05/16
*  #define DEGREES 360
*  #define STATES_ON_DISC 448
*
* deg = (yaw * DEGREES + (STATES_ON_DISC / 2)) / STATES_ON_DISC
*/

//*****************************************************************************
//
// yaw_calc.c - Program to monitor the change in yaw from two encoders.
//
// Author: 			Samuel Yamoah
// Date created: 	30.4.2016
// Last modified:	1.5.2016
//
//*****************************************************************************
// Program checks interrupts on Pin 27, PF5 and Pin 29, PF7 with both rising and
// falling edge detection. Each channel interrupt is checked against the other
// channel interrupt to find the rotation state. There are four possible states
// in which the transitions between states will determine the rotation direction.
// The yaw is then converted in to degrees and is continuallously displayed.
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
#define DEGREES 360
#define STATES_ON_DISC 448
//#define STATES_PER_SLOT 4
//#define YAW_DEG 360/112


//*****************************************************************************
// Global variables
//*****************************************************************************
static volatile unsigned long g_ulIntCntA;	// Monitors interrupts on A
static volatile unsigned long g_ulIntCntB;	// Monitors interrupts on B
int currentState = 1;
int previousState = 1;
int yaw = 0;

//*****************************************************************************
//
// The interrupt handler for the for the pin change interrupt. Note that
//  the SysTick counter is decrementing.
//
//*****************************************************************************

void yawCalc (void)
{
	if (previousState == 1 && currentState == 2 || previousState == 2 && currentState == 3
		|| previousState == 3 && currentState == 4 || previousState == 4 && currentState == 1)
	{
		yaw++;
	}
	else if (currentState == previousState)
	{
		yaw = yaw;
	}
	else
	{
		yaw--;
	}
}

void PinChangeIntHandler (void)
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
	previousState = currentState;
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

	yawCalc();
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
  SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
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

int yawToDeg ()
{
	int deg = 0;
	deg = (yaw * DEGREES + (STATES_ON_DISC / 2)) / STATES_ON_DISC;

	return deg;
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
displayIntCnt (int degrees)
{
   char string[30];

   sprintf (string, "yaw: %5d", yaw);
   RIT128x96x4StringDraw (string, 5, 44, 15);
   sprintf (string, "Deg = %4d", degrees);
   RIT128x96x4StringDraw (string, 5, 54, 15);
   sprintf (string, "curr State = %d", currentState);
   RIT128x96x4StringDraw (string, 5, 74, 15);
   sprintf (string, "prev State = %d", previousState);
   RIT128x96x4StringDraw (string, 5, 84, 15);
}
//*****************************************************************************

int
main(void)
{
	int degrees = 0;

	initClock ();
	initPin ();
	initDisplay ();

	//
	// Enable interrupts to the processor.
	IntMasterEnable ();

	while (1)
	{
		degrees = yawToDeg();
		displayMeanVal ();
		displayIntCnt (degrees);
	}
}
