//*****************************************************************************
//
// Milestone2.c - Simple interrupt driven program which samples with ADC0 to
// calculate the height of the helicopter and also yaw detection with an encoder
// on PF5 (pin 27) & PF7 (pin 29)
//		***  Version 2 - Calculates Reference based on newHght ***
//
// Author:  Samuel Yamoah & Josh Burt
// Date Created: 5.5.2016
// Last modified:	6.5.2016
//
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
#include "utils/ustdlib.h"
#include "utils/isqrt.h"
#include "utils/circBuf.h"
#include "stdio.h"
#include "stdlib.h"

//******************************************************************************
// Constants
//******************************************************************************
#define DEGREES 360
#define STATES_ON_DISC 448

#define BUF_SIZE 100
#define SAMPLE_RATE_HZ 10000
#define MILLI_VOLT 1000

#define ADC_REF 3000
#define ADC_MAX 1023
#define ADC_TO_MILLIS(adc) (((adc) * ADC_REF) / ADC_MAX)

//******************************************************************************
// Global variables
//******************************************************************************
int currentState = 1;
int previousState = 1;
int yaw = 0;

static circBuf_t g_inBuffer;		// Buffer of size BUF_SIZE integers (sample values)
static unsigned long g_ulSampCnt;	// Counter for the interrupts

int initialRead = 0; 	// Initial voltage read to calibrate the minimum height of the helicopter


//******************************************************************************
// The interrupt handler for the for SysTick interrupt.
//******************************************************************************

void SysTickIntHandler(void)
{
    // Initiate a conversion
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;
}

//******************************************************************************
// Determines the yaw movement based on state changes. If the state change
// between previousState and currentState is a positive increment, then yaw is
// is increased. If the currentState is the same as the previousState then no
// change in yaw else yaw is decreased.
//******************************************************************************
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

//******************************************************************************
// The interrupt handler for the for the pin change interrupt. Note that
//  the SysTick counter is decrementing.
//******************************************************************************
void YawChangeIntHandler (void)
{
	/*States:
	 * A 1 = 00
	 * B 2 = 01
	 * C 3 = 11
	 * D 4 = 10
	 */

	unsigned long ulPortValA;
	unsigned long ulPortValB;

	// Clear the interrupt (documentation recommends doing this early)
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_5 |  GPIO_PIN_7);

	// Read the pin
	ulPortValA = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_5);
	ulPortValB = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_7);
	previousState = currentState;

	if (!ulPortValA)
	{
		if (ulPortValB)
		{
			currentState = 2;
		}
		else
		{
			currentState = 1;
		}
	}
	else
	{
		if (ulPortValB)
		{
			currentState = 3;
		}
		else
		{
			currentState = 4;
		}
	}

	yawCalc();
}

//******************************************************************************
// The handler for the ADC conversion (height) complete interrupt.
// Writes to the circular buffer.
//*****************************************************************************
void HeightIntHandler(void)
{
	unsigned long ulValue;
	static int counter = 0;      //Keeping track of the buffer count for the initialRead
	int current;
	int sum = 0;
	int i;

	// Clean up, clearing the interrupt
	ADCIntClear(ADC0_BASE, 3);

	// Get the single sample from ADC0. (Yes, I know, I just did what you did sir :p)
	ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);

	// Place it in the circular buffer (advancing write index)
	g_inBuffer.data[g_inBuffer.windex] = (int) ulValue;
	g_inBuffer.windex++;
	if (g_inBuffer.windex >= g_inBuffer.size)
		g_inBuffer.windex = 0;

	if (counter < BUF_SIZE) {
			counter++;
			if (counter == BUF_SIZE) {
				for (i = 0; i < BUF_SIZE; i++) {
					current = ulValue;
					sum = sum + current;
				}
        //Average voltage to calibrate the minimum height of the helicopter
				initialRead = ADC_TO_MILLIS(sum/BUF_SIZE);
			}
		}
}

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void initClock (void)
{
  // Set the clock rate. From Section 19.1 in stellaris_peripheral_lib_UG.doc:
  //  "In order to use the ADC, the PLL must be used; the PLL output will be
  //  used to create the clock required by the ADC." ADC rate = 8 MHz / 10.
  //  The processor clock rate = 20 MHz.
  SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);

  // Set up the period for the SysTick timer.  The SysTick timer period is
  // set as a function of the system clock.
  SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);

  // Register the interrupt handler
  SysTickIntRegister(SysTickIntHandler);
  //
  // Enable interrupt and device
  SysTickIntEnable();
  SysTickEnable();
}

// *****************************************************************************
void initYaw (void)
{
    // Register the handler for Port F into the vector table
    GPIOPortIntRegister (GPIO_PORTF_BASE, YawChangeIntHandler);

    // Enable and configure the port and pin used:  input on PF5: Pin 27 & PF7: Pin 29
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    GPIOPadConfigSet (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_STRENGTH_2MA,
       GPIO_PIN_TYPE_STD_WPU);

    // Set up the pin change interrupt (both edges)
    GPIOIntTypeSet (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_BOTH_EDGES);

    // Enable the pin change interrupt
    GPIOPinIntEnable (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7);
    IntEnable (INT_GPIOF);	// Note: INT_GPIOF defined in inc/hw_ints.h
}

//******************************************************************************
void initADC (void)
{
  // The ADC0 peripheral must be enabled for configuration and use.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

  // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
  // will do a single sample when the processor sends a signal to start the
  // conversion.
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

  // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
  // single-ended mode (default) and configure the interrupt flag
  // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
  // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
  // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
  // sequence 0 has 8 programmable steps.  Since we are only doing a single
  // conversion using sequence 3 we will only configure step 0.  For more
  // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);

  // Since sample sequence 3 is now configured, it must be enabled.
  ADCSequenceEnable(ADC0_BASE, 3);

  // Register the interrupt handler
  ADCIntRegister (ADC0_BASE, 3, HeightIntHandler);

  // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
  ADCIntEnable(ADC0_BASE, 3);
}

//******************************************************************************
void initDisplay (void)
{
  // intialise the OLED display
  RIT128x96x4Init(1000000);
}

//******************************************************************************
//Calculates the current height of the helicopter as a ratio based on the
// reference and the current height. The returned value is a percentage.
//******************************************************************************
int calcHeight(int reference, int current)
{
	int height;

	height = (((reference - current)  * 100) / (MILLI_VOLT));

	return height;
}

//******************************************************************************
//The yaw is converted to degrees based on the number of slots on the encoder
// disc (can be adapted to other encoder discs). STATES_ON_DISC is = no. of
// states (4) * slots on the disc (112).
//******************************************************************************
int yawToDeg ()
{
	int deg = 0;
	deg = ((yaw * DEGREES + (STATES_ON_DISC / 2)) / STATES_ON_DISC);

	return deg;
}

//*****************************************************************************
// Function to display the displays the current height (mili Volts), reference
// height (mili Volts) and the current height as a percentage.
//*****************************************************************************
void displayInfo(int newHght, int inital, int height, int degrees)
{
	char string[40];

	sprintf(string, "prev state: %3d ", previousState);
	RIT128x96x4StringDraw(string, 5, 14, 15);
	sprintf(string, "curr state: %3d ", currentState);
	RIT128x96x4StringDraw(string, 5, 24, 15);

	sprintf(string, "Init. Hgt = %3dmV ", inital);
	RIT128x96x4StringDraw(string, 5, 44, 15);
	sprintf(string, "curr. Hgt = %3dmV ", newHght);
	RIT128x96x4StringDraw(string, 5, 54, 15);
	sprintf(string, "Hgt. (%%) = %d%%    ", height);
	RIT128x96x4StringDraw(string, 5, 64, 15);

	sprintf (string, "yaw: %5d", yaw);
	RIT128x96x4StringDraw (string, 5, 74, 15);
	sprintf (string, "Deg = %4d", degrees);
	RIT128x96x4StringDraw (string, 5, 84, 15);
}

int main(void)
{
	unsigned int i;
	int sum = 0;
	int current = 0;
	int hgt = 0;
	int degrees = 0;

	initClock ();
	initADC ();
	initYaw ();
	initDisplay ();
	initCircBuf (&g_inBuffer, BUF_SIZE);


	// Enable interrupts to the processor.
	IntMasterEnable();

	while (1)
	{
		degrees = yawToDeg();

		// Background task: calculate the (approximate) mean of the values in the
		// circular buffer and display it.
		sum = 0;
		for (i = 0; i < BUF_SIZE; i++) {
			current = readCircBuf (&g_inBuffer);
			sum = sum + current;

		}
		int newHght = ADC_TO_MILLIS(sum/BUF_SIZE);
		if(initialRead != 0)
		{
			hgt = calcHeight(initialRead, newHght);
			displayInfo(newHght, (int)initialRead, hgt, degrees);
		}
	}
}
