//*****************************************************************************
//
// Altitude.c - Simple interrupt driven program which samples with ADC0 to
// calculate the height of the helicopter
//		***  Version 2 - Calculates Reference based on newHght ***
//
// Author:  Samuel Yamoah
// Last modified:	5.5.2016
//
//*****************************************************************************
// The setup for the ADC is based on Steve Weddell's 'my_adc.c'.
//*****************************************************************************

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
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

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 100
#define SAMPLE_RATE_HZ 10000
#define MILLI_VOLT 1000

#define ADC_REF 3000
#define ADC_MAX 1023
#define ADC_TO_MILLIS(adc) (((adc) * ADC_REF) / ADC_MAX)

//******************************************************************************
// Global variables
//******************************************************************************
static circBuf_t g_inBuffer;		// Buffer of size BUF_SIZE integers (sample values)
static unsigned long g_ulSampCnt;	// Counter for the interrupts

int initialRead = 0; 				// Initial voltage read to calibrate the minimum height of the helicopter

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
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//*****************************************************************************
void ADCIntHandler(void)
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
  ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

  // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
  ADCIntEnable(ADC0_BASE, 3);
}

//******************************************************************************
//Calculates the current height of the helicopter as a ratio based on the
// reference and the current height. The returned value is a percentage.
//******************************************************************************
int calcHeight(int reference, int current)
{
	int height;

	height = (((current - reference)  * 100) / (MILLI_VOLT - reference));

	return height;
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
void displayInfo(int newHght, int inital, int height)
{
	char string[40];

	RIT128x96x4StringDraw("Height sensor", 5, 14, 15);
	usprintf(string, "curr. Hgt = %3dmV ", newHght);
	RIT128x96x4StringDraw(string, 5, 44, 15);
	usprintf(string, "Init. Hgt = %3dmV ", inital);
	RIT128x96x4StringDraw(string, 5, 64, 15);
	usprintf(string, "Hgt. = %d%%    ", height);
	RIT128x96x4StringDraw(string, 5, 74, 15);

}


int
main(void)
{
	unsigned int i;
	int sum;
	int current;
	int hgt;

	initClock ();
	initADC ();
	initDisplay ();
	initCircBuf (&g_inBuffer, BUF_SIZE);


  // Enable interrupts to the processor.
  IntMasterEnable();

	while (1)
	{
		// Background task: calculate the (approximate) mean of the values in the
		// circular buffer and display it.
		sum = 0;
		for (i = 0; i < BUF_SIZE; i++) {
			current = readCircBuf (&g_inBuffer);
			sum = sum + current;

		}
		int newHght = ADC_TO_MILLIS(sum/BUF_SIZE);
		if(initialRead != 0){
			hgt = calcHeight(initialRead, newHght);
			displayInfo(newHght, (int)initialRead, hgt);
		}
	}
}
