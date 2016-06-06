/*******************************************************************************
@file			altitude.c
@author		Samuel Yamoah
@date			06.06.2016
@modified	06.06.2016
@brief		 Initialising ADC and height conversion sensor
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

#include "altitude.h"
#include "stuff.h"

static circBuf_t g_inBuffer;		// Buffer of size BUF_SIZE integers (sample values)
//initialRead = 0; 	// Initial voltage read to calibrate the minimum height of the helicopter

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
//Calculates the current height of the helicopter as a ratio based on the
// reference and the current height. The returned value is a percentage.
//******************************************************************************
int calcHeight(int reference, int current)
{
	int height;

	height = (((reference - current)  * 100) / (MILLI_VOLT));

	return height;
}
