//*****************************************************************************
//
// Milestone2_MotorControl.c - Simple interrupt driven program which samples with ADC0 to
// calculate the height of the helicopter and also yaw detection with an encoder.
// Also basic motor output controll now added.
// on PF5 (pin 27) & PF7 (pin 29)
//		***  Version 1 - Calculates Reference based on newHght ***
//
// Author:  Samuel Yamoah & Josh Burt
// Date Created: 22.5.2016
// Last modified:	22.5.2016
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
#include "buttonSet.h"
#include "button.h"

#include "driverlib/uart.h"
#include "utils/uartstdio.h"


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

#define SYSTICK_RATE_HZ 1000ul
#define MOTOR_RATE_HZ 200
#define PWM_DIV_CODE SYSCTL_PWMDIV_4
#define PWM_DIVIDER 4
#define MOTOR_DUTY_MAIN 80
#define MOTOR_DUTY_TAIL 2

#define BAUD_RATE 9600ul

//******************************************************************************
// Global variables
//******************************************************************************
int currentState = 1;
int previousState = 1;
int yaw = 0;

static circBuf_t g_inBuffer;		// Buffer of size BUF_SIZE integers (sample values)
static unsigned long g_ulSampCnt;	// Counter for the interrupts

int initialRead = 0; 	// Initial voltage read to calibrate the minimum height of the helicopter

signed int main_duty = 0;
signed int tail_duty = 0;

// Compute the PWM period in terms of the PWM clock
unsigned long period;



//******************************************************************************
// The interrupt handler for the for SysTick interrupt.
//******************************************************************************

void SysTickIntHandler(void)
{
    // Initiate a conversion
    ADCProcessorTrigger(ADC0_BASE, 3);
    //Polls display on UART0
    g_ulSampCnt++;

    // Poll the buttons
	updateButtons();
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

void
ButtPressIntHandler(void)
{
	unsigned long ulSelect; //SELECT
	unsigned long ulUp; //UP
	unsigned long ulDown; //DOWN
	unsigned long ulCww; //CCW/RIGHT
	unsigned long ulCw; //CW/LEFT
	unsigned long ulReset; //RESET
	int power = 0;

    // Clear the interrupt (documentation recommends doing this early)
    GPIOPinIntClear (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

	// Read the pins simultaneously to ensure the right states are read
    ulReset = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_1);
    ulCw = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_2);
    ulCww = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_3);
    ulSelect = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_4);
    ulUp = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_5);
    ulDown = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_6);

    period = SysCtlClockGet () / PWM_DIVIDER / MOTOR_RATE_HZ;

    if(!ulUp && main_duty < 98){
    	main_duty += 10;
    	if (main_duty >= 98) main_duty = 98;
    	PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);
    }

    if (!ulDown && main_duty > 2){
    	main_duty -= 10;
    	if (main_duty <= 2) main_duty = 2;
    	PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);
    }

    if(!ulCww && tail_duty < 98){
    	tail_duty += 15;
    	if (tail_duty >= 98) tail_duty = 98;
    	PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
    }

    if (!ulCw && tail_duty > 2){
		  tail_duty -= 15;
		  if (tail_duty <= 2) tail_duty = 2;
		  PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
	   }

    if (ulSelect){
    	if (power == 0){
    		main_duty = MOTOR_DUTY_MAIN;
    		tail_duty = MOTOR_DUTY_TAIL;
    		power == 1;
    	}
    	if (power == 1){
			main_duty = 10;
			tail_duty = 10;
			power == 0;
		}
    }

    if (!ulReset) SysCtlReset();

	/*if (!ulPortValA) {
		 SysCtlReset();
	}
	if (!ulPortValB && STATE == FLYING) {
		degFinal += 15 ;
				}
	if (!ulPortValC && STATE == FLYING) {
		degFinal -= 15;
				}
	if (!ulPortValD) {
		if (STATE == IDLE) {
			PWMOutputState (PWM_BASE, PWM_OUT_4_BIT, true);
			PWMOutputState (PWM_BASE, PWM_OUT_1_BIT, true);
			STATE = ORIENTATING;
		}
		if (STATE == FLYING) {
			STATE = LANDING; // reset 2 = landing
			degFinal = 0;
			altFinal = 0;
		}
		}
	if (!ulPortValE && altFinal < 950 && STATE == FLYING) {
		altFinal += 100;
				}
	if (!ulPortValF && altFinal > 50 && STATE == FLYING) {
		altFinal -= 100;
				}*/

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

//*******************************************************************
// Initialise the SysTick interrupts
//*******************************************************************
void initSysTick (void)
{
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet (SysCtlClockGet() / SYSTICK_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister (SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable ();
    SysTickEnable ();
}

//******************************************************************
// Initialise the GPIO for the PWM output (Port D and Port F)
//******************************************************************
void initMotorPins (void)
{
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM (GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinTypePWM (GPIO_PORTF_BASE, GPIO_PIN_2);
}

//******************************************************************
// Initialise the PWM generator (PWM1 & PWM4)
//******************************************************************
void initPWMchan (void)
{
	unsigned long period;

    SysCtlPeripheralEnable (SYSCTL_PERIPH_PWM);
    //
    // Compute the PWM period based on the system clock.
    //
        SysCtlPWMClockSet (PWM_DIV_CODE);

	PWMGenConfigure (PWM_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure (PWM_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	period = SysCtlClockGet () / PWM_DIVIDER / MOTOR_RATE_HZ;
	PWMGenPeriodSet (PWM_BASE, PWM_GEN_0, period);
	PWMGenPeriodSet (PWM_BASE, PWM_GEN_2, period);
	PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * MOTOR_DUTY_MAIN / 100);
	PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * MOTOR_DUTY_TAIL / 100);
    //
    // Enable the PWM output signal.
    //
    PWMOutputState (PWM_BASE, PWM_OUT_1_BIT, true);
    PWMOutputState (PWM_BASE, PWM_OUT_4_BIT, true);
    //
    // Enable the PWM generator.
    //
    PWMGenEnable (PWM_BASE, PWM_GEN_0);
    PWMGenEnable (PWM_BASE, PWM_GEN_2);
}

// *****************************************************************************
void initYaw (void)
{
    // Register the handler for Port F into the vector table
    GPIOPortIntRegister (GPIO_PORTF_BASE, YawChangeIntHandler);

    // Enable and configure the port and pin used:  input on PF5: Pin 27 & PF7: Pin 29
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    GPIOPadConfigSet (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Set up the pin change interrupt (both edges)
    GPIOIntTypeSet (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_BOTH_EDGES);

    // Enable the pin change interrupt
    GPIOPinIntEnable (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7);
    IntEnable (INT_GPIOF);	// Note: INT_GPIOF defined in inc/hw_ints.h
}

//*******************************************************************************
void intButtons (void)
{
	// Register the handler for Port B into the vector table
	GPIOPortIntRegister (GPIO_PORTB_BASE, ButtPressIntHandler);

	//Initialising for buttons
	GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
										   GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
	GPIOPadConfigSet (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
						GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
					GPIO_PIN_5 | GPIO_PIN_6, GPIO_FALLING_EDGE);
	GPIOPinIntEnable (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
					  GPIO_PIN_5 | GPIO_PIN_6);

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
void displayInfo(int height, int degrees, int main, int tail)
{
	char string[40];

	sprintf(string, "main cycle: %2d%%  ", main);
	RIT128x96x4StringDraw(string, 5, 34, 15);
	sprintf(string, "tail cycle: %2d%%  ", tail);
	RIT128x96x4StringDraw(string, 5, 44, 15);

	sprintf(string, "Hgt. (%%) = %d%%    ", height);
	RIT128x96x4StringDraw(string, 5, 64, 15);

	sprintf (string, "yaw: %5d", yaw);
	RIT128x96x4StringDraw (string, 5, 74, 15);
	sprintf (string, "Deg = %4d", degrees);
	RIT128x96x4StringDraw (string, 5, 84, 15);

	if ((g_ulSampCnt % 100) == 0){
		usprintf(string, "YEah It\n");
		UARTSend (string);
		usprintf(string, "main: %d\n", main);
		UARTSend (string);
		usprintf(string, "tail: %d\n", tail);
		UARTSend (string);
		usprintf(string, "-----------------------\n");
		UARTSend (string);
	}
}

int main(void)
{
	unsigned int i;
	int sum = 0;
	int current = 0;
	int hgt_percent = 0;
	int degrees = 0;

	//signed int main_duty = MOTOR_DUTY_MAIN;
	//signed int tail_duty = MOTOR_DUTY_TAIL;
	unsigned long period;

	initClock ();
	initSysTick ();
	initADC ();
	initYaw ();
	intButtons ();
	initMotorPins ();
	initConsole ();
	initDisplay ();

	initButSet (UP_B | DOWN_B | LEFT_B | RIGHT_B, SYSTICK_RATE_HZ);
	initPWMchan ();

	initCircBuf (&g_inBuffer, BUF_SIZE);



	// Enable interrupts to the processor.
	IntMasterEnable();

	// Compute the PWM period in terms of the PWM clock
	period = SysCtlClockGet () / PWM_DIVIDER / MOTOR_RATE_HZ;

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
			hgt_percent = calcHeight(initialRead, newHght);
		}

	   if (checkBut (UP) && main_duty < 98)
	   {
		  main_duty += 10;
		  if (main_duty >= 98) main_duty = 98;
		  PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);

	   }
	   if (checkBut (DOWN) && main_duty > 2)
	   {
		  main_duty -= 10;
		  if (main_duty <= 2) main_duty = 2;
		  PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);
	   }

	   if (checkBut (RIGHT) && tail_duty < 98)
	   {
		  tail_duty += 15;
		  if (tail_duty >= 98) tail_duty = 98;
		  PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
	   }
	   if (checkBut (LEFT) && tail_duty > 2)
	   {
		  tail_duty -= 15;
		  if (tail_duty <= 2) tail_duty = 2;
		  PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
	   }

	   	displayInfo(hgt_percent, degrees, main_duty, tail_duty);
	}
}
