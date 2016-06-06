//*****************************************************************************
//
// Milestone2_MotorControl.c - Simple interrupt driven program which samples with ADC0 to
// calculate the height of the helicopter and also yaw detection with an encoder
// on PF5 (pin 27) & PF7 (pin 29)
//		***  Version 2 - Calculates Reference based on newHght ***
//
// Author:  Samuel Yamoah
// Date Created: 20.5.2016
// Last modified: 30.5.2016
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
#include "driverlib/uart.h"
#include "drivers/rit128x96x4.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "utils/isqrt.h"
#include "utils/circBuf.h"
#include "stdio.h"
#include "stdlib.h"

#include "my_buttons.h"
#include "pid_control.h"
#include "yaw_control.h"
#include "display.h"
#include "motor.h"
#include "altitude.h"
#include "clock.h"

//******************************************************************************
// Constants
//******************************************************************************

#define SYSTICK_RATE_HZ 1000ul


//******************************************************************************
// Global variables
//******************************************************************************
int currentState = 1;
int previousState = 1;
int yaw = 0;

static circBuf_t g_inBuffer;		// Buffer of size BUF_SIZE integers (sample values)
static unsigned long g_ulSampCnt;	// Counter for the interrupts

int initialRead = 0; 	// Initial voltage read to calibrate the minimum height of the helicopter

static volatile signed int main_duty = 0;
static volatile signed int tail_duty = 0;

unsigned long period;

static int desiredHeight = 0;
static int desiredYaw = 0;

static volatile signed long yawError = 0;
static volatile signed long altError = 0;


void ButtPressIntHandler (void)
{
	unsigned long ulSelect; //SELECT
	unsigned long ulUp; //UP
	unsigned long ulDown; //DOWN
	unsigned long ulCCw; //CCW/RIGHT
	unsigned long ulCw; //CW/LEFT
	unsigned long ulReset; //RESET
	//int power = 0;

	// Clear the interrupt (documentation recommends doing this early)
	GPIOPinIntClear (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

	// Read the pins simultaneously to ensure the right states are read
	ulReset = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_1);
	ulCw = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_2);
	ulCCw = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_3);
	ulSelect = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_4);
	ulUp = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_5);
	ulDown = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_6);

	// Compute the PWM period in terms of the PWM clock
	    period = SysCtlClockGet () / PWM_DIVIDER / MOTOR_RATE_HZ;

	if(ulUp == 0){
		desiredHeight += 10;
	}


	if(ulDown == 0){
		desiredHeight -= 10;
		}

	if(ulCw == 0){
		desiredYaw += 15;
		}

	if(ulCCw == 0){
		desiredYaw -= 15;
		}

	if(ulSelect == 0){
		main_duty = MOTOR_DUTY_MAIN;
		tail_duty = MOTOR_DUTY_TAIL;
		PWMOutputState (PWM_BASE, PWM_OUT_1_BIT, true);
		PWMOutputState (PWM_BASE, PWM_OUT_4_BIT, true);
		PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);
		PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);

	}

	if(ulReset == 0){
		if (!ulReset) SysCtlReset();
		}

}

void intButton (void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// Register the handler for Port B into the vector table
	GPIOPortIntRegister (GPIO_PORTB_BASE, ButtPressIntHandler);

	//Initialising for buttons
	GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
	GPIOPadConfigSet (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_FALLING_EDGE);
	GPIOPinIntEnable (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

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

	yawCalc(previousState, currentState, yaw);
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


int main(void)
{
	unsigned int i;
	int sum = 0;
	int current = 0;
	int hgt_percent = 0;
	int degrees = 0;


	initClock();
	initADC();
	initYaw();
	initMotorPin();
	initDisplay();
	intButton();
	initConsole();
	initPWMchan(main_duty, tail_duty);
	initCircBuf (&g_inBuffer, BUF_SIZE);


	// Enable interrupts to the processor.
	IntMasterEnable();

	while (1)
	{
		//double dt = SysCtlClockGet() / SYSTICK_RATE_HZ;
		degrees = yawToDeg(yaw);


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
		PIDControl(hgt_percent, SysCtlClockGet() / SYSTICK_RATE_HZ, yawError, altError, main_duty, tail_duty, desiredYaw, desiredHeight, yaw);

		PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty / 100);
		PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty / 100);

		displayInfo((int)initialRead, hgt_percent, degrees, g_ulSampCnt, altError, desiredHeight, desiredYaw, main_duty, tail_duty);
	}
}
