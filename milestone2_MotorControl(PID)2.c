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
