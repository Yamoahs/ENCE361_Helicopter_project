/*******************************************************************************
@file			my_buttons.c
@author		Samuel Yamoah
@date			06.06.2016
@modified	06.06.2016
@brief		 Initialising and controlling the output of the virtual Buttons on
			port B
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

#include "my_buttons.h"
#include "globals.h"
#include "pid_control.h"
#include "yaw_control.h"
#include "display.h"

int desiredHeight = 0;
int desiredYaw = 0;
volatile signed int main_duty = 0;
volatile signed int tail_duty = 0;

void ButtPressIntHandler ()
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
	unsigned long period = SysCtlClockGet () / PWM_DIVIDER / MOTOR_RATE_HZ;

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

void intButton ()
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
