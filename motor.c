/*******************************************************************************
@file			motor.c
@author		Samuel Yamoah
@date			06.06.2016
@modified	07.06.2016
@brief		Initialising PWM for the tail and main Motor output
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

#include "motor.h"



/*Initialising the pins for motor output*/
void initMotorPin (void)
{
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM (GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinTypePWM (GPIO_PORTF_BASE, GPIO_PIN_2);
}

//******************************************************************
// Initialise the PWM generator (PWM1 & PWM4)
//******************************************************************
void initPWMchan (signed int main_duty, signed int tail_duty)
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
    PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty / 100);
    PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty / 100);
    //
    // Enable the PWM output signal.
    //
    PWMOutputState (PWM_BASE, PWM_OUT_1_BIT, false);
    PWMOutputState (PWM_BASE, PWM_OUT_4_BIT, false);
    //
    // Enable the PWM generator.
    //
    PWMGenEnable (PWM_BASE, PWM_GEN_0);
    PWMGenEnable (PWM_BASE, PWM_GEN_2);
}
