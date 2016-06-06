/*******************************************************************************
@file			motor.h
@author		Samuel Yamoah
@date			06.06.2016
@modified	07.06.2016
@brief		Initialising PWM for the tail and main Motor output
*******************************************************************************/

#ifndef MOTOR_H_
#define MOTOR_H_

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

#define SYSTICK_RATE_HZ 1000ul
#define MOTOR_RATE_HZ 200
#define PWM_DIV_CODE SYSCTL_PWMDIV_4
#define PWM_DIVIDER 4
#define MOTOR_DUTY_MAIN 10
#define MOTOR_DUTY_TAIL 10

/*Initialising the pins for motor output*/
void initMotorPin (void);

/*Initialise the PWM generator (PWM1 & PWM4)*/
void initPWMchan (signed int main_duty, signed int tail_duty);


#endif /* MOTOR_H_ */
