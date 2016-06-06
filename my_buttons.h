/*******************************************************************************
@file			my_buttons.h
@author		Samuel Yamoah
@date			06.06.2016
@modified	06.06.2016
@brief		 Initialising and controlling the output of the virtual Buttons on
			port B
*******************************************************************************/

#ifndef MY_BUTTONS_H_
#define MY_BUTTONS_H_

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

#define MOTOR_RATE_HZ 200
#define PWM_DIVIDER 4
#define MOTOR_DUTY_MAIN 10
#define MOTOR_DUTY_TAIL 10

void ButtPressIntHandler ();


void intButton ();

#endif /* MY_BUTTONS_H_ */
