/*******************************************************************************
 @file	  pid_control.h
 @author	Samuel Yamoah
 @date		31.05.2016
@modified	07.06.2016
 @brief		Module calculates the PID Control for the duty cyvle of the motors*
 ******************************************************************************/

#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_


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

/*function calculates the PID control from a passed in varaibles hgt_percent and
 dt. YawError, altError, main_duty, tail_duty, desiredYaw & desiredHeight are
 all globals in the main program.*/
void PIDControl(int hgt_percent, double dt, long yawError, long altError,
                int main_duty, int tail_duty, int desiredYaw, int desiredHeight, int yaw);

#endif /* PID_CONTROL_H_ */
