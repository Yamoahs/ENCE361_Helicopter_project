/*******************************************************************************
 @file	   	pid_control.h
 @author	  Samuel Yamoah
 @date		  31.05.2016
 @modified  31.05.2016
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

/*Function Calculates the Proportional part of the PID Control*/
 double proportionalControl (double error, double Kp);

/*Function Calculates the Intergral part of the PID Control*/
 double integralControl (double error, double Ki, double dt);

/*Function Calculates the Derivative part of the PID Control*/
 double derivativeControl (double error, double prevError, double Kd, double dt);

/*Function Combines Proportional, Intergral & Derivative parts of the PID Control*/
void PIDControl(int currentHeight, int desiredHeight, int currrentYaw, int desiredYaw, double dt, int tail_duty, int main_duty);

#endif /* PID_CONTROL_H_ */
