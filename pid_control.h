/*
 @file		pid_control.h
 @author	Samuel Yamoah
 @date		31.05.2016
 @brief		Module calculates the PID Control for the duty cyvle of the motors*
 */

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


static double proportionalControl (double error, double Kp);



static double integralControl (double error, double Ki, double dt);


static double derivativeControl (double error, double Kd, double dt);


void PIDControl(int currentHeight, int currrentYaw, double dt);

#endif /* PID_CONTROL_H_ */
