/*******************************************************************************
@file			pid_control.c
@author		Samuel Yamoah
@date			25.05.2016
@modified	31.05.2016
@brief		 A PID Control system that will control the duty cycle of the Tail
 					 and Main Motor Output
*******************************************************************************/

/*Proportional (P): c(t) = Kpe(t)
 * Proportional-Intergral-Derivative(PID): c(t) = Kpe(t) + Ki
 *
 * x(t) is the desired height of the helicopter(%)
 * y(t) is the actual height of the helicopter (%)
 * e(t) =x(t) - y(t), the error (%)
 * c(t) is the control output = the duty cycle for the motor
 *
 * The Constants Kp, Ki are arbitrarily set to 1 for the example
 */

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
#include "pid_control.h"


#define MOTOR_DUTY_MAIN 10
#define MOTOR_DUTY_TAIL 10


 double proportionalControl (double error, double Kp)
{
	return error * Kp;

}

 double integralControl (double error, double Ki, double dt)
{
	static double errorIntegrated = 0;

	errorIntegrated += error * dt;

	return errorIntegrated * Ki;
}

 double derivativeControl (double error, double prevError, double Kd, double dt)
{
	double errorDerived = 0;

	errorDerived = (error - prevError) / dt;

	prevError = error;

	return errorDerived * Kd;
}


void PIDControl(int currentHeight, int desiredHeight, int currentYaw, int desiredYaw, double dt, int tail_duty, int main_duty)
{
	float altKp = 1;
	float altKi = 1;
	float altKd = 1.0;
	double altProportion;
	static double altIntergral = 0.0;
	static double altDerivative;
	static double altPrevError = 0.0;
	static signed long altError = 0;


	float yawKp = 1;
	float yawKi = 1;
	float yawKd = 1.0;
	double yawProportion;
	static double yawIntergral = 0.0;
	static double yawDerivative;
	static double yawPrevError = 0.0;
	static signed long yawError = 0;

	altPrevError = altError;
	yawPrevError = yawError;

	altError = desiredHeight - currentHeight;
	yawError = desiredYaw - currentYaw;


	altProportion = proportionalControl(altError, altKp);
	altIntergral = integralControl(altError, altKi, dt);
	altDerivative = derivativeControl(altError, altPrevError, altKd, dt);

	yawProportion = proportionalControl(yawError, yawKp);
	yawIntergral = integralControl(yawError, yawKi, dt);
	yawDerivative = derivativeControl(yawError, yawPrevError, yawKd, dt);


	main_duty = 10 + altProportion + altIntergral + altDerivative;
	tail_duty = yawProportion + yawIntergral + yawDerivative;

	if(main_duty >= 98) main_duty = 98;
	if(main_duty <= 2) main_duty = 2;

	if(tail_duty >= 98) tail_duty = 98;
	if(tail_duty >= 98) tail_duty = 2;

}
