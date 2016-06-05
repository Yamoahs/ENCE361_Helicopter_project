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


// PID Control, controls the helicopters response to the pushbuttons.
void PIDControl(int hgt_percent, double dt, long yawError, long altError,
                int main_duty, int tail_duty, int desiredYaw, int desiredHeight, int yaw)
{
	static signed long yawErrorPrev;
	static signed long yawIntegral;
	static signed long yawDerivative;
	static signed long altErrorPrev;
	static signed long altIntegral;
	static signed long altDerivative;

	// initialising the gain variables
	float altKp;
	float altKi;
	float altKd;
	float yawKp;
	float yawKi;
	float yawKd;

	// PID gain values
	altKp = 0.5; // altitude proportional gain, decreases the response to system disturbances
	altKi = 0.0009; // altitude integral gain, eliminates steady state error
	altKd = 0.8; // altitude derivative gain, accounts for overshoot produced by ki and kp

	yawKp = 0.6; // yaw proportional gain, decreases the response to system disturbances
	yawKi = 0.009; // yaw integral gain, eliminates steady state error
	yawKd = 2.5; // yaw derivative gain, accounts for overshoot produced by ki and kp

	altErrorPrev = altError;
	yawErrorPrev = yawError;

	altError = desiredHeight - hgt_percent; // positive if up
	yawError = desiredYaw - yaw; // positive if cw

	yawIntegral += yawError;
	altIntegral += altError;
	yawDerivative = (yawError-yawErrorPrev);
	altDerivative = (altError-altErrorPrev) / dt;


	main_duty = 10 + altKp*altError + altKi*altIntegral + altKd*altDerivative;
	tail_duty = ((main_duty * 40)/50) + yawKp*yawError + yawKi*yawIntegral + yawKd*yawDerivative;
	if(hgt_percent == desiredHeight) altIntegral = 0;
	if(yaw == desiredYaw) yawIntegral = 0;

	// controlling the duty cycle for the motors so that they are between the specified percentages
	if (main_duty > 98) {
		main_duty = 98;
	}
	if (tail_duty > 98) {
		tail_duty = 98;
	}
	if (main_duty < 2) {
		main_duty = 2;
	}
	if (tail_duty < 2) {
		tail_duty = 2;
	}

}
