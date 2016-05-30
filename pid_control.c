//************************************************************************
//
//pid_control.c - A PID Control system that will control the duty cycle of
//				  the Tail and Main Motor Output
//
// Author:  Samuel Yamoah
// Date Created: 25.5.2016
// Last modified: 31.5.2016
//
//************************************************************************
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

static int desiredHeight = 0;
static int desiredYaw = 0;
double dt = SysCtlClockGet() / SYSTICK_RATE_HZ;


static double proportionalControl (double error, double, Kp)
{
	return error * Kp;

}

static double integralControl (double error, double, Ki, double dt)
{
	static double errorIntegrated;

	errorInegrated += error * dt;

	return errorIntegrated * Ki;
}

static double derivativeControl (double error, double, Kd, double dt)
{
	double errorDerived;

	errorDerived = (error - prevError) / dt;

	prevError = error;

	return errorDerived * Kd;
}


void PIDControl(int currentHeight, int currrentYaw, double dt)
{
	float altKp = 1;
	float altKi = 1;
	float altKd = 1.0;
	double altProportion;
	static double altIntergral = 0.0;
	static double altDerivative;
	static double altPrevError = 0.0;
	static signed long altError = desiredHeight - currentHeight;

	float yawKp = 1;
	float yawKi = 1;
	float yawKd = 1.0;
	double yawProportion;
	static double yawIntergral = 0.0;
	static double yawDerivative;
	static double yawPrevError = 0.0;
	static signed long yawError = desiredYaw - currentYaw;



	altProportion = proportionalControl(AltError, altKp);
	altIntergral = integralControl(altError, altKi, dt);
	altDerivative = derivativeControl(altError, prevError, kd, dt);

	yawProportion = proportionalControl(yawError, yawKp);
	yawIntergral = integralControl(yawError, yawKi, dt);
	yawDerivative = derivativeControl(yawError, yawPrevError, yawKd, dt);

	altPrevError = altError;
	yawPrevError = yawError;


	main_duty = altProportion + altIntergral + altDerivative;
	tail_duty = yawProportion + yawIntergral + yawDerivative;

}
