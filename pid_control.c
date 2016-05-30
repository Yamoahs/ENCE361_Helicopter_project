//************************************************************************
//
//pid_control.c - A PID Control system that will control the duty cycle of
//				  the Tail and Main Motor Output
//
// Author:  Samuel Yamoah
// Date Created: 25.5.2016
// Last modified: 30.5.2016
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


void PIDControl(int currentHeight, double dt)
{
	float Kp = 1;
	float Ki = 1;
	float Kd;
	double proportion;
	static double intergral = 0.0;
	static double derivative;
	static double prevError = 0.0;
	static signed long error = desiredHeight - currentHeight;
	prevError = error;


	proportion = proportionalControl(error, Kp);
	intergral = integralControl(error, Ki, dt);


	main_duty = proportion + intergral;

}


