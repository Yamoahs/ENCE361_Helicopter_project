/*******************************************************************************
@file			yaw_control.c
@author		Samuel Yamoah
@date			03.06.2016
@modified	03.06.2016
@brief		 A Yaw Control system that will control the yaw of the helicopter via
           the Tail Motor
*******************************************************************************/

//******************************************************************************
// Determines the yaw movement based on state changes. If the state change
// between previousState and currentState is a positive increment, then yaw is
// is increased. If the currentState is the same as the previousState then no
// change in yaw else yaw is decreased.
//******************************************************************************
void yawCalc (void)
{
	if (previousState == 1 && currentState == 2 ||
      previousState == 2 && currentState == 3	||
      previousState == 3 && currentState == 4 ||
      previousState == 4 && currentState == 1)
	{
		yaw++;
	}
	else if (currentState == previousState)
	{
		yaw = yaw;
	}
	else
	{
		yaw--;
	}
}

//******************************************************************************
// The interrupt handler for the for the pin change interrupt. Note that
//  the SysTick counter is decrementing.
//******************************************************************************
void YawChangeIntHandler (void)
{
	/*States:
	 * A 1 = 00
	 * B 2 = 01
	 * C 3 = 11
	 * D 4 = 10
	 */

	unsigned long ulPortValA;
	unsigned long ulPortValB;

	// Clear the interrupt (documentation recommends doing this early)
	GPIOPinIntClear (GPIO_PORTF_BASE, GPIO_PIN_5 |  GPIO_PIN_7);

	// Read the pin
	ulPortValA = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_5);
	ulPortValB = GPIOPinRead (GPIO_PORTF_BASE, GPIO_PIN_7);
	previousState = currentState;

	if (!ulPortValA)
	{
		if (ulPortValB)
		{
			currentState = 2;
		}
		else
		{
			currentState = 1;
		}
	}
	else
	{
		if (ulPortValB)
		{
			currentState = 3;
		}
		else
		{
			currentState = 4;
		}
	}

	yawCalc();
}

// *****************************************************************************
void initYaw (void)
{
    // Register the handler for Port F into the vector table
    GPIOPortIntRegister (GPIO_PORTF_BASE, YawChangeIntHandler);

    // Enable and configure the port and pin used:  input on PF5: Pin 27 & PF7: Pin 29
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    GPIOPadConfigSet (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPU);

    // Set up the pin change interrupt (both edges)
    GPIOIntTypeSet (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7, GPIO_BOTH_EDGES);

    // Enable the pin change interrupt
    GPIOPinIntEnable (GPIO_PORTF_BASE, GPIO_PIN_5 | GPIO_PIN_7);
    IntEnable (INT_GPIOF);	// Note: INT_GPIOF defined in inc/hw_ints.h
}

//******************************************************************************
//The yaw is converted to degrees based on the number of slots on the encoder
// disc (can be adapted to other encoder discs). STATES_ON_DISC is = no. of
// states (4) * slots on the disc (112).
//******************************************************************************
int yawToDeg ()
{
	int deg = 0;
	deg = ((yaw * DEGREES + (STATES_ON_DISC / 2)) / STATES_ON_DISC);

	return deg;
}
