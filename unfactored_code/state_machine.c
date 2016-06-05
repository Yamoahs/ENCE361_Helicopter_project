/*
 * state_machine.c
 *
 *  Created on: 29/04/2016
 *      Author: Samuel
 */

/*States:
 * A 1 = 00
 * B 2 = 10
 * C 3 = 11
 * D 4 = 01
 */

int currentState;
int previousState;
int movement = 0;

//Conditions may need to be changed based on setup method

if (encoder_a == 1):
{
	if (encoder_b == 0):
	{
		currentState = 2;
	}

	else:
	{1
		currentState = 3;
	}
}
else:
{
	if (encoder_b == 0):
		{
			currentState = 1;
		}

		else:
		{
			currentState = 4;
		}
}

previousState = currentState;


if (previousState == 1 && currentState == 2 || previousState == 2 && currentState == 3
	|| previousState == 3 && currentState == 4 || previousState == 4 && currentState == 1)
{
	movement++;
}
else
{
	movement--;
}
