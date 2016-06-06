/*******************************************************************************
@file			yaw_control.h
@author			Samuel Yamoah
@date			05.06.2016
@modified		03.06.2016
@brief			A Yaw Control system that will control the yaw of the helicopter
				via the Tail Motor
*******************************************************************************/

#ifndef YAW_CONTROL_H_
#define YAW_CONTROL_H_

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

#define DEGREES 360
#define STATES_ON_DISC 448


/* Determines the yaw movement based on state changes and keeps track of changes as yaw.*/
void yawCalc (int previousState, int currentState, int yaw);


/*The yaw is converted to degrees based on the number of slots on the encoder
 disc (can be adapted to other encoder discs). STATES_ON_DISC is = no. of
 states (4) * slots on the disc (112).*/
int yawToDeg (int yaw);

void YawChangeIntHandler (void);

void initYaw (void);


#endif /* YAW_CONTROL_H_ */
