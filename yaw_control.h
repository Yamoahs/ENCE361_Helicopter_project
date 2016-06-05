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



/* Determines the yaw movement based on state changes.*/
void yawCalc (void);



/*The interrupt handler for the for the pin change interrupt. Note that
the SysTick counter is decrementing.*/
void YawChangeIntHandler (void);


/* Initialising the Pins and interrupt for the yaw*/
void initYaw (void);


/*The yaw is converted to degrees based on the number of slots on the encoder
 disc (can be adapted to other encoder discs). STATES_ON_DISC is = no. of
 states (4) * slots on the disc (112).*/
int yawToDeg ();

#endif /* YAW_CONTROL_H_ */
