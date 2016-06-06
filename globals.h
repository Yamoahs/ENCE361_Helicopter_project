/*******************************************************************************
@file			globals.h
@author		Samuel Yamoah
@date			05.06.2016
@modified	07.06.2016
@brief		Global variables that get used by multiple  odules
*******************************************************************************/

#ifndef GLOBALS_H_
#define GLOBALS_H_

extern  volatile signed int main_duty;
extern  volatile signed int tail_duty;

extern int desiredHeight;
extern int desiredYaw;

extern unsigned long period;

extern volatile signed long yawError;
extern volatile signed long altError;

extern int initialRead; 	// Initial voltage read to calibrate the minimum height of the helicopter
extern unsigned long g_ulSampCnt;	// Counter for the interrupts

extern int currentState;
extern int previousState;
extern int yaw;


#endif /* GLOBALS_H_ */
