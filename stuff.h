/*
 * stuff.h
 *
 *  Created on: Jun 5, 2016
 *      Author: sya57
 */

#ifndef STUFF_H_
#define STUFF_H_

extern  volatile signed int main_duty;
extern  volatile signed int tail_duty;

extern int desiredHeight;
extern int desiredYaw;

extern unsigned long period;

extern volatile signed long yawError;
extern volatile signed long altError;

extern int initialRead; 	// Initial voltage read to calibrate the minimum height of the helicopter



#endif /* STUFF_H_ */
