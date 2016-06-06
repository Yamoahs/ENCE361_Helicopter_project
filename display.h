/*******************************************************************************
@file			display.h
@author			Samuel Yamoah
@date			05.06.2016
@modified		05.06.2016
@brief			Set up of the UART Display and Stellaris
*******************************************************************************/

#ifndef SERIAL_H_
#define SERIAL_H_

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

#define BAUD_RATE 9600ul


void initConsole (void);


void UARTSend (char *pucBuffer);

void initDisplay (void);

//*****************************************************************************
// Function to display the displays the current height (mili Volts), reference
// height (mili Volts) and the current height as a percentage.
//*****************************************************************************
void displayInfo(int inital, int height, int degrees, long g_ulSampCnt, long altError, int desiredHeight, int desiredYaw, int main_duty, int tail_duty);

#endif /* SERIAL_H_ */
