/*******************************************************************************
@file			altitude.h
@author		Samuel Yamoah
@date			06.06.2016
@modified	07.06.2016
@brief		Initialising ADC and height conversion sensor
*******************************************************************************/

#ifndef ALTITUDE_H_
#define ALTITUDE_H_

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


#define BUF_SIZE 100
#define MILLI_VOLT 1000

#define ADC_REF 3000
#define ADC_MAX 1023
#define ADC_TO_MILLIS(adc) (((adc) * ADC_REF) / ADC_MAX)

/* The handler for the ADC conversion (height) complete interrupt. Writes to the
 circular buffer.*/
void HeightIntHandler(void);

/*Initialise the ADC*/
void initADC (void);

/*Calculates the current height of the helicopter as a ratio based on the
 reference and the current height. The returned value is a percentage.*/
int calcHeight(int reference, int current);

#endif /* ALTITUDE_H_ */
