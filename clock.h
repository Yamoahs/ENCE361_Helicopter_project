/*******************************************************************************
@file			clock.c
@author		Samuel Yamoah
@date			06.06.2016
@modified	06.06.2016
@brief		 Initialising Microprocessor Clock
*******************************************************************************/
#ifndef CLOCK_H_
#define CLOCK_H_

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


#define SYSTICK_RATE_HZ 1000ul
#define SAMPLE_RATE_HZ 10000

//******************************************************************************
// The interrupt handler for the for SysTick interrupt.
//******************************************************************************

void SysTickIntHandler(void);

void initClock (void);



#endif /* CLOCK_H_ */
