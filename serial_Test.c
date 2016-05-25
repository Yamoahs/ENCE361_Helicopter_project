//*****************************************************************************
//
// Milestone2_MotorControl.c - Simple interrupt driven program which samples with ADC0 to
// calculate the height of the helicopter and also yaw detection with an encoder.
// Also basic motor output controll now added.
// on PF5 (pin 27) & PF7 (pin 29)
//		***  Version 1 - Calculates Reference based on newHght ***
//
// Author:  Samuel Yamoah & Josh Burt
// Date Created: 22.5.2016
// Last modified:	22.5.2016
//
//*****************************************************************************

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
#include "drivers/rit128x96x4.h"
#include "utils/ustdlib.h"
#include "utils/isqrt.h"
#include "utils/circBuf.h"
#include "stdio.h"
#include "stdlib.h"
#include "buttonSet.h"
#include "button.h"

#include "driverlib/uart.h"
#include "utils/uartstdio.h"


//******************************************************************************
// Constants
//******************************************************************************

#define SAMPLE_RATE_HZ 10000
#define BAUD_RATE 9600ul
#define SYSTICK_RATE_HZ 1000ul

//******************************************************************************
// Global variables
//******************************************************************************

static unsigned long g_ulSampCnt;	// Counter for the interrupts
signed int main_duty = 0;
signed int tail_duty = 0;

//******************************************************************************
// The interrupt handler for the for SysTick interrupt.
//******************************************************************************

void SysTickIntHandler(void)
{
    // Initiate a conversion
    ADCProcessorTrigger(ADC0_BASE, 3);
    //Polls display on UART0
    g_ulSampCnt++;

    // Poll the buttons
	updateButtons();
}

/*
void
ButtPressIntHandler(void)
{
	unsigned long ulSelect; //SELECT
	unsigned long ulUp; //UP
	unsigned long ulDown; //DOWN
	unsigned long ulCww; //CCW/RIGHT
	unsigned long ulCw; //CW/LEFT
	unsigned long ulReset; //RESET
	int power = 0;

    // Clear the interrupt (documentation recommends doing this early)
    GPIOPinIntClear (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

	// Read the pins simultaneously to ensure the right states are read
    ulReset = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_1);
    ulCw = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_2);
    ulCww = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_3);
    ulSelect = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_4);
    ulUp = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_5);
    ulDown = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_6);

    period = SysCtlClockGet () / PWM_DIVIDER / MOTOR_RATE_HZ;

    if(!ulUp && main_duty < 98){
    	main_duty += 10;
    	if (main_duty >= 98) main_duty = 98;
    	PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);
    }

    if (!ulDown && main_duty > 2){
    	main_duty -= 10;
    	if (main_duty <= 2) main_duty = 2;
    	PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);
    }

    if(!ulCww && tail_duty < 98){
    	tail_duty += 15;
    	if (tail_duty >= 98) tail_duty = 98;
    	PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
    }

    if (!ulCw && tail_duty > 2){
		  tail_duty -= 15;
		  if (tail_duty <= 2) tail_duty = 2;
		  PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
	   }

    if (ulSelect){
    	if (power == 0){
    		main_duty = MOTOR_DUTY_MAIN;
    		tail_duty = MOTOR_DUTY_TAIL;
    		power == 1;
    	}
    	if (power == 1){
			main_duty = 10;
			tail_duty = 10;
			power == 0;
		}
    }

    if (!ulReset) SysCtlReset();

	if (!ulPortValA) {
		 SysCtlReset();
	}
	if (!ulPortValB && STATE == FLYING) {
		degFinal += 15 ;
				}
	if (!ulPortValC && STATE == FLYING) {
		degFinal -= 15;
				}
	if (!ulPortValD) {
		if (STATE == IDLE) {
			PWMOutputState (PWM_BASE, PWM_OUT_4_BIT, true);
			PWMOutputState (PWM_BASE, PWM_OUT_1_BIT, true);
			STATE = ORIENTATING;
		}
		if (STATE == FLYING) {
			STATE = LANDING; // reset 2 = landing
			degFinal = 0;
			altFinal = 0;
		}
		}
	if (!ulPortValE && altFinal < 950 && STATE == FLYING) {
		altFinal += 100;
				}
	if (!ulPortValF && altFinal > 50 && STATE == FLYING) {
		altFinal -= 100;
				}

}
*/

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void initClock (void)
{
  // Set the clock rate. From Section 19.1 in stellaris_peripheral_lib_UG.doc:
  //  "In order to use the ADC, the PLL must be used; the PLL output will be
  //  used to create the clock required by the ADC." ADC rate = 8 MHz / 10.
  //  The processor clock rate = 20 MHz.
  SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);

  // Set up the period for the SysTick timer.  The SysTick timer period is
  // set as a function of the system clock.
  SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);

  // Register the interrupt handler
  SysTickIntRegister(SysTickIntHandler);
  //
  // Enable interrupt and device
  SysTickIntEnable();
  SysTickEnable();
}

//*******************************************************************
// Initialise the SysTick interrupts
//*******************************************************************
void initSysTick (void)
{
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet (SysCtlClockGet() / SYSTICK_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister (SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable ();
    SysTickEnable ();
}

/*
*******************************************************************************
void intButtons (void)
{
	// Register the handler for Port B into the vector table
	GPIOPortIntRegister (GPIO_PORTB_BASE, ButtPressIntHandler);

	//Initialising for buttons
	GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
										   GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
	GPIOPadConfigSet (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
						GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
					GPIO_PIN_5 | GPIO_PIN_6, GPIO_FALLING_EDGE);
	GPIOPinIntEnable (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
					  GPIO_PIN_5 | GPIO_PIN_6);

}
*/


//******************************************************************************
void initDisplay (void)
{
  // intialise the OLED display
  RIT128x96x4Init(1000000);
}

//**********************************************************************
// Initialise UART0 - 8 bits, 1 stop bit, no parity
//**********************************************************************
void initConsole (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), BAUD_RATE,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UART0_BASE);
    UARTEnable(UART0_BASE);
}

//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void UARTSend (char *pucBuffer)
{
    //
    // Loop while there are more characters to send.
    //
    while(*pucBuffer)
    {
        //
        // Write the next character to the UART Tx FIFO.
        //
        UARTCharPut(UART0_BASE, *pucBuffer);
        pucBuffer++;
    }
}


//*****************************************************************************
// Function to display the displays the current height (mili Volts), reference
// height (mili Volts) and the current height as a percentage.
//*****************************************************************************
void displayInfo(int height, int degrees, int main, int tail)
{
	char string[40];

	sprintf(string, "main cycle: %2d%%  ", main);
	RIT128x96x4StringDraw(string, 5, 34, 15);
	sprintf(string, "tail cycle: %2d%%  ", tail);
	RIT128x96x4StringDraw(string, 5, 44, 15);

	sprintf(string, "Hgt. (%%) = %d%%    ", height);
	RIT128x96x4StringDraw(string, 5, 64, 15);


	sprintf (string, "Deg = %4d", degrees);
	RIT128x96x4StringDraw (string, 5, 84, 15);

	//if ((g_ulSampCnt % 100) == 0){
		usprintf(string, "YEah It\n");
		UARTSend (string);
		usprintf(string, "main: %d\n", main_duty);
		UARTSend (string);
		usprintf(string, "tail: %d\n", tail_duty);
		UARTSend (string);
		usprintf(string, "-----------------------\n");
		UARTSend (string);
	//}
}

int main(void)
{

	int hgt_percent = 0;
	int degrees = 0;

	//signed int main_duty = MOTOR_DUTY_MAIN;
	//signed int tail_duty = MOTOR_DUTY_TAIL;


	initClock ();
	initSysTick ();

	//intButtons ();

	initConsole ();
	initDisplay ();

	//initButSet (UP_B | DOWN_B | LEFT_B | RIGHT_B, SYSTICK_RATE_HZ);



	// Enable interrupts to the processor.
	IntMasterEnable();



	while (1)
	{


	   	displayInfo(hgt_percent, degrees, main_duty, tail_duty);
	}
}
