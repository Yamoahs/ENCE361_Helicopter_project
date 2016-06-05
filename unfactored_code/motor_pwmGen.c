//************************************************************************
//
//motor_pwmGen.c - Generates two indiviual PWM outputs, one for the main motor
//					and the other for the tail motor with duty cycle
//					controlled by UP, DOWN, LEFT and RIGHT buttons.
//
// Author:  Samuel Yamoah
// Date Created: 22.5.2016
// Last modified: 22.5.2016
//
//************************************************************************

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "drivers/rit128x96x4.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "stdio.h"
#include "stdlib.h"
#include "buttonSet.h"
#include "button.h"

//************************************************************************
// Generates a single PWM signal at 100 Hz on PD1/PWM1: Pin 53
// Note that PWM Generator 0 drives output 'PWM1' (Ref LMS1968 datasheet)
//************************************************************************

// Constants
#define SYSTICK_RATE_HZ 1000ul
#define MOTOR_RATE_HZ 200
#define PWM_DIV_CODE SYSCTL_PWMDIV_4
#define PWM_DIVIDER 4
#define MOTOR_DUTY_MAIN 10
#define MOTOR_DUTY_TAIL 10

#define BAUD_RATE 9600ul

signed int main_duty = 0;
signed int tail_duty = 0;
int state = 0;
static volatile unsigned long g_tickCnt;
unsigned long period;

//*******************************************************************
// ISR for the SysTick interrupt (used for button debouncing).
//*******************************************************************
void
SysTickIntHandler (void)
{
	//
	// Poll the buttons
	//updateButtons();

	//Polls display on UART0
	g_tickCnt++;
}

//*******************************************************************
// Initialise the clock
//*******************************************************************
void
initClock (void)
{
    //
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_8MHZ);
}

//*******************************************************************
// Initialise the SysTick interrupts
//*******************************************************************
void
initSysTick (void)
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

void ButtPressIntHandler (void)
{
	unsigned long ulSelect; //SELECT
	unsigned long ulUp; //UP
	unsigned long ulDown; //DOWN
	unsigned long ulCCw; //CCW/RIGHT
	unsigned long ulCw; //CW/LEFT
	unsigned long ulReset; //RESET
	//int power = 0;

	// Clear the interrupt (documentation recommends doing this early)
	GPIOPinIntClear (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

	// Read the pins simultaneously to ensure the right states are read
	ulReset = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_1);
	ulCw = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_2);
	ulCCw = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_3);
	ulSelect = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_4);
	ulUp = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_5);
	ulDown = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_6);

	// Compute the PWM period in terms of the PWM clock
	    period = SysCtlClockGet () / PWM_DIVIDER / MOTOR_RATE_HZ;

	if(ulUp == 0 && main_duty < 98){
		main_duty += 10;
		if (main_duty >= 98) main_duty = 98;
		PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);
	}

	if(ulDown == 0 && main_duty > 10){
		main_duty -= 10;
		if (main_duty <= 10) main_duty = 10;
		PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);
		}

	if(ulCw == 0 && tail_duty < 98){
		tail_duty += 15;
		if (tail_duty >= 98) tail_duty = 98;
		PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
		}

	if(ulCCw == 0 && tail_duty > 10){
		tail_duty -= 15;
		if (tail_duty <= 10) tail_duty = 10;
		PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
		}

	if(ulSelect == 0){
		if (state == 0){
			main_duty = MOTOR_DUTY_MAIN;
			tail_duty = MOTOR_DUTY_TAIL;
			state = 1;
		}
	}
		//if (state == 1){
			//main_duty = 0;
			//tail_duty = 0;
			//state = 0;

		//}

	if(ulReset == 0){
		if (!ulReset) SysCtlReset();
		}

}

void intButton (void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	// Register the handler for Port B into the vector table
	GPIOPortIntRegister (GPIO_PORTB_BASE, ButtPressIntHandler);

	//Initialising for buttons
	GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);
	GPIOPadConfigSet (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_FALLING_EDGE);
	GPIOPinIntEnable (GPIO_PORTB_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6);

}

//**********************************************************************
// Initialise UART0 - 8 bits, 1 stop bit, no parity
//**********************************************************************
initConsole (void)
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
void
UARTSend (char *pucBuffer)
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

//**********************************************************************
// Transmit a dummy "status" block of helicopter data, with buffering
//**********************************************************************
void
TxStatus (void)
{
	char string[40];

	//sprintf(string, "butt = %d  ", button);
	//RIT128x96x4StringDraw(string, 5, 14, 15);

	if ((g_tickCnt % 25) == 0){


		sprintf(string, " Main Duty: %d\n----------\n", main_duty);
		UARTSend (string);
		sprintf(string, " Tail Duty: %d\n----------\n", tail_duty);
		UARTSend (string);
   }
}

//******************************************************************
// Initialise the GPIO for the PWM output (Port D and Port F)
//******************************************************************
void
initMotorPin (void)
{
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM (GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinTypePWM (GPIO_PORTF_BASE, GPIO_PIN_2);
}

void
initDisplay (void)
{
  // intialise the OLED display
  RIT128x96x4Init(1000000);
}

//******************************************************************
// Initialise the PWM generator (PWM1 & PWM4)
//******************************************************************
void
initPWMchan (void)
{
	unsigned long period;

    SysCtlPeripheralEnable (SYSCTL_PERIPH_PWM);
    //
    // Compute the PWM period based on the system clock. 
    //
        SysCtlPWMClockSet (PWM_DIV_CODE);

    PWMGenConfigure (PWM_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure (PWM_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    period = SysCtlClockGet () / PWM_DIVIDER / MOTOR_RATE_HZ;
    PWMGenPeriodSet (PWM_BASE, PWM_GEN_0, period);
    PWMGenPeriodSet (PWM_BASE, PWM_GEN_2, period);
    PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * MOTOR_DUTY_MAIN / 100);
    PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * MOTOR_DUTY_TAIL / 100);
    //
    // Enable the PWM output signal.
    //
    PWMOutputState (PWM_BASE, PWM_OUT_1_BIT, true);
    PWMOutputState (PWM_BASE, PWM_OUT_4_BIT, true);
    //
    // Enable the PWM generator.
    //
    PWMGenEnable (PWM_BASE, PWM_GEN_0);
    PWMGenEnable (PWM_BASE, PWM_GEN_2);
}

void
displayInfo(int main, int tail)
{
	char string[40];

	usprintf(string, "Current Freq: %3dHz", MOTOR_RATE_HZ);
	RIT128x96x4StringDraw(string, 5, 24, 15);

	usprintf(string, "main cycle: %2d%%  ", main);
	RIT128x96x4StringDraw(string, 5, 34, 15);

	usprintf(string, "tail cycle: %2d%%  ", tail);
	RIT128x96x4StringDraw(string, 5, 44, 15);

}

int
main (void)
{



    initClock ();
    initMotorPin ();
    initDisplay ();
    intButton();
	initConsole ();
    initButSet (UP_B | DOWN_B | LEFT_B | RIGHT_B, SYSTICK_RATE_HZ);
    initPWMchan ();
    initSysTick ();

    //
    // Enable interrupts to the processor.
    IntMasterEnable();
    //

    //
    // Loop forever while controlling the PWM duty cycle.
    while (1)
    {


       displayInfo(main_duty, tail_duty);
       TxStatus ();

    }
}
