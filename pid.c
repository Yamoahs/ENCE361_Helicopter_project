//************************************************************************
//
// pwmGen3.c - Example code which generates a single PWM output
//            with duty cycle controlled by UP and DOWN buttons.
//
// Author: Samuel Yamoah	UCECE
// Date Created:
// Last modified:	18.5.2015
//
//************************************************************************

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "drivers/rit128x96x4.h"
#include "utils/ustdlib.h"
#include "buttonSet.h"
#include "button.h"

//************************************************************************
// Generates a single PWM signal at 100 Hz on PD1/PWM1: Pin 53
// Note that PWM Generator 0 drives output 'PWM1' (Ref LMS1968 datasheet)
//************************************************************************

// Constants
#define SYSTICK_RATE_HZ 10000ul
#define TICKS_PER_BUT_CHK 10ul
#define PWM4_RATE_HZ 150
#define PWM1_RATE_HZ 150
#define PWM_DIV_CODE SYSCTL_PWMDIV_4
#define PWM_DIVIDER 4
#define PWM4_DEF_DUTY 50
#define PWM1_DEF_DUTY 50
#define CLOCK_SPEED 20000000

//*************************************************************************
//Globals:
volatile unsigned long g_tickCnt; // Counter for the interrupts
volatile unsigned long g_period;


//*******************************************************************
// ISR for the SysTick interrupt (used for button debouncing).
//*******************************************************************
void
SysTickIntHandler (void)
{
	//
	// Poll the buttons
	g_tickCnt++;
	if (g_tickCnt % TICKS_PER_BUT_CHK == 0)
		updateButtons();          // poll buttons
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

//Input Code Added here
void
PinChangeIntHandler (void)
{
    static unsigned long ulLastCnt;    // Retains previous count of ticks

    // Clear the interrupt (documentation recommends doing this early)
    GPIOPinIntClear (GPIO_PORTD_BASE, GPIO_PIN_0);
     // Calculate the period in ticks and prepare for next interrupt
    g_period = g_tickCnt - ulLastCnt;
    ulLastCnt = g_tickCnt;
}

//*******************************************************************
//Calculating the output frequency from dividing the system rate (HZ)
// and dividing it by the
unsigned long
convert_to_freq(void)
{
	unsigned long input_freq = SYSTICK_RATE_HZ / g_period;
	return (input_freq);
}

//******************************************************************
// Initialise the GPIO for the PWM output (Port D)
//******************************************************************
void
initPin (void)
{
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF | SYSCTL_PERIPH_GPIOD);
    GPIOPinTypePWM (GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypePWM (GPIO_PORTD_BASE, GPIO_PIN_1);

    //Input
    // Register the handler for Port F and D into the vector table
    GPIOPortIntRegister (GPIO_PORTD_BASE, PinChangeIntHandler);
    //
    // Enable and configure the port and pin used:  input on PD0
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD);
    GPIOPadConfigSet (GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA,
    		GPIO_PIN_TYPE_STD_WPU);
    //
    // Set up the pin change interrupt (both edges)
    GPIOIntTypeSet (GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);
    //
    // Enable the pin change interrupt
    GPIOPinIntEnable (GPIO_PORTD_BASE, GPIO_PIN_0);
    IntEnable (INT_GPIOD);	// Note: INT_GPIOD defined in inc/hw_ints.h
}


//******************************************************************
// Initialise the PWM generator (PWM1)
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
    PWMGenConfigure (PWM_BASE, PWM_GEN_2 | PWM_GEN_0,
        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    /*period = SysCtlClockGet () / PWM_DIVIDER / PWM4_RATE_HZ;
    PWMGenPeriodSet (PWM_BASE, PWM_GEN_2, period);
    PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * PWM4_DEF_DUTY / 100);*/
    //
    // Enable the PWM output signal.
    //
    PWMOutputState (PWM_BASE, PWM_OUT_4_BIT, true);
    PWMOutputState (PWM_BASE, PWM_OUT_1_BIT, true);
    //
    // Enable the PWM generator.
    //
    PWMGenEnable (PWM_BASE, PWM_GEN_2 | PWM_GEN_0);
}

void
initDisplay (void)
{
  // intialise the OLED display
  RIT128x96x4Init(1000000);
}

void
displayInfo(int count, int duty, unsigned long frequency, unsigned long input)
{
	char string[40];

	usprintf(string, "Current Freq: %3dHz", frequency);
	RIT128x96x4StringDraw(string, 5, 24, 15);
	usprintf(string, "Duty cycle: %2d%%", duty);
	RIT128x96x4StringDraw(string, 5, 34, 15);
	usprintf(string, "Time: %7ds", (count/SYSTICK_RATE_HZ));
	RIT128x96x4StringDraw(string, 5, 44, 15);
	usprintf(string, "input: %7ds", (input));
	RIT128x96x4StringDraw(string, 5, 54, 15);

}

int
main (void)
{
    unsigned int duty = PWM4_DEF_DUTY;
    unsigned long period;
    unsigned long input;

    initClock ();
    initPin ();
    initButSet (UP_B | DOWN_B, SYSTICK_RATE_HZ/TICKS_PER_BUT_CHK);
    initPWMchan ();
    initSysTick ();
    initDisplay ();
    PinChangeIntHandler();

    //
    // Enable interrupts to the processor.
    IntMasterEnable();
    //
    // Compute the PWM period in terms of the PWM clock
    period = SysCtlClockGet () / PWM_DIVIDER / PWM4_RATE_HZ;

    //
    // Loop forever while controlling the PWM duty cycle.
    while (1)
    {
    	input = convert_to_freq();
    	if (input > 100 && input < 300)
    	{
    	   period = SysCtlClockGet () / PWM_DIVIDER / input;
    	}
    	else
    	{
    		period = SysCtlClockGet () / PWM_DIVIDER / PWM4_RATE_HZ;
    	}
    	PWMGenPeriodSet (PWM_BASE, PWM_GEN_2, period);
		PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * duty /100);

	   // Background task: Check for button pushes and control the
	   //  PWM duty cycle +/- 5% within the range 5% to 95%.
	   if (checkBut (UP) && duty < 95)
	   {
		  duty += 5;
	   }
	   if (checkBut (DOWN) && duty > 5)
	   {
		  duty -= 5;
	   }
    	displayInfo((int) g_tickCnt, duty,  (SysCtlClockGet () / PWM_DIVIDER) / period, input);
    }
}
