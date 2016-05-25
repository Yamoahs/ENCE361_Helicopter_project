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
#include "drivers/rit128x96x4.h"
#include "utils/ustdlib.h"
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
#define MOTOR_DUTY_MAIN 80
#define MOTOR_DUTY_TAIL 2

//*******************************************************************
// ISR for the SysTick interrupt (used for button debouncing).
//*******************************************************************
void
SysTickIntHandler (void)
{
	//
	// Poll the buttons
	updateButtons();
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

//******************************************************************
// Initialise the GPIO for the PWM output (Port D and Port F)
//******************************************************************
void
initPin (void)
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
    signed int main_duty = MOTOR_DUTY_MAIN;
    signed int tail_duty = MOTOR_DUTY_TAIL;
    unsigned long period;

    initClock ();
    initPin ();
    initDisplay ();
    initButSet (UP_B | DOWN_B | LEFT_B | RIGHT_B, SYSTICK_RATE_HZ);
    initPWMchan ();
    initSysTick ();

    //
    // Enable interrupts to the processor.
    IntMasterEnable();
    //
    // Compute the PWM period in terms of the PWM clock
    period = SysCtlClockGet () / PWM_DIVIDER / MOTOR_RATE_HZ;
    //
    // Loop forever while controlling the PWM duty cycle.
    while (1)
    {
       // Background task: Check for button pushes and control the
       //  PWM duty cycle +/- 5% within the range 2% to 98%.
       if (checkBut (UP) && main_duty < 98)
       {
    	  main_duty += 10;
    	  if (main_duty >= 98) main_duty = 98;
          PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);

       }
       if (checkBut (DOWN) && main_duty > 2)
       {
    	  main_duty -= 10;
    	  if (main_duty <= 2) main_duty = 2;
          PWMPulseWidthSet (PWM_BASE, PWM_OUT_1, period * main_duty /100);
       }

       if (checkBut (RIGHT) && tail_duty < 98)
       {
    	  tail_duty += 15;
    	  if (tail_duty >= 98) tail_duty = 98;
          PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
       }
       if (checkBut (LEFT) && tail_duty > 2)
       {
    	  tail_duty -= 15;
    	  if (tail_duty <= 2) tail_duty = 2;
          PWMPulseWidthSet (PWM_BASE, PWM_OUT_4, period * tail_duty /100);
       }

       displayInfo(main_duty, tail_duty);

    }
}
