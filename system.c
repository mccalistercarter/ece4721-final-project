// Main system file responsible for all hardware initialization
// All system governing functions should be in here

#include "system.h"
#include <stdint.h>
#include <stdbool.h>

// TivaWare libraries, more will be included as necessary
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"

// 16 MHz System Clock
#define SYSTEM_CLOCK_HZ 16000000

// Define a 1ms SysTick clock for now to be used (may be changed)
#define SYSTICK_FREQUENCY 1000

// Variable to be used as millisecond counter
static volatile uint32_t system_tick = 0;

// SysTick handler simply increments the variable
void SysTick_Handler(void)
{
	system_tick++;
}

void System_Init(void)
{
		// Configure the system clock to run at 16 MHz
		SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	
		// Enable GPIO Port F for use of onboard LEDs
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
		
		// Configure PF1, PF2, PF3 as outputs
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
		
		// Configure SysTick to generate 1ms interrupts using the defines above
		SysTickPeriodSet(SYSTEM_CLOCK_HZ / SYSTICK_FREQUENCY);
	
		SysTickIntEnable();
		SysTickEnable();
			
		// Enable global interrupts for SysTick as well as external interrupts for future use
		IntMasterEnable();
			
		// Use the Green LED to indicate succesful system intialization for debugging and verification
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);
}

// Small function to return millisecond count as needed, will be used in later scheduling tasks
uint32_t System_GetTickCount(void)
{
		return system_tick;
}