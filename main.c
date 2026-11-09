/*
* Wireless Cold-Chain and Package Integrity Transity Monitor
*
* Main system file that will initialize hardware and run the primary system firmware.
*/

#include <stdint.h>
#include <stdbool.h>

#include "system.h"
// Other subsystem headers will be called here as well

int main(void) 
{
	// Initialize hardware (Green LED indicates succesful implementation)
	System_Init();
	
	// Initialize all of the individual subsystem modules (will come from headers)
	// example: Temperature_Init(); or Acceleromemter_Init();
	
	// Initialize the scheduler
	
	// Main application loop, should stay relatively simple with just the Scheduler running consistently
	// Infinite while loop for preliminary testing
	while (1) {}
}