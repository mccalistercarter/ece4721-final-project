// Wireless Cold-Chain and Package Integrity Transity Monitor
// Entrypoint for all subsystem files

#include <stdint.h>
#include <stdbool.h>

#include "system.h"
#include "scheduler.h"

// Other subsystem headers will be called here as well

int main(void) 
{
	// Initialize hardware (Green LED indicates succesful implementation)
	System_Init();
	
	// Initialize all of the individual subsystem modules (will come from headers)
	// example: Temperature_Init(); or Acceleromemter_Init();
	
	// Initialize the scheduler
	Scheduler_Init();
	
	// Main application loop, should stay relatively simple with just the Scheduler running consistently
	while (1) 
	{
		// Scheduler is demonstrated to be running by the blinking red (makes overall LED yellow) LED
		Scheduler_Run();
	}
}