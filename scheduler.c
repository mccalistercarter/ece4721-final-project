// Main scheduler controlling firmware operation and timing
// Left mostly open for now with plans to fully integrate following each subsystem's completion

#include <stdint.h>
#include <stdbool.h>

#include "system.h"
#include "scheduler.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

// Purposefully left configurable so they can change as the system develops
#define ACCLEROMETER_PERIOD_MS 	20
#define BLUETOOTH_PERIOD_MS			10
#define TEMPERATURE_PERIOD_MS		1000
#define LED_PERIOD_MS						500 		// Debug

// For debugging to confirm the scheudler is running
#define RED_LED GPIO_PIN_1

// Variables to measure when a measurement was last taken for each modality
// Static so they last for the entirety of the program
static uint32_t last_accel = 0;
static uint32_t last_bluetooth = 0;
static uint32_t last_temperature = 0;

// Also debug for the LED
static uint32_t last_led = 0;

static uint8_t led_state = 0;

void Scheduler_Init(void)
{
		uint32_t time = System_GetTickCount();
	
		last_accel = time;
		last_bluetooth = time;
		last_temperature = time;
		
		// Everything below is debug and test
		last_led = time;
		led_state = 0;
		
		// Start with debug LED off
		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);
}	

void Scheduler_Run(void)
{
		uint32_t time = System_GetTickCount();
	
		if ((uint32_t)(time - last_accel) >= ACCLEROMETER_PERIOD_MS)
		{
			last_accel = time;
			// Placeholder: would be updated from the specific subsection
			// Acclerometer_Update();
		}
		if ((uint32_t)(time - last_bluetooth) >= BLUETOOTH_PERIOD_MS)
		{
			last_bluetooth = time;
			// Placeholder
			// Bluetooth_Update();
		}
		if ((uint32_t)(time - last_temperature) >= TEMPERATURE_PERIOD_MS)
		{
			last_temperature = time;
			// Placeholder
			// Temperature_Update();
		}
		
		// Debug to toggle LED every 500ms to ensure this file is running
		if ((uint32_t)(time - last_led) >= LED_PERIOD_MS)
		{
			last_led = time;
			led_state = !led_state;
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, led_state ? RED_LED : 0);
		}
}