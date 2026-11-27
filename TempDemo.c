/*
 * Filename: adc_sensor_demo.c CHANGED FOR THERMISTOR SENSOR (everything applies but just with a new sensor)
 *
 * Description: Lecture 10 (ADC & Sampling) live demo. Reads one analog sensor on
 *  AIN0 (PE3) with ADC0, sample sequencer 3, SOFTWARE-triggered - the lecture's
 *  "read the pot, report counts and voltage" example. The 12-bit result and its
 *  millivolt value are left in globals for the debugger, and the on-board RED LED
 *  (PF1) turns on above mid-scale so the reading is visible without a display
 *  (cover the light sensor / turn the pot and the LED flips).
 *
 *  SENSOR (from the kit): any ANALOG-OUT module works - photoresistor/light
 *  module, the analog temperature (thermistor) module, the sound module's AO pin,
 *  or a rotary potentiometer. Wire:  sensor AO -> PE3,  sensor GND -> board GND.
 *
 *  !! POWER THE SENSOR FROM 3.3 V, NOT 5 V !!  The ADC input maximum is VDDA =
 *  3.3 V and the analog pins are NOT 5 V tolerant. Powering the module from 3.3 V
 *  keeps its analog output inside 0-3.3 V so it can never over-drive the pin.
 *
 *  CLOCK: runs at the ~16 MHz default (no PLL). The ADC meets its >=16 MHz clock
 *  requirement at this setting, so no extra clock setup is needed.
 *
 *  Count -> voltage:  V = count/4095 * 3.3 V   (LSB ~= 0.81 mV).  This demo is
 *  software-triggered; Lab 4 extends it to TIMER-triggered sampling + averaging.
 *
 *  Datasheet page numbers for every register address are given in the #defines.
 */
 
/*
 * For this specific Thermistor demo, we need to take values of the thermistor itself 
 * and do some ADC math to get the thermistors resistance, then followed by temperature math
 * to get the temperature from the resistance in both C and F
 */


#include <stdint.h>
#include <stdbool.h>
#include <math.h>       // logf() for temp math
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

//================  TIMING  ================//
#define F_CLK        16000000UL                   // default internal osc (no PLL configured)
#define PERIOD_MS    250UL                        // interrupt (and flag) every 250 ms
#define RELOAD       ((F_CLK/1000UL)*PERIOD_MS)   // ticks in PERIOD_MS = 4,000,000

//================  THERMISTOR PARAMETERS  ================//
#define R_FIXED      10000.0f    // series resistor (top of divider), ohms
#define R0           10000.0f    // thermistor resistance at 25 C (assumed, unmarked part)
#define B_COEF       3977.0f     // Beta (assumed, unmarked part)
#define T0_K         298.15f     // 25 C in kelvin
#define ADC_MAX      4095.0f     // 12-bit ADC 

#define TEMP_ALERT_HIGH 	29.0f       // LED higher threshold [NEW - replaces demo's count >= 2048]
#define TEMP_ALERT_LOW 		20.0f		// LED lower threshold
#define N_SAMPLES    		16          // readings averaged per update [noise filtering]

//================  TEMPERATURE STATES  ================//
#define STATE_OK     0           // within limits
#define STATE_COLD   1           // below TEMP_ALERT_LOW
#define STATE_HOT    2           // above TEMP_ALERT_HIGH

//================  PINS  ================//
#define RED_LED   	GPIO_PIN_1		//PF1
#define BLUE_LED   	GPIO_PIN_2		//PF2
#define GREEN_LED 	GPIO_PIN_3		//PF3
#define LEDS      	(RED_LED | BLUE_LED | GREEN_LED)

#define ADC_SEQ		 3			 // sampler sequencer 3
#define VREF_MV      3300UL      //reference voltage in millivolts

//---Inspect in the debugger---//
volatile uint32_t g_adc;      		// raw 12-bit count, 0..4095
volatile uint32_t g_mv;       		// converted millivolts, 0..3300
volatile float g_rt;               	// thermistor resistance, ohms
volatile float g_temp_c;           	// temperature, C
volatile float g_temp_f;           	// temperature, F

volatile uint32_t g_ticks = 0;
volatile uint32_t g_newsample = 0;
volatile uint32_t g_alertflag = 0;
volatile uint32_t g_alertcount = 0;
volatile uint32_t g_temp_state = STATE_OK;

uint32_t g_prev_state = STATE_OK;
volatile uint32_t g_alert_state = STATE_OK;

void LEDS_Init(void);
void ADC0_SS3_Init(void);
uint32_t ADC0_Read(void);
void LEDS_Update(void);
uint32_t TEMP_Update(void);
void TEMP_State(void); 
void Timer0A_Periodic_Init(void);

//=== THE TIMER INTERRUPT ===//
void TIMER0A_Handler(void)			// referenced from timer_periodic_demo.c
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);		// clear the timeout flag
	g_newsample = 1;
	g_ticks++;											// count interrupts
}

int main(void)
{
	LEDS_Init();
    ADC0_SS3_Init();
	Timer0A_Periodic_Init();
	IntMasterEnable();				// make sure interrupts are globally enabled

    while (1)
	{
	    if (g_newsample)
	    {
			g_newsample = 0;
	
			if (TEMP_Update())
	      	{
				TEMP_State();
	       		LEDS_Update();
	
				if (g_temp_state != g_prev_state)
				{
					g_prev_state = g_temp_state;
	
					if (g_temp_state != STATE_OK)
					{
						g_alert_state = g_temp_state;
						g_alertflag = 1;
						g_alertcount++;
					}
				}
			}
		}
	}
}

void LEDS_Init(void)
{
	//Port F output - LED [this was moved out of main for organization]
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LEDS);
	GPIOPinWrite(GPIO_PORTF_BASE, LEDS, 0);
}

//---Configure ADC0 SS3: single sample of AIN0, processor (software) trigger---//
void ADC0_SS3_Init(void) // [adc_sensor_demo.c] but with tivware api
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}

    // PE3 -> analog input: alt function ON, digital OFF, analog mode ON
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

	ADCSequenceDisable(ADC0_BASE, ADC_SEQ);
	ADCSequenceConfigure(ADC0_BASE, ADC_SEQ, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, ADC_SEQ);
	ADCIntClear(ADC0_BASE, ADC_SEQ);
}

//---Software-trigger one conversion and return the 12-bit result---//
uint32_t ADC0_Read(void) // [adc_sensor_demo.c] but with tivware api
{
    uint32_t result;
	
    ADCProcessorTrigger(ADC0_BASE, ADC_SEQ);                  	// start a conversion
    while (!ADCIntStatus(ADC0_BASE, ADC_SEQ, false)) { }    	// wait for INR3 (conversion complete)
    ADCIntClear(ADC0_BASE, ADC_SEQ);                     		// clear the INR3 flag// read the 12-bit sample from the FIFO
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQ, &result);         	// read the 12-bit sample from the FIFO
    return result & 0XFFF; // from result = ADC0_SSFIFO3 & 0xFFF;
}

//---TIMER0A: 32-bit periodic, interrupt every PERIOD_MS---//
void Timer0A_Periodic_Init(void)		// [timer_periodic_demo.c] but with tivware api
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);             	// enable the TIMER0 clock
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) { }  	// wait until the timer is ready

    TimerDisable(TIMER0_BASE, TIMER_A);                        	// disable Timer A while we configure it
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);          	// 32-bit timer mode, periodic, count-down, auto-reload
    TimerLoadSet(TIMER0_BASE, TIMER_A, RELOAD - 1);            	// reload value: period = (TAILR+1)/f_clk -> N+1 rule
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);            	// clear any stale time-out flag
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);          	// enable the time-out interrupt in the timer

    IntEnable(INT_TIMER0A);                                    	// enable TIMER0A in the NVIC

	TimerEnable(TIMER0_BASE, TIMER_A);							// start timer A
}

void TEMP_State(void)
{
	if (g_temp_c >= TEMP_ALERT_HIGH)
    {
        g_temp_state = STATE_HOT;
    }
    else if (g_temp_c <= TEMP_ALERT_LOW)
    {
        g_temp_state = STATE_COLD;
    }
    else
    {
        g_temp_state = STATE_OK;
	}
}

uint32_t TEMP_Update(void)	// moved from main()
{
	uint32_t sum, i;
	sum = 0;
	for (i = 0; i < N_SAMPLES; i++) { 
			sum += ADC0_Read();		// The purpose of this loop is to smooth out the ADC reading from the sensor to get an average in case their is electrical noise
		}
				
    g_adc = sum / N_SAMPLES;
    g_mv  = (g_adc * VREF_MV) / 4095UL;          // [adc_sensor_demo.c]

    //---Count -> resistance -> temperature ---//
    if (g_adc > 0 && g_adc < 4095)               // 0 = shorted, 4095 = open/unplugged
    {
        g_rt     = R_FIXED * (float)g_adc / (ADC_MAX - (float)g_adc);             // Calculates Thermistor resistance based on voltage divider equation (rearranged for resistance, but with ADC values)
        g_temp_c = 1.0f / (logf(g_rt / R0) / B_COEF + 1.0f / T0_K) - 273.15f;
        g_temp_f = g_temp_c * 1.8f + 32.0f;
		return 1;								// a good reading
    }
	return 0;									// a bad reading
}

void LEDS_Update(void)	// moved from main()
{
	//---Temperature thresholds---//
    if (g_temp_c >= TEMP_ALERT_HIGH)		// hot
	{
		GPIOPinWrite(GPIO_PORTF_BASE, LEDS, RED_LED);
	}	
    else if (g_temp_c <= TEMP_ALERT_LOW)	// cold
	{
		GPIOPinWrite(GPIO_PORTF_BASE, LEDS, BLUE_LED);
	}
	else
	{
		GPIOPinWrite(GPIO_PORTF_BASE, LEDS, GREEN_LED);
	}
}
