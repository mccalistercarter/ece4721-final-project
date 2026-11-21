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
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

//================  THERMISTOR PARAMETERS  ================//
#define R_FIXED      10000.0f    // series resistor (top of divider), ohms
#define R0           10000.0f    // thermistor resistance at 25 C (assumed, unmarked part)
#define B_COEF       3977.0f     // Beta (assumed, unmarked part)
#define T0_K         298.15f     // 25 C in kelvin
#define ADC_MAX      4095.0f     // 12-bit ADC 

#define TEMP_ALERT_C 29.0f       // LED threshold [NEW - replaces demo's count >= 2048]
#define ADC_SEQ		 3			 // sampler sequencer 3
#define N_SAMPLES    16          // readings averaged per update [noise filtering]

#define RED_LED      GPIO_PIN_1		//PF1
#define VREF_MV      3300UL        	//reference voltage in millivolts

//---Inspect in the debugger---//
volatile unsigned long g_adc;      // raw 12-bit count, 0..4095
volatile unsigned long g_mv;       // converted millivolts, 0..3300
volatile float g_rt;               // thermistor resistance, ohms
volatile float g_temp_c;           // temperature, C
volatile float g_temp_f;           // temperature, F

void ADC0_SS3_Init(void);
unsigned long ADC0_Read(void);
void delay(volatile unsigned long n);

int main(void)
{
    uint32_t sum, i;

	//Port F output - LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED);
	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);

    ADC0_SS3_Init();

    while (1)
    {
        //---Average N readings---//
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
        }

        //---Temperature threshold instead of raw count---//
        if (g_temp_c >= TEMP_ALERT_C) GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, RED_LED);
        else                          GPIOPinWrite(GPIO_PORTF_BASE, RED_LED, 0);

        delay(200000);                               // ~human-visible update rate
    }
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

void delay(volatile uint32_t n) { while (n--) { } } // [adc_sensor_demo.c]
