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


#include <TM4C123.h>
#include <math.h>       // logf() for temp math

//================  THERMISTOR PARAMETERS  ================//
#define R_FIXED      10000.0f    // series resistor (top of divider), ohms
#define R0           10000.0f    // thermistor resistance at 25 C (assumed, unmarked part)
#define B_COEF       3977.0f     // Beta (assumed, unmarked part)
#define T0_K         298.15f     // 25 C in kelvin
#define ADC_MAX      4095.0f     // 12-bit ADC 

#define TEMP_ALERT_C 30.0f       // LED threshold [NEW - replaces demo's count >= 2048]
#define N_SAMPLES    16          // readings averaged per update [noise filtering]

//================  SYSTEM CONTROL: clock gating + peripheral-ready  ================//
#define SYS_RCGCGPIO (*((volatile unsigned long *)0x400FE608)) //GPIO clock gate,  p.340
#define SYS_RCGCADC  (*((volatile unsigned long *)0x400FE638)) //ADC  clock gate,  p.352
#define SYS_PRGPIO   (*((volatile unsigned long *)0x400FEA08)) //GPIO ready,       p.406
#define SYS_PRADC    (*((volatile unsigned long *)0x400FEA38)) //ADC  ready,       p.418
#define CLK_GPIOE    0x10                                       //bit 4 = Port E (AIN0/PE3)
#define CLK_GPIOF    0x20                                       //bit 5 = Port F (LED)
#define CLK_ADC0     0x01                                       //bit 0 = ADC module 0

//================  GPIO PORT E: PE3 as the analog input (AIN0)  ================//
#define PORTE_DIR    (*((volatile unsigned long *)0x40024400)) //GPIODIR,    p.663
#define PORTE_AFSEL  (*((volatile unsigned long *)0x40024420)) //GPIOAFSEL,  p.671
#define PORTE_DEN    (*((volatile unsigned long *)0x4002451C)) //GPIODEN,    p.682
#define PORTE_AMSEL  (*((volatile unsigned long *)0x40024528)) //GPIOAMSEL,  p.687
#define PE3          0x08                                       //PE3 = AIN0 (p.801)

//================  GPIO PORT F: red LED (visible threshold indicator)  ================//
#define PORTF_DIR    (*((volatile unsigned long *)0x40025400)) //GPIODIR,    p.663
#define PORTF_DEN    (*((volatile unsigned long *)0x4002551C)) //GPIODEN,    p.682
#define PORTF_DATA   (*((volatile unsigned long *)0x400253FC)) //GPIODATA,   p.662
#define RED_LED      0x02                                       //PF1

//================  ADC0 (base 0x40038000), sample sequencer 3  ================//
#define ADC0_ACTSS   (*((volatile unsigned long *)0x40038000)) //ADCACTSS,    p.821
#define ADC0_RIS     (*((volatile unsigned long *)0x40038004)) //ADCRIS,      p.823
#define ADC0_ISC     (*((volatile unsigned long *)0x4003800C)) //ADCISC,      p.828
#define ADC0_EMUX    (*((volatile unsigned long *)0x40038014)) //ADCEMUX,     p.833
#define ADC0_PSSI    (*((volatile unsigned long *)0x40038028)) //ADCPSSI,     p.845
#define ADC0_SSMUX3  (*((volatile unsigned long *)0x400380A0)) //ADCSSMUX3,   p.875
#define ADC0_SSCTL3  (*((volatile unsigned long *)0x400380A4)) //ADCSSCTL3,   p.876
#define ADC0_SSFIFO3 (*((volatile unsigned long *)0x400380A8)) //ADCSSFIFO3,  p.860

#define SS3          0x08          //sequencer-3 bit, shared by ACTSS / PSSI / RIS / ISC (INR3)
#define SSCTL3_END0  0x02          //ADCSSCTL3: END0 - this is the last (only) sample
#define SSCTL3_IE0   0x04          //ADCSSCTL3: IE0  - set the INR3 flag when done

#define VREF_MV      3300UL        //reference voltage in millivolts

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
    volatile unsigned long s;
    unsigned long sum, i;
	
    SYS_RCGCGPIO |= (CLK_GPIOE | CLK_GPIOF);          // Port E (sensor) + Port F (LED)
    while ((SYS_PRGPIO & (CLK_GPIOE | CLK_GPIOF)) != (CLK_GPIOE | CLK_GPIOF)) { }

    PORTF_DIR |= RED_LED;  PORTF_DEN |= RED_LED;  PORTF_DATA &= ~RED_LED;  // LED output
    s = PORTF_DIR; (void)s;

    ADC0_SS3_Init();

    while (1)
    {
        //---Average N readings---//
        sum = 0;
        for (i = 0; i < N_SAMPLES; i++) { // The purpose of this loop is to smooth out the ADC reading from the sensor to get an average in case their is electrical noise
					sum += ADC0_Read(); 
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
        if (g_temp_c >= TEMP_ALERT_C) PORTF_DATA |=  RED_LED;
        else                          PORTF_DATA &= ~RED_LED;

        delay(200000);                               // ~human-visible update rate
    }
}

//---Configure ADC0 SS3: single sample of AIN0, processor (software) trigger---//
void ADC0_SS3_Init(void)
{
    volatile unsigned long s;

    SYS_RCGCADC |= CLK_ADC0;                           // clock the ADC
    while ((SYS_PRADC & CLK_ADC0) == 0) { }            // wait until ADC0 is ready

    // PE3 -> analog input: alt function ON, digital OFF, analog mode ON
    PORTE_DIR   &= ~PE3;                               // input (direction is don't-care in analog)
    PORTE_AFSEL |=  PE3;                               // alternate function
    PORTE_DEN   &= ~PE3;                               // disable the digital buffer
    PORTE_AMSEL |=  PE3;                               // enable the analog switch
    s = PORTE_AMSEL; (void)s;

    ADC0_ACTSS  &= ~SS3;                               // disable SS3 while configuring
    ADC0_EMUX   &= ~0x0000F000;                        // SS3 trigger = processor (software), EM3=0
    ADC0_SSMUX3  = 0x0;                                // SS3 samples channel AIN0
    ADC0_SSCTL3  = SSCTL3_IE0 | SSCTL3_END0;           // one sample, flag INR3 at the end
    ADC0_ACTSS  |=  SS3;                               // enable SS3
}

//---Software-trigger one conversion and return the 12-bit result---//
unsigned long ADC0_Read(void)
{
    unsigned long result;
    ADC0_PSSI |= SS3;                                  // start a conversion on SS3
    while ((ADC0_RIS & SS3) == 0) { }                  // wait for INR3 (conversion complete)
    result = ADC0_SSFIFO3 & 0xFFF;                     // read the 12-bit sample from the FIFO
    ADC0_ISC = SS3;                                    // clear the INR3 flag
    return result;
}

void delay(volatile unsigned long n) { while (n--) { } }