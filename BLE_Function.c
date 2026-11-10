// BLE_Function.c - Based on bluetooth_uart_example project
//
// Talks to an Adafruit Bluefruit LE UART Friend on UART1.  The module's mode
// switch must be on UART.  At startup the module is named BT_NAME; then
// anything sent from the Bluefruit Connect app is echoed back to the phone
// and shown on the debug console.
//
// UART0 is a debug console on the USB virtual serial port (115,200 8-N-1);
// use UARTprintf() to print to it.
//
// (Based on TI's uart_echo example for the EK-TM4C123GXL)
//
// Keil uVision version: builds with Arm Compiler 6 (or 5).  The vector table
// and stack are in startup_keil.s.

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
// Bluetooth (UART1) - copy this section to use it in your own project, and
// put BT_IntHandler in the UART1 entry of the vector table in startup_keil.s.
//
// Receiving: data can arrive at any time, so the UART1 interrupt moves each
// received byte into rx_buffer.  main() takes bytes out whenever it is ready
// with BT_RX_Available() / BT_RX_Read().
//
// Sending: BT_TX_Byte() hands each byte straight to the UART, waiting if the
// UART is full.  At 9600 baud each byte takes about 1 ms to send.
//*****************************************************************************

// Must be a power of 2 so "& (SIZE - 1)" wraps the index around.
#define BT_RX_BUFFER_SIZE 128

// Received bytes.  The interrupt writes at rx_head; main reads at rx_tail.
// Each index has only one writer, so no extra protection is needed.
volatile uint8_t  rx_buffer[BT_RX_BUFFER_SIZE];
volatile uint32_t rx_head = 0;
volatile uint32_t rx_tail = 0;

// Set up UART1 (PB0/PB1, RTS/CTS on PC4/PC5, 9600 8-N-1) and its interrupt.
void BT_Init(void)
{
    // Enable UART1, GPIO B (Rx/Tx), and GPIO C (RTS/CTS).
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Set GPIO B0 and B1 as UART1 Rx and Tx.
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Set GPIO C4 and C5 as UART1 RTS and CTS.  PC4/PC5 can also be U1RX/U1TX,
    // so the RTS/CTS mappings must be selected explicitly.  Until PC4 drives
    // U1RTS, the module's CTS input floats high and it will not transmit.
    GPIOPinConfigure(GPIO_PC4_U1RTS);
    GPIOPinConfigure(GPIO_PC5_U1CTS);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Enable RTS/CTS hardware flow control (no ROM version of this one).
    UARTFlowControlSet(UART1_BASE, UART_FLOWCONTROL_TX | UART_FLOWCONTROL_RX);

    // Configure UART1 for 9,600, 8-N-1 operation (the module's default).
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    // Interrupt when data arrives, and also when a few bytes have arrived and
    // then the line goes quiet (so short messages are not left waiting).
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    ROM_IntEnable(INT_UART1);
}



// Send one byte.  Waits if the UART can't take another byte yet.
void BT_TX_Byte(uint8_t c)
{
    ROM_UARTCharPut(UART1_BASE, c);
}

// Send a string.
void BT_TX_String(const char *s)
{
    while(*s)
    {
        BT_TX_Byte(*s++);
    }
}

// Number of received bytes waiting to be read.
uint32_t BT_RX_Available(void)
{
    return (rx_head - rx_tail) & (BT_RX_BUFFER_SIZE - 1);
}

// Take one received byte.  Only call when BT_RX_Available() > 0.
uint8_t BT_RX_Read(void)
{
    uint8_t c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) & (BT_RX_BUFFER_SIZE - 1);

    // There is room again, so let the interrupt resume if it stopped on a
    // full buffer.
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    return c;
}

// UART1 interrupt: move every byte the UART has received into rx_buffer.
void BT_IntHandler(void)
{
    uint32_t status = ROM_UARTIntStatus(UART1_BASE, true);
    ROM_UARTIntClear(UART1_BASE, status);

    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        uint32_t next = (rx_head + 1) & (BT_RX_BUFFER_SIZE - 1);

        // rx_buffer is full: stop reading.  The UART holds on to new bytes,
        // flow control pauses the module, and BT_RX_Read() turns this back on.
        if(next == rx_tail)
        {
            ROM_UARTIntDisable(UART1_BASE, UART_INT_RX | UART_INT_RT);
            return;
        }

        rx_buffer[rx_head] = ROM_UARTCharGetNonBlocking(UART1_BASE);
        rx_head = next;
    }
}

void UART1_Handler(void) {
	BT_IntHandler();
}

// Wait about the given number of milliseconds.
// Each SysCtlDelay loop is 3 clocks.
void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        ROM_SysCtlDelay(ROM_SysCtlClockGet() / 3000);
    }
}

// Throw away anything received so far.
void BT_Flush(void)
{
    while(BT_RX_Available())
    {
        BT_RX_Read();
    }
}

// Set up the Bluetooth module with AT commands, sent "open loop": we don't
// read the replies, we just wait long enough for each command to finish.
// Call once at startup (interrupts must be on), before a phone connects.
void BT_Config(const char *name)
{
    // "+++" switches the module from data mode to command mode.
    BT_TX_String("+++\r\n");
    delay_ms(200);

    // Make the module's red MODE LED blink when data goes to or from the phone.
    BT_TX_String("AT+HWMODELED=BLEUART\r\n");
    delay_ms(200);

    // Set the Bluetooth name (the module saves it).
    BT_TX_String("AT+GAPDEVNAME=");
    BT_TX_String(name);
    BT_TX_String("\r\n");
    delay_ms(200);

    // Reset the module so the new name takes effect.  It restarts in the mode
    // its switch sets (data mode), so this also ends command mode.
    BT_TX_String("ATZ\r\n");
    delay_ms(1500);

    // Clear RX buffer to discard module's replies ("OK" ...) to config commands
    BT_Flush();
}

//*****************************************************************************
// Buttons: SW1 (PF4) and SW2 (PF0).  Each reads 0 while pressed.
//*****************************************************************************

// Set up SW1 and SW2 as inputs with pull-up resistors.
void Buttons_Init(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // PF0 is locked at reset (it can also be the NMI input), so unlock it
    // before it can be set up as a normal input.
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    // The buttons connect the pins to ground when pressed; the pull-ups hold
    // them high otherwise.
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}



//*****************************************************************************
// Program
//*****************************************************************************

// The name the module shows up as in the Bluefruit Connect app.
#define BT_NAME "Package_Monitor"

// Set up the clock, the UART0 debug console, and the Bluetooth UART.
void init(void)
{
    // Run directly from the 16 MHz crystal.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    // Debug console: UART0 on GPIO A0/A1, 115,200 8-N-1, used by UARTprintf().
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, ROM_SysCtlClockGet());

    // Bluetooth module on UART1.
    BT_Init();
	
		//Switches for testing purposes only. Will likely be removed for finished project
    // SW1 and SW2 (used by EXAMPLE 2). 
    Buttons_Init();

    // Enable processor interrupts.
    ROM_IntMasterEnable();
}

int main(void)
{
    bool sw1, sw2;              // EXAMPLE 2: buttons pressed now?
    bool sw1_was = false;       // EXAMPLE 2: ...and last time we checked?
    bool sw2_was = false;

    init();

    UARTprintf("\nbluetooth_uart_example started :)\n");

    // Name the module and set it up (takes about 2 seconds).
    BT_Config(BT_NAME);
    UARTprintf("Connect to \"%s\" in the Bluefruit Connect app.\n", BT_NAME);

    while(1)
    {
			/* Debug testing:
        // EXAMPLE 0: show every byte received over Bluetooth on the debug
        // console, in hex (and as a character if it is printable).  Useful for
        // seeing exactly what the app sends from any screen.  Comment out
        // EXAMPLE 3 while using this one: both read the same received bytes.
        while(BT_RX_Available())
        {
            uint8_t c = BT_RX_Read();

            if(c >= ' ' && c <= '~')
            {
                UARTprintf("0x%02x  '%c'\n", c, c);
            }
            else
            {
                UARTprintf("0x%02x\n", c);
            }
        }
        */

        /*
        // EXAMPLE 1: echo anything received over Bluetooth back to the phone,
        // and show it on the debug console.  (Bluefruit Connect: UART)
        while(BT_RX_Available())
        {
            uint8_t c = BT_RX_Read();
            BT_TX_Byte(c);
            UARTprintf("%c", c);
        }
        */

        // EXAMPLE 2: send a message to the phone when SW1 or SW2 is pressed
        // or released.
        // (Bluefruit Connect: UART)  Runs alongside EXAMPLE 3.
        sw1 = (ROM_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0);
        sw2 = (ROM_GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0) == 0);

        // Send a message only when a button changes: when it goes down
        // (pressed) or comes back up (released).
        if(sw1 && !sw1_was) //Test demo for temperature alert
        {
            BT_TX_String("Temp too High\n");
            UARTprintf("Sent: Temp Alert\n");
        }
        if(!sw1 && sw1_was)
        {
            BT_TX_String("Temp Stable\n");
            UARTprintf("Sent: Temp Stable\n");
        }
        if(sw2 && !sw2_was) //Test demo for impact detection
        {
            BT_TX_String("Impact Detection\n");
            UARTprintf("Sent: Impact Detection\n");
        }
        if(!sw2 && sw2_was)
        {
            //BT_TX_String("SW2 released\n");
            //UARTprintf("Sent: SW2 released\n");
        }
        sw1_was = sw1;
        sw2_was = sw2;

        // Check the buttons every 10 ms.  A button's contacts bounce for a few
        // ms when pressed or released; checking less often than that keeps
        // one press from looking like several.
        delay_ms(10);
    }
}