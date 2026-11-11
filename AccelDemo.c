// MPU6500 Accelerometer Orientation Demo
// TM4C123G LaunchPad + TivaWare API
//
// X axis > +/-0.9g -> RED
// Y axis > +/-0.9g -> BLUE
// Z axis > +/-0.9g -> GREEN
//
// MPU6500 communicates with TM4C123G through I2C0.
//

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"


// ============================================================
// MPU6500 DEFINITIONS
// ============================================================
// These values are NOT from TivaWare.
// These are MPU6500 sensor registers/configuration values.

#define IMU_ADDR        0x68

#define REG_PWR_MGMT_1  0x6B    // MPU6500 power management register
#define REG_ACCEL_CFG   0x1C    // MPU6500 accelerometer configuration register
#define REG_ACCEL_XOUT  0x3B    // First accelerometer output register
#define REG_WHO_AM_I    0x75    // Device identification register

// Accelerometer full-scale setting:
// 0x00 = +/-2g   -> 16384 counts/g
// 0x08 = +/-4g   -> 8192 counts/g
// 0x10 = +/-8g   -> 4096 counts/g
// 0x18 = +/-16g  -> 2048 counts/g

#define ACCEL_FS        0x00
#define LSB_PER_G       16384

// 90% of 1g.
// At +/-2g:
// 16384 * 0.9 = approximately 14745 counts.
#define THRESH          ((LSB_PER_G * 9) / 10)

// GPIOPinTypeGPIOOutput:
// TivaWare API p.275
//
// GPIOPinWrite:
// TivaWare API pp.286-287

#define RED     GPIO_PIN_1
#define BLUE    GPIO_PIN_2
#define GREEN   GPIO_PIN_3
#define ALL     (RED | BLUE | GREEN)


// Variables are volatile so they are convenient to observe
// using the Keil Watch window while debugging.

volatile int16_t ax;
volatile int16_t ay;
volatile int16_t az;

volatile uint8_t whoami;


// ============================================================
// WAIT FOR I2C TRANSACTION
// ============================================================
//
// I2CMasterBusy:
// TivaWare API p.331
//
// Waits until the I2C controller reports that the
// current transaction has completed.

void I2C_Wait(void)
{
    while (I2CMasterBusy(I2C0_BASE))
    {
    }
}


// ============================================================
// WRITE ONE MPU6500 REGISTER
// ============================================================
//
// I2CMasterSlaveAddrSet:
// TivaWare I2C Master API
//
// I2CMasterDataPut:
// TivaWare I2C Master API
//
// I2CMasterControl:
// TivaWare API p.331
//
// BURST_SEND_START / BURST_SEND_FINISH are TivaWare
// command values used to generate the required I2C
// START/STOP sequence.

void IMU_Write(uint8_t reg, uint8_t val)
{
    // false means TM4C will WRITE to the sensor.
    I2CMasterSlaveAddrSet(I2C0_BASE, IMU_ADDR, false);

    // First byte tells MPU6500 which register we want.
    I2CMasterDataPut(I2C0_BASE, reg);

    I2CMasterControl(I2C0_BASE,
                     I2C_MASTER_CMD_BURST_SEND_START);

    I2C_Wait();

    // Second byte is the value written into that register.
    I2CMasterDataPut(I2C0_BASE, val);

    I2CMasterControl(I2C0_BASE,
                     I2C_MASTER_CMD_BURST_SEND_FINISH);

    I2C_Wait();
}


// ============================================================
// READ MPU6500 REGISTER(S)
// ============================================================
//
// I2CMasterSlaveAddrSet:
// TivaWare I2C Master API
//
// I2CMasterDataPut / DataGet:
// TivaWare I2C Master API
//
// I2CMasterControl and command values:
// TivaWare API p.331
//
// This function lets us read either one register or
// several consecutive MPU6500 registers.

void IMU_Read(uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint8_t i;

    // --------------------------------------------------------
    // Step 1: tell MPU6500 which register we want.
    // --------------------------------------------------------

    I2CMasterSlaveAddrSet(I2C0_BASE, IMU_ADDR, false);

    I2CMasterDataPut(I2C0_BASE, reg);

    I2CMasterControl(I2C0_BASE,
                     I2C_MASTER_CMD_SINGLE_SEND);

    I2C_Wait();


    // --------------------------------------------------------
    // Step 2: switch I2C controller into READ mode.
    // true = read from slave.
    // --------------------------------------------------------

    I2CMasterSlaveAddrSet(I2C0_BASE, IMU_ADDR, true);


    // --------------------------------------------------------
    // Reading only one byte (example: WHO_AM_I)
    // --------------------------------------------------------

    if (n == 1)
    {
        I2CMasterControl(I2C0_BASE,
                         I2C_MASTER_CMD_SINGLE_RECEIVE);

        I2C_Wait();

        buf[0] = I2CMasterDataGet(I2C0_BASE);

        return;
    }


    // --------------------------------------------------------
    // Reading several consecutive bytes.
    // Used for X/Y/Z acceleration.
    // --------------------------------------------------------

    I2CMasterControl(I2C0_BASE,
                     I2C_MASTER_CMD_BURST_RECEIVE_START);

    I2C_Wait();

    buf[0] = I2CMasterDataGet(I2C0_BASE);


    for (i = 1; i < n - 1; i++)
    {
        I2CMasterControl(I2C0_BASE,
                         I2C_MASTER_CMD_BURST_RECEIVE_CONT);

        I2C_Wait();

        buf[i] = I2CMasterDataGet(I2C0_BASE);
    }


    I2CMasterControl(I2C0_BASE,
                     I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    I2C_Wait();

    buf[n - 1] = I2CMasterDataGet(I2C0_BASE);
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
    uint8_t d[6];
    uint8_t led;


    // ========================================================
    // 1. SYSTEM CLOCK
    // ========================================================
    //
    // SysCtlClockSet:
    // TivaWare API p.493
    //
    // SYSCTL_XTAL_16MHZ = LaunchPad's 16 MHz crystal.
    // SYSCTL_USE_PLL     = use internal PLL.
    // SYSCTL_SYSDIV_4    = configure approximately 50 MHz.

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);


    // ========================================================
    // 2. CONFIGURE RGB LED
    // ========================================================
    //
    // SysCtlPeripheralEnable:
    // TivaWare API pp.509-510
    //
    // SysCtlPeripheralReady:
    // TivaWare API p.512
    //
    // GPIOPinTypeGPIOOutput:
    // TivaWare API p.275

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
			
    }

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, ALL);


    // ========================================================
    // 3. ENABLE I2C0 AND GPIO PORT B
    // ========================================================
    //
    // SysCtlPeripheralEnable:
    // TivaWare API pp.509-510
    //
    // SysCtlPeripheralReady:
    // TivaWare API p.512

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
    {
    }

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }


    // ========================================================
    // 4. CONFIGURE PB2/PB3 FOR I2C0
    // ========================================================
    //
    // TM4C123GH6PM datasheet pp.998-999:
    // PB2 = I2C0SCL
    // PB3 = I2C0SDA
    //
    // GPIOPinConfigure:
    // TivaWare API p.268
    //
    // GPIOPinTypeI2C:
    // TivaWare API pp.276-277
    //
    // GPIOPinTypeI2CSCL:
    // TivaWare API p.277

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);


    // ========================================================
    // 5. INITIALIZE I2C MASTER
    // ========================================================
    //
    // I2CMasterInitExpClk:
    // TivaWare I2C Master API
    //
    // SysCtlClockGet:
    // TivaWare API p.492
    //
    // false selects standard-mode I2C (100 kbit/s).

    I2CMasterInitExpClk(I2C0_BASE,
                        SysCtlClockGet(),
                        false);


    // ========================================================
    // 6. WAKE MPU6500
    // ========================================================
    //
    // REG_PWR_MGMT_1 = 0x6B is an MPU6500 register,
    // NOT a TM4C/TivaWare register.
    //
    // Writing 0x00 wakes the sensor.
    //
    // SysCtlDelay behavior is documented in the
    // TivaWare System Control API.

    IMU_Write(REG_PWR_MGMT_1, 0x00);

    SysCtlDelay(SysCtlClockGet() / 30);


    // ========================================================
    // 7. CONFIGURE ACCELEROMETER RANGE
    // ========================================================
    //
    // MPU6500 ACCEL_CONFIG register = 0x1C
    //
    // ACCEL_FS currently equals 0x00:
    // +/-2g range.
    //
    // This value comes from MPU6500 documentation,
    // NOT TivaWare.

    IMU_Write(REG_ACCEL_CFG, ACCEL_FS);


    // ========================================================
    // 8. VERIFY MPU6500 ID
    // ========================================================
    //
    // WHO_AM_I register = 0x75
    //
    // Your sensor returned 0x70, identifying the
    // MPU6500-compatible device on the breakout.
    //
    // If the device does not respond with a supported ID,
    // all three LEDs turn on -> WHITE = error.

    IMU_Read(REG_WHO_AM_I, d, 1);

    whoami = d[0];

    if ((whoami != 0x70) &&
        (whoami != 0x71))
    {
        GPIOPinWrite(GPIO_PORTF_BASE,
                     ALL,
                     ALL);

        while (1)
        {
        }
    }


    // ========================================================
    // 9. MAIN PROGRAM LOOP
    // ========================================================

    while (1)
    {
        // ----------------------------------------------------
        // Read six consecutive accelerometer bytes.
        //
        // Starting at MPU6500 address 0x3B:
        //
        // d[0] = X high byte
        // d[1] = X low byte
        // d[2] = Y high byte
        // d[3] = Y low byte
        // d[4] = Z high byte
        // d[5] = Z low byte
        // ----------------------------------------------------

        IMU_Read(REG_ACCEL_XOUT, d, 6);


        // ----------------------------------------------------
        // Combine each pair of 8-bit measurements into
        // one signed 16-bit acceleration measurement.
        // ----------------------------------------------------

        ax = (int16_t)((d[0] << 8) | d[1]);
        ay = (int16_t)((d[2] << 8) | d[3]);
        az = (int16_t)((d[4] << 8) | d[5]);


        // ----------------------------------------------------
        // Decide which LED should turn on.
        //
        // At +/-2g:
        //
        // 1g   = 16384 counts
        // 0.9g = approximately 14745 counts
        //
        // Positive OR negative acceleration works because
        // orientation can produce either +1g or -1g.
        // ----------------------------------------------------

        led = 0;

        if ((ax > THRESH) || (ax < -THRESH))
        {
            led = RED;
        }

        else if ((ay > THRESH) || (ay < -THRESH))
        {
            led = BLUE;
        }

        else if ((az > THRESH) || (az < -THRESH))
        {
            led = GREEN;
        }


        // ----------------------------------------------------
        // Write selected color to LaunchPad RGB LED.
        //
        // GPIOPinWrite:
        // TivaWare API pp.286-287
        // ----------------------------------------------------

        GPIOPinWrite(GPIO_PORTF_BASE,
                     ALL,
                     led);


        // Small delay before taking another measurement.
        SysCtlDelay(SysCtlClockGet() / 30);
    }
}
