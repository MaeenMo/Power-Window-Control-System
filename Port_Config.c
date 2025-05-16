#include "Config.h"

void PortA_Config(void) {
    // Enable the clock of PORTA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    // Configure PA6 & PA7 as outputs and PA2..5 as inputs
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    GPIOIntTypeSet(GPIO_PORTA_BASE,
                   Driver_Elevate_Button | Driver_Lower_Button |
                   Passenger_Elevate_Button | Passenger_Lower_Button
                   , GPIO_RISING_EDGE);

    // Enable the pins for GPIO
    GPIOIntEnable(GPIO_PORTA_BASE,
                  Driver_Elevate_Button | Driver_Lower_Button |
                  Passenger_Elevate_Button | Passenger_Lower_Button);

    // Set the interrupt priority
    IntEnable(INT_GPIOA_TM4C123);
    IntPrioritySet(INT_GPIOA_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY+1);

    // Set the initial state of the motor control pins to LOW
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7, 0);
}

void PortB_Config_LCD(void)
{
    // Enable GPIOB and I2C0 peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}

    // Configure PB2 as SCL, PB3 as SDA
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Enable the pins for I2C
    GPIOPadConfigSet(GPIO_PORTB_BASE,
                     GPIO_PIN_2|GPIO_PIN_3,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    // Set the I2C pins to I2C mode
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C   (GPIO_PORTB_BASE, GPIO_PIN_3);
}

void PortC_Config(void) {
    // Enable the clock of PORTC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    // Configure pins PC4, PC5, PC6, and PC7 as inputs
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    GPIOIntTypeSet(GPIO_PORTC_BASE, Window_Lock_Switch, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTC_BASE,
                   Window_Upper_Limit | Window_Lower_Limit |
                   Object_Detection_Sensor, GPIO_RISING_EDGE);

    GPIOIntEnable(GPIO_PORTC_BASE, Window_Lock_Switch | Window_Upper_Limit | Window_Lower_Limit | Object_Detection_Sensor);

    IntEnable(INT_GPIOC_TM4C123);
    IntPrioritySet(INT_GPIOC_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY);
}

void PortD_Config_QEI(void)
{
    // Turn on GPIOD and QEI0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));

    // Unlock PD7
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    // Mux PD6→PHA0 and PD7→PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7);

    // Pull-ups so encoder lines don’t float
    GPIOPadConfigSet(GPIO_PORTD_BASE,
                     GPIO_PIN_6|GPIO_PIN_7,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    // Configure & start QEI
    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_RESET, 0x000001F4);

    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() / 10);
    QEIVelocityEnable(QEI0_BASE);

    QEIPositionSet(QEI0_BASE, 0);
    QEIEnable(QEI0_BASE);
}
