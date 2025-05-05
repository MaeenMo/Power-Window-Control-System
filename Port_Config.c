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
    GPIOIntEnable(GPIO_PORTA_BASE,
                  Driver_Elevate_Button | Driver_Lower_Button |
                  Passenger_Elevate_Button | Passenger_Lower_Button);

    IntEnable(INT_GPIOA_TM4C123);
    IntPrioritySet(INT_GPIOA_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY+1);
}

void PortB_Config(void)
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
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C   (GPIO_PORTB_BASE, GPIO_PIN_3);
}

void PortC_Config(void) {
    // Enable the clock of PORTC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    // Configure pins PC4, PC5, PC6, and PC7 as inputs
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    GPIOIntTypeSet(GPIO_PORTC_BASE,
                   Window_Lock_Switch | Window_Upper_Limit |
                   Window_Lower_Limit | Object_Detection_Sensor
                   , GPIO_BOTH_EDGES);

    GPIOIntEnable(GPIO_PORTC_BASE, Window_Lock_Switch | Window_Upper_Limit | Window_Lower_Limit | Object_Detection_Sensor);

    IntEnable(INT_GPIOC_TM4C123);
    IntPrioritySet(INT_GPIOC_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY);
}