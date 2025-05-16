#include "Config.h"
#include "Helper_Functions.h"
#include "WindowTasks.h"
#include "Safety.h"
#include "RotaryEncoder.h"
#include "LCD.h"

/* Global Variables */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

volatile bool debounce_in_progress = false; // variable for handling the debounce state

TimerHandle_t xDebounceTimer;

/*––– MAIN –––*/
int main(void) {
    // Clock
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ
    );

    // Init I/O
    PortA_Config();         // must set PA6, PA7 outputs; PA4..2 inputs w/ digital enable
    PortB_Config_LCD();     // LCD sets PB2-SCL, PB3-SDA
    PortC_Config();         // limit switches, lock switch, IR sensor
    PortD_Config_QEI();     // QEI inputs (Encoder) PD6-CLK PD7-DT

    // LCD
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    LCD_I2C_Init();

    // --- create queue & tasks ---
    xStatusQueue = xQueueCreate(1, sizeof(StatusMsg_t));

    // Semaphores
    xDriverUpSem       = xSemaphoreCreateBinary();
    xDriverDownSem     = xSemaphoreCreateBinary();
    xPassengerUpSem    = xSemaphoreCreateBinary();
    xPassengerDownSem  = xSemaphoreCreateBinary();
    xLockWindowsSem    = xSemaphoreCreateBinary();
    xUpperLimitSem     = xSemaphoreCreateBinary();
    xLowerLimitSem     = xSemaphoreCreateBinary();
    xObstacleDetectionSem = xSemaphoreCreateBinary();

    // GPIO interrupts
    GPIOIntRegister(GPIO_PORTA_BASE, ISRHandlers);
    GPIOIntRegister(GPIO_PORTC_BASE, ISRHandlers);

    // Create single-touch driver‑up task
    xTaskCreate(vDriverWindowElevateTask, "DrvUp", 128, NULL, 3, &xDriverWindowElevateTaskHandle);
    xTaskCreate(vDriverWindowLowerTask, "DrvDn", 128, NULL, 3, &xDriverWindowLowerTaskHandle);
    xTaskCreate(vPassengerWindowElevateTask, "PUp", 128, NULL, 3, &xPassengerWindowElevateTaskHandle);
    xTaskCreate(vPassengerWindowLowerTask, "PDn", 128, NULL, 3, &xPassengerWindowLowerTaskHandle);
    xTaskCreate(vLockWindowsTask, "LockCtrl", 128, NULL, 4, &xLockWindowsTaskHandle);
    xTaskCreate(vUpperLimitTask, "UpperLimit", 128, NULL, 5, &xUpperLimitTaskHandle);
    xTaskCreate(vLowerLimitTask, "LowerLimit", 128, NULL, 5, &xLowerLimitTaskHandle);
    xTaskCreate(vObstacleDetection, "ObstacleDet", 128, NULL, 6, &xObstacleDetectionHandle);
    xTaskCreate(vLCDTask,            "LCD",   128, NULL, 2, NULL);
    xTaskCreate(vStatusProducerTask, "Stat",  128, NULL, 1, NULL);
    xTaskCreate(vEncoderMonitorTask,       "Enc",   128, NULL, 1, NULL);

    xDebounceTimer = xTimerCreate(
            "DebounceTimer",                // Timer name
            pdMS_TO_TICKS(350),             // Timer period
            pdFALSE,                        // One-shot timer
            (void*)0,                       // Timer ID
            vDebounceTimerCallback          // Callback function
    );

    // Start scheduler
    vTaskStartScheduler();

    while (1);
    // should never get here
}

/* Idle Task for sleeping the processor */
void vApplicationIdleHook( void ) {
    // Checking for initial condition of switch
    lock_state = GPIOPinRead(Sensors_Port,Window_Lock_Switch);
    // Disabling the passenger control
    if (lock_state == LOW){
        vTaskSuspend(xPassengerWindowElevateTaskHandle);
        vTaskSuspend(xPassengerWindowLowerTaskHandle);
    }
    /* Put the microcontroller in a low power mode */
    SysCtlSleep();
}

/*––– ISR HANDLER –––*/
void ISRHandlers(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Read *all* pending (masked) interrupts on each port
    uint32_t a = GPIOIntStatus(GPIO_PORTA_BASE, true);
    uint32_t c = GPIOIntStatus(GPIO_PORTC_BASE, true);

    // If it's a button bounce, and we're still debouncing, clear PORTA and PORTC.
    if( debounce_in_progress && a )
    {
        GPIOIntClear(GPIO_PORTA_BASE, a);
        GPIOIntClear(GPIO_PORTC_BASE, c);
        return;
    }

    // If this is the first button edge, start the debounce timer
    if( a )
    {
        debounce_in_progress = true;
        xTimerStartFromISR(xDebounceTimer, &xHigherPriorityTaskWoken);
    }

    // — PORT A —
    if (a & Driver_Elevate_Button) {
        if(last_task == STOP){
            // Wake the driver‑up task
            xSemaphoreGiveFromISR(xDriverUpSem, &xHigherPriorityTaskWoken);
        }
    }
    else if (a & Driver_Lower_Button) {
        if (last_task == STOP) {
            // Wake the passenger‑up task
            xSemaphoreGiveFromISR(xDriverDownSem, &xHigherPriorityTaskWoken);
        }
    }
    else if (a & Passenger_Elevate_Button) {
        if (lock_state == HIGH && last_task == STOP) {
            // Wake the passenger‑up task
            xSemaphoreGiveFromISR(xPassengerUpSem, &xHigherPriorityTaskWoken);
        }
    }
    else if (a & Passenger_Lower_Button) {
        if (lock_state == HIGH && last_task == STOP) {
            // Wake the passenger‑down task
            xSemaphoreGiveFromISR(xPassengerDownSem, &xHigherPriorityTaskWoken);
        }
    }
    // — PORT C —
    else if (c & Window_Lock_Switch) {
        GPIOIntClear(GPIO_PORTC_BASE, Window_Lock_Switch);
        xSemaphoreGiveFromISR(xLockWindowsSem, &xHigherPriorityTaskWoken);
    }
    else if (c & Window_Upper_Limit) {
        GPIOIntClear(GPIO_PORTC_BASE, Window_Upper_Limit);

        uint8_t good = 0;
        for (uint8_t i = 0; i < 3; ++i) {
            if (GPIOPinRead(Sensors_Port, Window_Upper_Limit))
                good++;
            for (uint8_t i = 0; i < 10; ++i);
        }
        if (good == 0x3) {
            xSemaphoreGiveFromISR(xUpperLimitSem, &xHigherPriorityTaskWoken);
        }

    }
    else if (c & Window_Lower_Limit) {
        GPIOIntClear(GPIO_PORTC_BASE, Window_Lower_Limit);

        uint8_t good = 0;
        for (uint8_t i = 0; i < 3; ++i) {
            if (GPIOPinRead(Sensors_Port, Window_Lower_Limit))
                good++;
            for (uint8_t i = 0; i < 10; i++);
        }
        if (good == 0x3) {
            xSemaphoreGiveFromISR(xLowerLimitSem, &xHigherPriorityTaskWoken);
        }
    }
    else if (c & Object_Detection_Sensor) {
        GPIOIntClear(GPIO_PORTC_BASE, Object_Detection_Sensor);

        if (operation == UP) {
            uint8_t good = 0;
            for (uint8_t i = 0; i < 3; ++i) {
                if (GPIOPinRead(Sensors_Port, Object_Detection_Sensor))
                    good++;
                for (uint8_t i = 0; i < 10; ++i);
            }
            if (good == 0x3) {
                xSemaphoreGiveFromISR(xObstacleDetectionSem, &xHigherPriorityTaskWoken);
            }
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vDebounceTimerCallback(TimerHandle_t xTimer) {
    debounce_in_progress = false;
}

#pragma clang diagnostic pop