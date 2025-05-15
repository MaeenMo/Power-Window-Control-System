#include "Config.h"
#include "Helper_Functions.h"
#include "WindowTasks.h"
#include "Safety.h"
#include "RotaryEncoder.h"

/* Global Variables */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

volatile bool debounce_in_progress = false; // variable for handling the debounce state

// Encoder globals
static volatile uint32_t TOTAL_PULSES = 150UL;   // pulses from top→bottom
volatile uint32_t windowPct      = 0;        // 0–100%

typedef struct {
    uint8_t      window_state;
    bool     lock_state;
    uint32_t percent_open;
} StatusMsg_t;

static const uint8_t lockChar[8] = {
        0x0E, 0x11, 0x11, 0x11,
        0x1F, 0x1B, 0x1B, 0x1F
};
static const uint8_t unlockChar[8] = {
        0x0E, 0x11, 0x10, 0x10,
        0x1F, 0x1B, 0x1B, 0x1F
};

static QueueHandle_t xStatusQueue;

/* Handlers, Semaphores */
TaskHandle_t xDriverWindowElevateTaskHandle = NULL, xDriverWindowLowerTaskHandle = NULL;	// handler of the vDriverWindowElevateTask and vDriverWindowLowerTask
TaskHandle_t xPassengerWindowElevateTaskHandle = NULL, xPassengerWindowLowerTaskHandle = NULL; // handler of the vPassengerWindowElevateTask and vPassengerWindowLowerTask
TaskHandle_t xLockWindowsTaskHandle = NULL; // handler of the vLockWindowsTask
TaskHandle_t xUpperLimitTaskHandle = NULL, xLowerLimitTaskHandle = NULL; // handler of the vUpperLimitTask
TaskHandle_t xObstacleDetectionHandle = NULL; // handler of the vObstacleDetection

TimerHandle_t xDebounceTimer;

/*––– Global semaphore handle –––*/
SemaphoreHandle_t xDriverUpSem = NULL, xDriverDownSem = NULL;
SemaphoreHandle_t xPassengerUpSem = NULL, xPassengerDownSem = NULL;
SemaphoreHandle_t xLockWindowsSem = NULL;
SemaphoreHandle_t xUpperLimitSem = NULL, xLowerLimitSem = NULL;
SemaphoreHandle_t xObstacleDetectionSem = NULL;

/*––– MAIN –––*/
int main(void) {
    // Clock
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ
    );

    // Init I/O
    PortA_Config();  // must set PA6, PA7 outputs; PA4..2 inputs w/ digital enable
    PortB_Config_LCD();  // LCD sets PB2-SCL, PB3-SDA
    PortC_Config();  // limit switches, lock switch, IR sensor
    PortD_Config_QEI();  // QEI inputs (Encoder) PD6-CLK PD7-DT

    // LCD
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    LCD_I2C_Init();
    LCD_I2C_CreateChar(0, lockChar);
    LCD_I2C_CreateChar(1, unlockChar);

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
            pdMS_TO_TICKS(350),              // 20 ms debounce
            pdFALSE,                        // One-shot timer
            (void*)0,                       // Timer ID
            vDebounceTimerCallback          // Callback function
    );

    // Start scheduler
    vTaskStartScheduler();

    while (1);
    // should never get here
}

//—– LCD updater task —–//
void vLCDTask(void *pvParameters) {
    StatusMsg_t msg;
    char buf[17];

    while (1) {
        // Wait indefinitely for a new status update
        if (xQueueReceive(xStatusQueue, &msg, portMAX_DELAY) == pdTRUE) {
            LCD_I2C_Clear();

            // Line 1: window state
            LCD_I2C_SetCursor(0, 0);
            switch(msg.window_state) {
                case DU:  LCD_I2C_Print("WinStat: Closing"); break;
                case DD:  LCD_I2C_Print("WinStat: Opening"); break;
                case PU:  LCD_I2C_Print("WinStat: Closing"); break;
                case PD:  LCD_I2C_Print("WinStat: Opening"); break;
                default: {
                    if (window_state == WINDOW_OPEN)
                        LCD_I2C_Print("WinStat: Opened");
                    else if (window_state == WINDOW_CLOSED)
                        LCD_I2C_Print("WinStat: Closed");
                    else
                        LCD_I2C_Print("WinStat: Stopped");
                }
            }

            // Line 2: lock symbol + window percentage
            LCD_I2C_SetCursor(1, 0);
            LCD_I2C_Print("WinLk");
            LCD_I2C_WriteChar(msg.lock_state ? 1 : 0);
            snprintf(buf, sizeof(buf), " WPos:%u%%", (unsigned)msg.percent_open);
            LCD_I2C_Print(buf);
        }
    }
}

//—– Periodic status producer —–//
void vStatusProducerTask(void *pv) {
    StatusMsg_t msg, prev_msg = {-1, -1, 255}; // Initialize to invalid values

    while (1) {
        msg.window_state  = last_task;
        msg.lock_state    = lock_state;
        msg.percent_open  = windowPct;

        // Only send if something changed
        if (msg.window_state != prev_msg.window_state ||
            msg.lock_state   != prev_msg.lock_state ||
            msg.percent_open != prev_msg.percent_open) {
            xQueueOverwrite(xStatusQueue, &msg);
            prev_msg = msg;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/*––– DRIVER ELEVATE TASK –––*/
void vDriverWindowElevateTask(void *pvParameters) {
    // Taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xDriverUpSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xDriverUpSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(200));
        bool driver_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button);

        if (driver_elevate_button_state == HIGH && window_state != WINDOW_CLOSED) {
            // === MANUAL MODE ===
            objDet = false;
            while (driver_elevate_button_state == HIGH && window_state != WINDOW_CLOSED && !objDet) {
                closeWindow(DRIVER_WINDOW);
                vTaskDelay(pdMS_TO_TICKS(10));
                driver_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button);
            }
        }
        else if (driver_elevate_button_state == LOW && window_state != WINDOW_CLOSED) {
            // === ONE‑TOUCH AUTO MODE ===
            autoMode = true;
            objDet = false;
            while (autoMode && window_state != WINDOW_CLOSED && !objDet) {
                closeWindow(DRIVER_WINDOW);
                vTaskDelay(pdMS_TO_TICKS(10));
                autoMode = !GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button | Driver_Lower_Button | Passenger_Elevate_Button | Passenger_Lower_Button);
            }
        }
        // Stop
        stopWindow();
    }
}

/*––– DRIVER LOWER TASK –––*/
void vDriverWindowLowerTask(void *pvParameters) {
    // Taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xDriverDownSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xDriverDownSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(200));
        bool driver_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Lower_Button);

        if (driver_lower_button_state == HIGH && window_state != WINDOW_OPEN) {
            // === MANUAL MODE ===
            objDet = false;
            while (driver_lower_button_state == HIGH && window_state != WINDOW_OPEN && !objDet) {
                openWindow(DRIVER_WINDOW);
                vTaskDelay(pdMS_TO_TICKS(10));
                driver_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Lower_Button);
            }
        }
        else if (driver_lower_button_state == LOW && window_state != WINDOW_OPEN) {
            // === ONE‑TOUCH AUTO MODE ===
            autoMode = true;
            objDet = false;
            while (autoMode && window_state != WINDOW_OPEN && !objDet) {
                openWindow(DRIVER_WINDOW);
                vTaskDelay(pdMS_TO_TICKS(10));
                autoMode = !GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button | Driver_Lower_Button | Passenger_Elevate_Button | Passenger_Lower_Button);
            }
        }
        // Stop
        stopWindow();
    }
}

/*––– PASSENGER ELEVATE TASK –––*/
void vPassengerWindowElevateTask(void *pvParameters) {
    // Taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xPassengerUpSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xPassengerUpSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(200));
        passenger_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Elevate_Button);

        if (passenger_elevate_button_state == HIGH && window_state != WINDOW_CLOSED) {
            // === MANUAL MODE ===
            objDet = false;
            while (passenger_elevate_button_state == HIGH && window_state != WINDOW_CLOSED && !objDet) {
                closeWindow(PASSENGER_WINDOW);
                vTaskDelay(pdMS_TO_TICKS(10));
                passenger_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Elevate_Button);
            }
        }
        else if (passenger_elevate_button_state == LOW && window_state != WINDOW_CLOSED) {
            // === ONE‑TOUCH AUTO MODE ===
            autoMode = true;
            objDet = false;
            while (autoMode && window_state != WINDOW_CLOSED && !objDet) {
                closeWindow(PASSENGER_WINDOW);
                vTaskDelay(pdMS_TO_TICKS(10));
                autoMode = !GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button | Driver_Lower_Button | Passenger_Elevate_Button | Passenger_Lower_Button) && lock_state;
            }
        }
       // Stop
       stopWindow();
    }
}

/*––– PASSENGER LOWER TASK –––*/
void vPassengerWindowLowerTask(void *pvParameters) {
    // Taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xPassengerDownSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xPassengerDownSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(200));
        passenger_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Lower_Button);

        if (passenger_lower_button_state == HIGH && window_state != WINDOW_OPEN) {
            // === MANUAL MODE ===
            objDet = false;
            while (passenger_lower_button_state == HIGH && window_state != WINDOW_OPEN && !objDet) {
                openWindow(PASSENGER_WINDOW);
                vTaskDelay(pdMS_TO_TICKS(10));
                passenger_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Lower_Button);
            }
        }
        else if (passenger_lower_button_state == LOW && window_state != WINDOW_OPEN) {
            // === ONE‑TOUCH AUTO MODE ===
            autoMode = true;
            objDet = false;
            while (autoMode && window_state != WINDOW_OPEN && !objDet) {
                openWindow(PASSENGER_WINDOW);
                vTaskDelay(pdMS_TO_TICKS(10));
                autoMode = !GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button | Driver_Lower_Button | Passenger_Elevate_Button | Passenger_Lower_Button) && lock_state;
            }
        }
        // Stop
        stopWindow();
    }
}

void vLockWindowsTask(void *pvParameters) {
    // Taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xLockWindowsSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xLockWindowsSem, portMAX_DELAY);

        lock_state = GPIOPinRead(Sensors_Port, Window_Lock_Switch);

        if (lock_state == LOW) {
            if(passenger_elevate_button_state || passenger_lower_button_state){
                stopWindow();
            }
            // Disabling passenger control
            vTaskSuspend(xPassengerWindowElevateTaskHandle);
            vTaskSuspend(xPassengerWindowLowerTaskHandle);
        }
        else {
            // Allowing passenger control
            vTaskResume(xPassengerWindowElevateTaskHandle);
            vTaskResume(xPassengerWindowLowerTaskHandle);
        }
    }
}

void vUpperLimitTask(void *pvParameters) {
    // Taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xUpperLimitSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xUpperLimitSem, portMAX_DELAY);

        bool upper_limit_switch_state;

        if (encoder_limit){
            upper_limit_switch_state = HIGH;
            encoder_limit = false;
        }
        else upper_limit_switch_state = GPIOPinRead(Sensors_Port, Window_Upper_Limit);

        // Stopping the motor at upper limit
        if (upper_limit_switch_state == HIGH && operation == UP) {
            stopWindow();
            window_state = WINDOW_CLOSED;
            vTaskSuspend(xDriverWindowElevateTaskHandle);
            vTaskSuspend(xPassengerWindowElevateTaskHandle);
            QEIPositionSet(QEI0_BASE, TOTAL_PULSES);
            pos = QEIPositionGet(QEI0_BASE);
            prev_pos = pos;
            windowPct = 100;
            xSemaphoreGive(xUpperLimitSem);
        } else {
            // Re-allow window closing option
            vTaskResume(xDriverWindowElevateTaskHandle);
            vTaskResume(xPassengerWindowElevateTaskHandle);
        }
    }
}

void vLowerLimitTask(void *pvParameters) {
    // Taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xLowerLimitSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xLowerLimitSem, portMAX_DELAY);

        bool lower_limit_switch_state;

        if (encoder_limit){
            lower_limit_switch_state = HIGH;
            encoder_limit = false;
        }
        else lower_limit_switch_state = GPIOPinRead(Sensors_Port, Window_Lower_Limit);

        // Stopping the motor at lower limit
        if (lower_limit_switch_state == HIGH && operation == DOWN) {
            stopWindow();
            window_state = WINDOW_OPEN;
            vTaskSuspend(xDriverWindowLowerTaskHandle);
            vTaskSuspend(xPassengerWindowLowerTaskHandle);
            QEIPositionSet(QEI0_BASE, 0);
            pos = QEIPositionGet(QEI0_BASE);
            prev_pos = pos;
            windowPct = 0;
            xSemaphoreGive(xLowerLimitSem);
        } else {
            // Re-allow window opening option
            vTaskResume(xDriverWindowLowerTaskHandle);
            vTaskResume(xPassengerWindowLowerTaskHandle);
        }
    }
}

/*––– OBSTACLE DETECTION TASK –––*/
void vObstacleDetection(void *pvParameters) {
    // Taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xObstacleDetectionSem, 0);
    while (1) {
        xSemaphoreTake(xObstacleDetectionSem, portMAX_DELAY);

        bool object_detection_switch_state = GPIOPinRead(Sensors_Port,Object_Detection_Sensor);
        // Process the IR sensor state

        if ( object_detection_switch_state == HIGH && operation == UP) {  // Function to check if an obstacle is detected
            objDet = true; // Set the object detection flag to stop the window in both manual mode & automatic mode
            openWindow(DRIVER_WINDOW);
            vTaskSuspend(xDriverWindowElevateTaskHandle);
            vTaskSuspend(xPassengerWindowElevateTaskHandle);
            vTaskDelay(pdMS_TO_TICKS(500));
            stopWindow();
            vTaskResume(xDriverWindowElevateTaskHandle);
            vTaskResume(xPassengerWindowElevateTaskHandle);
        }
    }
}

/*––– ENCODER MONITOR –––*/
void vEncoderMonitorTask(void *pv) {
    QEIPositionSet(QEI0_BASE, 0);
    while (1) {
        int8_t dir = QEIDirectionGet(QEI0_BASE);  // +1 = CW, -1 = CCW

        if (operation == UP && dir > 0) {
            // Closing: percentage increases
            pos = QEIPositionGet(QEI0_BASE);
            if (pos > TOTAL_PULSES && pos < TOTAL_PULSES + 20) {
                QEIPositionSet(QEI0_BASE, TOTAL_PULSES);
                pos = QEIPositionGet(QEI0_BASE);
            } else if (pos > 450) {
                QEIPositionSet(QEI0_BASE, 0);
                pos = QEIPositionGet(QEI0_BASE);
            }
            if (pos > prev_pos) {
                windowPct = (pos * 100UL / TOTAL_PULSES);
                prev_pos = pos;
            }
            else QEIPositionSet(QEI0_BASE, prev_pos);
            if (windowPct > 100) windowPct = 100;
            if (windowPct == 100){
                prev_pos = TOTAL_PULSES;
                trigger_limit_semaphore(xUpperLimitSem, &encoder_limit);
            }
        } else if (operation == DOWN && dir < 0) {
            // Opening: percentage decreases
            pos = QEIPositionGet(QEI0_BASE);
            if (pos > TOTAL_PULSES && pos < TOTAL_PULSES + 20) {
                QEIPositionSet(QEI0_BASE, TOTAL_PULSES);
                pos = QEIPositionGet(QEI0_BASE);
            } else if (pos > 450) {
                QEIPositionSet(QEI0_BASE, 0);
                pos = QEIPositionGet(QEI0_BASE);
            }
            if (pos < prev_pos) {
                windowPct = (pos * 100UL) / TOTAL_PULSES;
                prev_pos = pos;
            }
            else QEIPositionSet(QEI0_BASE, prev_pos);
            if (windowPct < 0) windowPct = 0;
            if (windowPct > 100) windowPct = 100;
            if (windowPct == 0){
                prev_pos = 0;
                trigger_limit_semaphore(xLowerLimitSem, &encoder_limit);
            }
        } else if (operation == STOP){
            QEIPositionSet(QEI0_BASE, pos); // Set the position to the last known value
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void trigger_limit_semaphore(SemaphoreHandle_t xLimitSem, bool* encoder_limit) {
    *encoder_limit = true;

    // First trigger
    xSemaphoreGive(xLimitSem);
    taskYIELD();
    vTaskDelay(pdMS_TO_TICKS(1));

    // Second trigger
    xSemaphoreGive(xLimitSem);
    taskYIELD();
    vTaskDelay(pdMS_TO_TICKS(1));
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