#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "Port_Config.h"

#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/qei.h"

#include "lcd_i2c.h"


/* Global Variables */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define WINDOW_OPEN  		        HIGH // Saving the state of the open window as 1
#define WINDOW_CLOSED  	            LOW // Saving the state of the closed window as 0
#define UP							HIGH // Saving the UP Direction of window as 1
#define DOWN						LOW // Saving the DOWN Direction of window as 0
#define MIDDLE					    2
#define STOP						0
#define PU							1
#define PD							2
#define DU							3
#define DD							4
#define ON                          LOW
#define OFF                         HIGH


bool driver_elevate_button_state; // variable for handling the state of the elevator button from the driver side
bool driver_lower_button_state;	// variable for handling the state of the lowering button from the driver side
bool passenger_elevate_button_state; // variable for handling the state of the elevator button from the Passenger side
bool passenger_lower_button_state; // variable for handling the state of the Lowering button from the Passenger side
bool lock_state; // variable for handling the state of the lock switch
bool upper_limit_switch_state; // variable for handling the state of the upper limit switch
bool lower_limit_switch_state; // variable for handling the state of the lower limit switch
int	 window_state = MIDDLE;	// variable for handling the state of the window state
bool operation;	// variable for handling the return from the manual control
int last_task = STOP;
volatile bool debounce_in_progress = false;

// Encoder globals
//static volatile uint32_t TOTAL_PULSES = 500UL;   // pulses from top→bottom
//volatile uint32_t windowPct      = 0;        // 0–100%

typedef struct {
    int      window_state;    // use your DU, DD, STOP, etc.
    bool     lock_state;
    uint32_t percent_open;
} StatusMsg_t;

// custom 8×5 “lock” glyph
static const uint8_t lockChar[8] = {
        0x0E, 0x11, 0x11, 0x11,
        0x1F, 0x1B, 0x1B, 0x1F
};
static const uint8_t unlockChar[8] = {
        0x0E, 0x11, 0x10, 0x10,
        0x1F, 0x1B, 0x1B, 0x1F
};

static QueueHandle_t xStatusQueue;

/* Handlers, Semaphores, Mutexes */
TaskHandle_t xDriverWindowElevateTaskHandle = NULL, xDriverWindowLowerTaskHandle = NULL;	// handler of the vDriverWindowElevateTask and vDriverWindowLowerTask
TaskHandle_t xPassengerWindowElevateTaskHandle = NULL, xPassengerWindowLowerTaskHandle = NULL; // handler of the vPassengerWindowElevateTask and vPassengerWindowLowerTask
TaskHandle_t xLockWindowsTaskHandle = NULL; // handler of the vLockWindowsTask
TaskHandle_t xUpperLimitTaskHandle = NULL, xLowerLimitTaskHandle = NULL; // handler of the vUpperLimitTask

TimerHandle_t xDebounceTimer;

/*––– Global semaphore handle –––*/
SemaphoreHandle_t xDriverUpSem = NULL, xDriverDownSem = NULL;
SemaphoreHandle_t xPassengerUpSem = NULL, xPassengerDownSem = NULL;
SemaphoreHandle_t xLockWindowsSem = NULL;
SemaphoreHandle_t xUpperLimitSem = NULL, xLowerLimitSem = NULL;



void vDriverWindowElevateTask(void *pvParameters); // RTOS task for controlling (elevating) a passenger window from the driver side

void vDriverWindowLowerTask(void *pvParameters); // RTOS task for controlling (lowering) a passenger window from the driver side

void vPassengerWindowElevateTask(void *pvParameters); // RTOS task for controlling (elevating) a passenger window from the passenger side

void vPassengerWindowLowerTask(void *pvParameters); // RTOS task for controlling (lowering) a passenger window from the passenger side

void vLockWindowsTask(void *pvParameters); // RTOS task for locking/unlocking the control from the passenger side

void vUpperLimitTask(void *pvParameters); // RTOS task for reacting to reaching the upper limit

void vLowerLimitTask(void *pvParameters); // RTOS task for reacting to reaching the lower limit

//void vCalibrationTask(void *pv);

//void vEncoderMonitorTask(void *pvParameters); // RTOS task for monitoring the encoder

void vLCDTask(void *pvParameters); // RTOS task for updating the LCD display

void vStatusProducerTask(void *pv); // RTOS task for producing status messages

void ISRHandlers(void); // ISR handler for GPIO interrupts

void vDebounceTimerCallback(TimerHandle_t xTimer);  // Callback function for the debounce timer

/*––– MAIN –––*/
int main(void) {
    // Clock
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ
    );

    // Init I/O
    PortA_Config();  // must set PA6, PA7 outputs; PA4..2 inputs w/ digital enable
    PortB_Config();  // QEI inputs (Encoder) / LCD sets PB2/SCL, PB3/SDA
    PortC_Config();  // limit switches, lock switch, IR sensor

    // LCD
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    LCD_I2C_Init();
    LCD_I2C_CreateChar(0, lockChar);
    LCD_I2C_CreateChar(1, unlockChar);

    // --- create queue & tasks ---
    xStatusQueue = xQueueCreate(1, sizeof(StatusMsg_t));
    xTaskCreate(vLCDTask,            "LCD",   128, NULL, 2, NULL);
    xTaskCreate(vStatusProducerTask, "Stat",  128, NULL, 1, NULL);

//    // 3) QEI0 on PB6/PB7
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)) {}
//
//    // Configure PB6/PB7 for QEI signals
//    GPIOPinTypeQEI(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
//
//    // Manually set PCTL bits for PB6 = PHA0, PB7 = PHB0 (function 5)
//    //   PB6 PCTL field = bits [27:24], PB7 = [31:28].
//    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R
//                         & ~(((uint32_t)0xF << (6*4)) | ((uint32_t)0xF << (7*4))))
//                        | ((uint32_t)5 << (6*4))
//                        | ((uint32_t)5 << (7*4));
//
//    // Initialize QEI: capture both edges, quadrature, no auto-reset
//    GPIOPadConfigSet(GPIO_PORTB_BASE,
//                     GPIO_PIN_6|GPIO_PIN_7,
//                     GPIO_STRENGTH_2MA,
//                     GPIO_PIN_TYPE_STD_WPU);
//    QEIConfigure(QEI0_BASE,
//                 QEI_CONFIG_CAPTURE_A_B
//                 | QEI_CONFIG_QUADRATURE
//                 | QEI_CONFIG_NO_RESET,
//                 0);
//    QEIEnable(QEI0_BASE);

    // Semaphores
    xDriverUpSem       = xSemaphoreCreateBinary();
    xDriverDownSem     = xSemaphoreCreateBinary();
    xPassengerUpSem    = xSemaphoreCreateBinary();
    xPassengerDownSem  = xSemaphoreCreateBinary();
    xLockWindowsSem    = xSemaphoreCreateBinary();
    xUpperLimitSem     = xSemaphoreCreateBinary();
    xLowerLimitSem     = xSemaphoreCreateBinary();

    // GPIO interrupts
    GPIOIntRegister(GPIO_PORTA_BASE, ISRHandlers);
    GPIOIntRegister(GPIO_PORTC_BASE, ISRHandlers);

    GPIOIntTypeSet(GPIO_PORTA_BASE, Driver_Elevate_Button, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTA_BASE, Driver_Lower_Button, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTA_BASE, Passenger_Elevate_Button, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTA_BASE, Passenger_Lower_Button, GPIO_RISING_EDGE);

    GPIOIntTypeSet(GPIO_PORTC_BASE, Window_Lock_Switch, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTC_BASE, Window_Upper_Limit, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTC_BASE, Window_Lower_Limit, GPIO_BOTH_EDGES);

    GPIOIntEnable(GPIO_PORTA_BASE,
                  Driver_Elevate_Button | Driver_Lower_Button |
                  Passenger_Elevate_Button | Passenger_Lower_Button);

    GPIOIntEnable(GPIO_PORTC_BASE, Window_Lock_Switch | Window_Upper_Limit | Window_Lower_Limit);

    IntEnable(INT_GPIOA_TM4C123);
    IntPrioritySet(INT_GPIOA_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY+1);
    IntEnable(INT_GPIOC_TM4C123);
    IntPrioritySet(INT_GPIOC_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY);

    // Create single-touch driver‑up task
    xTaskCreate(vDriverWindowElevateTask, "DrvUp", 128, NULL, 3, &xDriverWindowElevateTaskHandle);
    xTaskCreate(vDriverWindowLowerTask, "DrvDn", 128, NULL, 3, &xDriverWindowLowerTaskHandle);
    xTaskCreate(vPassengerWindowElevateTask, "PUp", 128, NULL, 3, &xPassengerWindowElevateTaskHandle);
    xTaskCreate(vPassengerWindowLowerTask, "PDn", 128, NULL, 3, &xPassengerWindowLowerTaskHandle);
    xTaskCreate(vLockWindowsTask, "LockCtrl", 128, NULL, 4, &xLockWindowsTaskHandle);
    xTaskCreate(vUpperLimitTask, "UpperLimit", 128, NULL, 5, &xUpperLimitTaskHandle);
    xTaskCreate(vLowerLimitTask, "LowerLimit", 128, NULL, 5, &xLowerLimitTaskHandle);
//    xTaskCreate(vEncoderMonitorTask, "EncMon", 128, NULL, 1, NULL);
//    xTaskCreate(vCalibrationTask, "Calib", 128, NULL, 2, NULL);

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

            // Line 2: lock symbol + percentage
            LCD_I2C_SetCursor(1, 0);
            LCD_I2C_Print("WinLk");
            LCD_I2C_WriteChar(msg.lock_state ? 1 : 0);
            snprintf(buf, sizeof(buf), " WPos:%u%%", (unsigned)msg.percent_open);
            LCD_I2C_Print(buf);
        }
    }
}

//—– Periodic status producer (for now sends static “0% stopped unlocked”) —–//
void vStatusProducerTask(void *pv) {
    StatusMsg_t msg, prev_msg = {-1, -1, 255};

    while (1) {
        msg.window_state  = last_task;
        msg.lock_state    = lock_state;
        msg.percent_open  = 0;

        // Only send if something changed
        if (msg.window_state != prev_msg.window_state ||
            msg.lock_state   != prev_msg.lock_state ||
            msg.percent_open != prev_msg.percent_open) {
            xQueueOverwrite(xStatusQueue, &msg);
            prev_msg = msg;
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // sample rate
    }
}


/*––– DRIVER ELEVATE TASK –––*/
void vDriverWindowElevateTask(void *pvParameters) {
    // taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xDriverUpSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xDriverUpSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(200));
        driver_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button);

        if (driver_elevate_button_state == HIGH && window_state != WINDOW_CLOSED) {
            // === MANUAL MODE ===
            // While held, move up
            while (driver_elevate_button_state == HIGH && window_state != WINDOW_CLOSED) {
                GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In1);
                operation = UP;
                window_state = MIDDLE;
                driver_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button);
                last_task = DU;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Stop
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
        else if (driver_elevate_button_state == LOW && window_state != WINDOW_CLOSED) {
            // === ONE‑TOUCH AUTO MODE ===
            // Start motor up
            uint8_t state = false;
            while (!state && window_state != WINDOW_CLOSED) {
                GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In1);
                operation = UP;
                window_state = MIDDLE;
                last_task = DU;
                state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button | Driver_Lower_Button | Passenger_Elevate_Button | Passenger_Lower_Button);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Stop
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
    }
}

/*––– DRIVER LOWER TASK –––*/
void vDriverWindowLowerTask(void *pvParameters) {
    // taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xDriverDownSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xDriverDownSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(200));
        driver_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Lower_Button);

        if (driver_lower_button_state == HIGH && window_state != WINDOW_OPEN) {
            // === MANUAL MODE ===
            // While held, move down
            while (driver_lower_button_state == HIGH && window_state != WINDOW_OPEN) {
                GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In2);
                operation = DOWN;
                window_state = MIDDLE;
                driver_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Lower_Button);
                last_task = DD;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Stop
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
        else if (driver_lower_button_state == LOW && window_state != WINDOW_OPEN) {
            // === ONE‑TOUCH AUTO MODE ===
            // Start motor down
            uint8_t state = false;
            while (!state && window_state != WINDOW_OPEN) {
                GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In2);
                operation = DOWN;
                window_state = MIDDLE;
                last_task = DD;
                state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button | Driver_Lower_Button | Passenger_Elevate_Button | Passenger_Lower_Button);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Stop
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
    }
}

/*––– PASSENGER ELEVATE TASK –––*/
void vPassengerWindowElevateTask(void *pvParameters) {
    // taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xPassengerUpSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xPassengerUpSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(200));
        passenger_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Elevate_Button);

        if (passenger_elevate_button_state == HIGH && window_state != WINDOW_CLOSED) {
            // === MANUAL MODE ===
            // While held, move up
            while (passenger_elevate_button_state == HIGH && window_state != WINDOW_CLOSED) {
                GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In1);
                last_task = PU;
                operation = UP;
                window_state = MIDDLE;
                passenger_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Elevate_Button);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Stop
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
        else if (passenger_elevate_button_state == LOW && window_state != WINDOW_CLOSED) {
            // === ONE‑TOUCH AUTO MODE ===
            // Start motor up
            uint8_t state = false;
            while (!state && window_state != WINDOW_CLOSED) {
                GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In1);
                operation = UP;
                last_task = PU;
                window_state = MIDDLE;
                state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button | Driver_Lower_Button | Passenger_Elevate_Button | Passenger_Lower_Button) && lock_state;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Stop
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
    }
}

/*––– PASSENGER LOWER TASK –––*/
void vPassengerWindowLowerTask(void *pvParameters) {
    // taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xPassengerDownSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xPassengerDownSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(200));
        passenger_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Lower_Button);

        if (passenger_lower_button_state == HIGH && window_state != WINDOW_OPEN) {
            // === MANUAL MODE ===
            // While held, move down
            while (passenger_lower_button_state == HIGH && window_state != WINDOW_OPEN) {
                GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In2);
                operation = DOWN;
                last_task = PD;
                window_state = MIDDLE;
                passenger_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Lower_Button);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Stop
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
        else if (passenger_lower_button_state == LOW && window_state != WINDOW_OPEN) {
            // === ONE‑TOUCH AUTO MODE ===
            // Start motor down
            uint8_t state = false;
            while (!state && window_state != WINDOW_OPEN) {
                GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In2);
                operation = DOWN;
                last_task = PD;
                window_state = MIDDLE;
                state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button | Driver_Lower_Button | Passenger_Elevate_Button | Passenger_Lower_Button) && lock_state;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Stop
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
    }
}

void vLockWindowsTask(void *pvParameters) {
    // taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xLockWindowsSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xLockWindowsSem, portMAX_DELAY);

        lock_state = GPIOPinRead(Sensors_Port, Window_Lock_Switch);

        if (lock_state == LOW) {
            if(passenger_elevate_button_state || passenger_lower_button_state){
                GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
                last_task = STOP;
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
    // taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xUpperLimitSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xUpperLimitSem, portMAX_DELAY);

        // Debounce
//        vTaskDelay(pdMS_TO_TICKS(10));
        upper_limit_switch_state = GPIOPinRead(Sensors_Port, Window_Upper_Limit);

        // Stopping the motor at upper limit
        if (upper_limit_switch_state == HIGH && operation == UP) {
            // stop motor…
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
            window_state = WINDOW_CLOSED;
            vTaskSuspend(xDriverWindowElevateTaskHandle);
            vTaskSuspend(xPassengerWindowElevateTaskHandle);
            // Calibrate top = zero pulses
//            QEIPositionSet(QEI0_BASE, 0);
        }
        else {
            // Re-allow window closing option
            vTaskResume(xDriverWindowElevateTaskHandle);
            vTaskResume(xPassengerWindowElevateTaskHandle);
        }
    }
}

void vLowerLimitTask(void *pvParameters) {
    // taking semaphore at the beginning for blocking the task
    xSemaphoreTake(xLowerLimitSem, 0);
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xLowerLimitSem, portMAX_DELAY);

        // Debounce
//        vTaskDelay(pdMS_TO_TICKS(10));
        lower_limit_switch_state = GPIOPinRead(Sensors_Port, Window_Lower_Limit);

        // Stopping the motor at lower limit
        if (lower_limit_switch_state == HIGH && operation == DOWN) {
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1 | DC_Motor_In2, 0);
            last_task = STOP;
            window_state = WINDOW_OPEN;
            vTaskSuspend(xDriverWindowLowerTaskHandle);
            vTaskSuspend(xPassengerWindowLowerTaskHandle);
            // Calibrate bottom = count pulses
//            QEIPositionSet(QEI0_BASE, 0);
//            vTaskDelay(pdMS_TO_TICKS(10));
//            TOTAL_PULSES = QEIPositionGet(QEI0_BASE);
        } else {
            // Re-allow window opening option
            vTaskResume(xDriverWindowLowerTaskHandle);
            vTaskResume(xPassengerWindowLowerTaskHandle);
        }
    }
}

//void vCalibrationTask(void *pv) {
//    // Drive down at half-speed (or by hand) until lower limit trips
//    GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In2);
//    // Block here until the switch reads HIGH
//    while(GPIOPinRead(Sensors_Port, Window_Lower_Limit)==LOW) {
//        vTaskDelay(pdMS_TO_TICKS(10));
//    }
//    GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
//
//    // Now record TOTAL_PULSES
//    QEIPositionSet(QEI0_BASE, 0);
//    vTaskDelay(pdMS_TO_TICKS(20)); // debounce
//    TOTAL_PULSES = QEIPositionGet(QEI0_BASE);
//
//    // Delete self
//    vTaskDelete(NULL);
//}

/*––– ENCODER MONITOR –––*/
//void vEncoderMonitorTask(void *pv) {
//    while (1) {
//        uint32_t pos = QEIPositionGet(QEI0_BASE);
//        // Always compute pct against a test constant
//        windowPct = (pos * 100UL) / TOTAL_PULSES;
//        vTaskDelay(pdMS_TO_TICKS(100));
//    }
//}

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

    // If it's *only* a button bounce, and we're still debouncing, clear PORTA *and* PORTC (so limit-switch flags don't stick) and exit.
    if( debounce_in_progress && a )
    {
        GPIOIntClear(GPIO_PORTA_BASE, a);
        GPIOIntClear(GPIO_PORTC_BASE, c);
        return;
    }

    // If this is the *first* button edge, start the debounce timer
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
        xSemaphoreGiveFromISR(xUpperLimitSem, &xHigherPriorityTaskWoken);
    }
    else if (c & Window_Lower_Limit) {
        GPIOIntClear(GPIO_PORTC_BASE, Window_Lower_Limit);
        xSemaphoreGiveFromISR(xLowerLimitSem, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vDebounceTimerCallback(TimerHandle_t xTimer) {
    debounce_in_progress = false;
}

#pragma clang diagnostic pop