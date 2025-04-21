#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "Port_Config.h"

#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/qei.h"



/* Global Variables */
#define WINDOW_OPEN  		        HIGH																											// Saving the state of the open window as 1
#define WINDOW_CLOSED  	            LOW																												// Saving the state of the closed window as 0
#define UP							HIGH																											// Saving the UP Direction of window as 1
#define DOWN						LOW																												// Saving the DOWN Direction of window as 0
#define MIDDLE					    2
#define STOP						0
#define PU							1
#define PD							2
#define DU							3
#define DD							4
#define ALL_BUTTONS  (Driver_Elevate_Button | Driver_Lower_Button | Passenger_Elevate_Button | Passenger_Lower_Button)

bool driver_elevate_button_state;																									// variable for handling the state of the elevator button from the driver side
bool driver_lower_button_state;																										// variable for handling the state of the lowering button from the driver side
bool passenger_elevate_button_state;																							// variable for handling the state of the elevator button from the Passenger side
bool passenger_lower_button_state;																								// variable for handling the state of the Lowering button from the Passenger side
bool lock_state;                                                                                                                  // variable for handling the state of the lock switch
int	 window_state = MIDDLE;																												// variable for handling the state of the window window state
bool operation;																																	// variable for handling the return from the manual control
int last_task = STOP;
volatile bool debounce_in_progress = false;

/* Handlers, Semaphores, Mutexes */
TaskHandle_t 				xDriverWindowElevateTaskHandle 									= NULL;				// handler of the vDriverWindowElevateTask
TaskHandle_t 				xDriverWindowLowerTaskHandle 										= NULL;				// handler of the vDriverWindowLowerTask
TaskHandle_t 				xPassengerWindowElevateTaskHandle 							= NULL;				// handler of the vPassengerWindowElevateTask
TaskHandle_t 				xPassengerWindowLowerTaskHandle 								= NULL;				// handler of the vPassengerWindowLowerTask

TimerHandle_t xDebounceTimer;

/*––– Global semaphore handle –––*/
static SemaphoreHandle_t xDriverUpSem, xDriverDownSem = NULL;
static SemaphoreHandle_t xPassengerUpSem, xPassengerDownSem = NULL;



void vDriverWindowElevateTask(void *pvParameters);
/*
	RTOS task for controlling (elevating) a passenger window from the driver side
*/
void vDriverWindowLowerTask(void *pvParameters);
/*
	RTOS task for controlling (lowering) a passenger window from the driver side
*/
void vPassengerWindowElevateTask(void *pvParameters);
/*
	RTOS task for controlling (elevating) a passenger window from the passenger side
*/
void vPassengerWindowLowerTask(void *pvParameters);
/*
	RTOS task for controlling (lowering) a passenger window from the passenger side
*/
void ISRHandlers(void);
/*
	Function responsible for handling the interrupts
*/
void vDebounceTimerCallback(TimerHandle_t xTimer);

/*––– MAIN –––*/
int main(void) {
    // Clock
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ
    );

    // Init I/O
    PortA_Config();  // must set PA6, PA7 outputs; PA4..2 inputs w/ digital enable
//    PortC_Config();  // limit switches, lock switch, IR sensor

    // Semaphores
    xDriverUpSem       = xSemaphoreCreateBinary();
    xDriverDownSem     = xSemaphoreCreateBinary();
    xPassengerUpSem    = xSemaphoreCreateBinary();
    xPassengerDownSem  = xSemaphoreCreateBinary();

    // GPIO interrupts
    GPIOIntRegister(GPIO_PORTA_BASE, ISRHandlers);

    GPIOIntTypeSet(GPIO_PORTA_BASE, Driver_Elevate_Button, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTA_BASE, Driver_Lower_Button, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTA_BASE, Passenger_Elevate_Button, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTA_BASE, Passenger_Lower_Button, GPIO_RISING_EDGE);


    GPIOIntEnable(GPIO_PORTA_BASE,
                  Driver_Elevate_Button | Driver_Lower_Button |
                  Passenger_Elevate_Button | Passenger_Lower_Button);

    IntEnable(INT_GPIOA_TM4C123);
    IntPrioritySet(INT_GPIOA_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY);

    // Create single-touch driver‑up task
    xTaskCreate(vDriverWindowElevateTask, "DrvUp", 128, NULL, 4, &xDriverWindowElevateTaskHandle);
    xTaskCreate(vDriverWindowLowerTask,    "DrvDn",   128, NULL, 4, &xDriverWindowLowerTaskHandle);
    xTaskCreate(vPassengerWindowElevateTask, "PUp",   128, NULL, 3, &xPassengerWindowElevateTaskHandle);
    xTaskCreate(vPassengerWindowLowerTask, "PDn",    128, NULL, 3, &xPassengerWindowLowerTaskHandle);

    xDebounceTimer = xTimerCreate(
            "DebounceTimer",                // Timer name
            pdMS_TO_TICKS(350),              // 20 ms debounce
            pdFALSE,                        // One-shot timer
            (void*)0,                       // Timer ID (optional)
            vDebounceTimerCallback          // Callback function
    );

    // Start scheduler
    vTaskStartScheduler();

    while (1) {;
        // should never get here
    }
}

/*––– DRIVER ELEVATE TASK –––*/
void vDriverWindowElevateTask(void *pvParameters) {
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xDriverUpSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(150));
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
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In1);
            operation = UP;
            window_state = MIDDLE;
            last_task = DU;
        }

        // Allow other tasks/idle to run
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*––– DRIVER LOWER TASK –––*/
void vDriverWindowLowerTask(void *pvParameters) {
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xDriverDownSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(150));
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
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In2);
            operation = DOWN;
            window_state = MIDDLE;
            last_task = DD;
        }

        // Allow other tasks/idle to run
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*––– PASSENGER ELEVATE TASK –––*/
void vPassengerWindowElevateTask(void *pvParameters) {
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xPassengerUpSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(150));
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
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In1);
            operation = UP;
            last_task = PU;
            window_state = MIDDLE;
        }

        // Allow other tasks/idle to run
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*––– PASSENGER LOWER TASK –––*/
void vPassengerWindowLowerTask(void *pvParameters) {
    while (1) {
        // Block until ISR gives us the semaphore
        xSemaphoreTake(xPassengerDownSem, portMAX_DELAY);

        // Debounce
        vTaskDelay(pdMS_TO_TICKS(150));
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
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, DC_Motor_In2);
            operation = DOWN;
            last_task = PD;
            window_state = MIDDLE;
        }

        // Allow other tasks/idle to run
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Idle Task for sleeping the processor */
void vApplicationIdleHook( void ) {
    /* Put the microcontroller in a low power mode */
    SysCtlSleep();
}

/*––– ISR HANDLER –––*/
void ISRHandlers(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (debounce_in_progress) {
        // Skip handling while debouncing
        GPIOIntClear(GPIO_PORTA_BASE, ALL_BUTTONS);
        return;
    }
    debounce_in_progress = true;

    // Start debounce timer for 20ms
    xTimerStartFromISR(xDebounceTimer, &xHigherPriorityTaskWoken);
//    SysCtlDelay(SysCtlClockGet() / 3000 * 200);

    // Only handling Driver_Elevate_Button here:
    if (GPIOIntStatus(GPIO_PORTA_BASE, Driver_Elevate_Button) == Driver_Elevate_Button) {
        GPIOIntClear(GPIO_PORTA_BASE, Driver_Elevate_Button);

        if(last_task == STOP){
            // Wake the driver‑up task
            xSemaphoreGiveFromISR(xDriverUpSem, &xHigherPriorityTaskWoken);
        } else if(last_task == PU || last_task == DU || last_task == DD || last_task == PD){
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
    }
    // Only handling Driver_Lower_Button here:
    else if (GPIOIntStatus(GPIO_PORTA_BASE, Driver_Lower_Button) == Driver_Lower_Button) {
        GPIOIntClear(GPIO_PORTA_BASE, Driver_Lower_Button);

        if(last_task == STOP){
            // Wake the driver‑down task
            xSemaphoreGiveFromISR(xDriverDownSem, &xHigherPriorityTaskWoken);
        } else if(last_task == PU || last_task == DU || last_task == DD || last_task == PD){
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
    }
    // Only handling Passenger_Elevate_Button here:
    else if (GPIOIntStatus(GPIO_PORTA_BASE, Passenger_Elevate_Button) == Passenger_Elevate_Button) {
        GPIOIntClear(GPIO_PORTA_BASE, Passenger_Elevate_Button);

        if (lock_state == LOW && last_task == STOP) {
            // Wake the passenger‑up task
            xSemaphoreGiveFromISR(xPassengerUpSem, &xHigherPriorityTaskWoken);
        } else if (lock_state == LOW && (last_task == PU || last_task == DU || last_task == DD || last_task == PD)) {
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
    }
    // Only handling Passenger_Lower_Button here:
    else if (GPIOIntStatus(GPIO_PORTA_BASE, Passenger_Lower_Button) == Passenger_Lower_Button) {
        GPIOIntClear(GPIO_PORTA_BASE, Passenger_Lower_Button);

        if (lock_state == LOW && last_task == STOP) {
            // Wake the passenger‑down task
            xSemaphoreGiveFromISR(xPassengerDownSem, &xHigherPriorityTaskWoken);
        } else if (lock_state == LOW && (last_task == PU || last_task == DU || last_task == DD || last_task == PD)) {
            GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1|DC_Motor_In2, 0);
            last_task = STOP;
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vDebounceTimerCallback(TimerHandle_t xTimer) {
    // Clear debounce flag
    debounce_in_progress = false;
}
