#include "WindowTasks.h"

TaskHandle_t xDriverWindowElevateTaskHandle = NULL, xDriverWindowLowerTaskHandle = NULL;	// handler of the vDriverWindowElevateTask and vDriverWindowLowerTask
TaskHandle_t xPassengerWindowElevateTaskHandle = NULL, xPassengerWindowLowerTaskHandle = NULL; // handler of the vPassengerWindowElevateTask and vPassengerWindowLowerTask

SemaphoreHandle_t xDriverUpSem = NULL, xDriverDownSem = NULL;
SemaphoreHandle_t xPassengerUpSem = NULL, xPassengerDownSem = NULL;

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