#include "Safety.h"

TaskHandle_t xLockWindowsTaskHandle = NULL; // handler of the vLockWindowsTask
TaskHandle_t xUpperLimitTaskHandle = NULL, xLowerLimitTaskHandle = NULL; // handler of the vUpperLimitTask
TaskHandle_t xObstacleDetectionHandle = NULL; // handler of the vObstacleDetection

SemaphoreHandle_t xLockWindowsSem = NULL;
SemaphoreHandle_t xUpperLimitSem = NULL, xLowerLimitSem = NULL;
SemaphoreHandle_t xObstacleDetectionSem = NULL;

/*––– LOCK WINDOWS TASK –––*/
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

/*---– UPPER LIMIT TASK –––*/
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

/*––– LOWER LIMIT TASK –––*/
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