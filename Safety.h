#ifndef SAFETY_H
#define SAFETY_H

#include "Config.h"
#include "Helper_Functions.h"
#include "WindowTasks.h"
#include "RotaryEncoder.h"

// Task handles
extern TaskHandle_t xLockWindowsTaskHandle, xUpperLimitTaskHandle, xLowerLimitTaskHandle, xObstacleDetectionHandle;

// Semaphore handles
extern SemaphoreHandle_t xLockWindowsSem, xUpperLimitSem, xLowerLimitSem, xObstacleDetectionSem;

void vLockWindowsTask(void *pvParameters);   // RTOS task for locking/unlocking the control from the passenger side
void vUpperLimitTask(void *pvParameters);    // RTOS task for reacting to reaching the upper limit
void vLowerLimitTask(void *pvParameters);    // RTOS task for reacting to reaching the lower limit
void vObstacleDetection(void *pvParameters); // RTOS task for detecting obstacles in the path of the window during closing

#endif
