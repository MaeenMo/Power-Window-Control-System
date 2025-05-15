#ifndef WINDOW_H
#define WINDOW_H

#include "Config.h"
#include "Helper_Functions.h"

extern TaskHandle_t xDriverWindowElevateTaskHandle, xDriverWindowLowerTaskHandle,
                    xPassengerWindowElevateTaskHandle, xPassengerWindowLowerTaskHandle; // handler of the vDriverWindowElevateTask and vDriverWindowLowerTask

extern SemaphoreHandle_t xDriverUpSem, xDriverDownSem, xPassengerUpSem, xPassengerDownSem;

void vDriverWindowElevateTask(void *pvParameters);
void vDriverWindowLowerTask(void *pvParameters);
void vPassengerWindowElevateTask(void *pvParameters);
void vPassengerWindowLowerTask(void *pvParameters);

#endif
