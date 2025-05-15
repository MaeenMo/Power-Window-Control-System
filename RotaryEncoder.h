#ifndef ENCODER_H
#define ENCODER_H

#include "Config.h"
#include "Helper_Functions.h"
#include "Safety.h"

extern volatile uint32_t windowPct; // percentage of the window position
extern volatile uint32_t TOTAL_PULSES; // total pulses from top to bottom

void vEncoderMonitorTask(void *pv); // RTOS task for monitoring the position of the window

#endif