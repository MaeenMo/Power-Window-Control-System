#ifndef LCD_H
#define LCD_H

#include "Config.h"
#include "RotaryEncoder.h"

typedef struct {
    uint8_t  window_state;
    bool     lock_state;
    uint32_t percent_open;
} StatusMsg_t; // Structure for status messages

extern QueueHandle_t xStatusQueue; // Queue for sending status messages

void vLCDTask(void *pv); // RTOS task for updating the LCD display
void vStatusProducerTask(void *pv); // RTOS task for producing status messages

#endif // LCD_H
