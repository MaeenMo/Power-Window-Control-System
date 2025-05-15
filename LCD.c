#include "LCD.h"

static const uint8_t lockChar[8] = {
        0x0E, 0x11, 0x11, 0x11,
        0x1F, 0x1B, 0x1B, 0x1F
};
static const uint8_t unlockChar[8] = {
        0x0E, 0x11, 0x10, 0x10,
        0x1F, 0x1B, 0x1B, 0x1F
};

QueueHandle_t xStatusQueue;

//—– LCD updater task —–//
void vLCDTask(void *pv) {
    LCD_I2C_CreateChar(0, lockChar);
    LCD_I2C_CreateChar(1, unlockChar);

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