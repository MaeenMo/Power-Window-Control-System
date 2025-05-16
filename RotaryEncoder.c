#include "RotaryEncoder.h"

volatile uint32_t TOTAL_PULSES = 150UL;   // pulses from top→bottom
volatile uint32_t windowPct      = 0;        // 0–100%


/*––– ENCODER MONITOR –––*/
void vEncoderMonitorTask(void *pv) {
    QEIPositionSet(QEI0_BASE, 0);
    while (1) {
        int8_t dir = QEIDirectionGet(QEI0_BASE);  // +1 = CW, -1 = CCW

        if (operation == UP && dir > 0) {
            // Closing: percentage increases
            uint32_t speed = QEIVelocityGet(QEI0_BASE);
            if (speed < 16) {
                QEIPositionSet(QEI0_BASE, prev_pos); // ignore as noise
                continue;
            }
            handle_position_bounds(&pos);
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
            uint32_t speed = QEIVelocityGet(QEI0_BASE);
            if (speed < 16) {
                QEIPositionSet(QEI0_BASE, prev_pos); // ignore as noise
                continue;
            }
            handle_position_bounds(&pos);
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