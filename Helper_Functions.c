#include "Config.h"
#include "Helper_Functions.h"
#include "RotaryEncoder.h"

uint8_t last_task = STOP;
uint8_t window_state = MIDDLE;
uint8_t operation = STOP;
bool passenger_elevate_button_state;
bool passenger_lower_button_state;
bool lock_state;
bool autoMode;
bool objDet;
bool encoder_limit;
uint32_t pos = 0;
uint32_t prev_pos = 0;

void stopWindow(void) {
    GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1 | DC_Motor_In2, 0);
    last_task = STOP;
    operation = STOP;
}

void openWindow(uint8_t window) {
    GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1 | DC_Motor_In2, DC_Motor_In2);
    if (window == DRIVER_WINDOW) {
        last_task = DD;
    } else if (window == PASSENGER_WINDOW) {
        last_task = PD;
    }
    operation = DOWN;
    window_state = MIDDLE;
}

void closeWindow(uint8_t window) {
    GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1 | DC_Motor_In2, DC_Motor_In1);
    if (window == DRIVER_WINDOW) {
        last_task = DU;
    } else if (window == PASSENGER_WINDOW) {
        last_task = PU;
    }
    operation = UP;
    window_state = MIDDLE;
}

void trigger_limit_semaphore(SemaphoreHandle_t xLimitSem, bool* encoder_limit) {
    *encoder_limit = true;

    // Trigger to suspend the task
    xSemaphoreGive(xLimitSem);
    taskYIELD();
}

void handle_position_bounds(uint32_t* pos) {
    *pos = QEIPositionGet(QEI0_BASE);
    if (*pos > TOTAL_PULSES && *pos < TOTAL_PULSES + 20) {
        QEIPositionSet(QEI0_BASE, TOTAL_PULSES);
        *pos = QEIPositionGet(QEI0_BASE);
    } else if (*pos > 450) {
        QEIPositionSet(QEI0_BASE, 0);
        *pos = QEIPositionGet(QEI0_BASE);
    }
}