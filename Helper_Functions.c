#include "Config.h"
#include "Helper_Functions.h"

int last_task = STOP;
int window_state = MIDDLE;
bool operation;
bool autoMode;
bool objDet;

void stopWindow(void) {
    GPIOPinWrite(GPIO_PORTA_BASE, DC_Motor_In1 | DC_Motor_In2, 0);
    last_task = STOP;
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
