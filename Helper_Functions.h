#ifndef POWER_WINDOW_CONTROL_SYSTEM_HELPER_FUNCTIONS_H
#define POWER_WINDOW_CONTROL_SYSTEM_HELPER_FUNCTIONS_H

extern int last_task; // variable for handling the last task performed
extern int window_state; // variable for handling the state of the window state
extern bool operation; // variable for handling the return from the manual control
extern uint8_t state; // variable for handling the state of auto mode

void stopWindow(void);

void openWindow(uint8_t window);

void closeWindow(uint8_t window);


#endif //POWER_WINDOW_CONTROL_SYSTEM_HELPER_FUNCTIONS_H
