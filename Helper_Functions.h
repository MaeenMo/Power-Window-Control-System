#ifndef POWER_WINDOW_CONTROL_SYSTEM_HELPER_FUNCTIONS_H
#define POWER_WINDOW_CONTROL_SYSTEM_HELPER_FUNCTIONS_H

extern uint8_t last_task; // variable for handling the last task performed
extern uint8_t window_state; // variable for handling the state of the window state
extern uint8_t operation; // variable for handling the return from the manual control
extern bool autoMode; // variable for handling the state of auto mode
extern bool objDet; // variable for handling the state of object detection
extern bool encoder_limit;
extern uint32_t pos;
extern uint32_t prev_pos;

void stopWindow(void);

void openWindow(uint8_t window);

void closeWindow(uint8_t window);


#endif //POWER_WINDOW_CONTROL_SYSTEM_HELPER_FUNCTIONS_H
