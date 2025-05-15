#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

extern uint8_t last_task;                     // variable for handling the last task performed
extern uint8_t window_state;                  // variable for handling the state of the window state
extern uint8_t operation;                     // variable for handling the return from the manual control
extern bool passenger_elevate_button_state;   // variable for handling the state of the elevator button from the Passenger side
extern bool passenger_lower_button_state;     // variable for handling the state of the Lowering button from the Passenger side
extern bool lock_state;                       // variable for handling the state of the lock switch
extern bool autoMode;                         // variable for handling the state of auto mode
extern bool objDet;                           // variable for handling the state of object detection
extern bool encoder_limit;                    // variable for handling the state of the encoder limit
extern uint32_t pos;                          // variable for handling the position of the window
extern uint32_t prev_pos;                     // variable for handling the previous position of the window

void stopWindow(void); // Function to stop the window

void openWindow(uint8_t window); // Function to open the window

void closeWindow(uint8_t window); // Function to close the window

void trigger_limit_semaphore(SemaphoreHandle_t xLimitSem, bool* is_encoder_limit); // Function to trigger the semaphore for the limit switch

void handle_position_bounds(uint32_t* pos); // Function to handle the position bounds of the window

#endif // HELPER_FUNCTIONS_H
