# Power Window Control System

A real-time embedded system for controlling automotive power windows using the **TM4C123GH6PM** microcontroller and **FreeRTOS**.

---

## 🚗 Features

- **Dual Control:** Driver and passenger can operate the window independently.
- **Manual and Auto Modes:**
    - *Manual:* Window moves while the button is held.
    - *Auto:* One-touch open/close.
- **Safety Mechanisms:**
    - *Anti-pinch protection:* Automatically reverses window if obstacle is detected.
    - *Window lock:* Driver can disable passenger controls.
    - *Limit switch detection:* Prevents over-travel at top/bottom ends.
- **Encoder-Based Tracking:**
    - Real-time position tracking via rotary encoder.
    - Accurate calculation of open/close percentage.
- **User Feedback:**
    - LCD shows window state, lock status, and position percentage.
- **Debounced Inputs:** All switches are debounced using timers to ensure reliable signal handling.

---

## 🧰 Hardware Components

| Component                                         | Description                     |
|---------------------------------------------------|---------------------------------|
| TM4C123GH6PM MCU                                  | Main controller                 |
| DC Motor + H-Bridge Driver                        | Controls window movement        |
| Rotary Encoder (QEI)                              | Tracks window position          |
| LCD (I2C)                                         | Displays system status          |
| Buttons (x4)                                      | Driver/Passenger Up & Down      |
| Limit Switches (x2)                               | Upper and lower travel limits   |
| Lock Switch                                       | Enables/disables passenger side |
| IR Obstacle Sensor (Simulated with a push-button) | Detects object in window path   |

---

## 📍 Pin Configuration

### **Port A**
| Pin | Function                    |
|-----|-----------------------------|
| PA2 | Passenger Up Button         |
| PA3 | Passenger Down Button       |
| PA4 | Driver Up Button            |
| PA5 | Driver Down Button          |
| PA6 | Motor IN1 (transistor base) |
| PA7 | Motor IN2 (transistor base) |

### **Port B**
| Pin | Function   |
|-----|------------|
| PB2 | I2C SCL    |
| PB3 | I2C SDA    |

### **Port C**
| Pin | Function                |
|-----|-------------------------|
| PC4 | Object Detection Sensor |
| PC5 | Upper Limit Switch      |
| PC6 | Lower Limit Switch      |
| PC7 | Window Lock Switch      |

### **Port D**
| Pin | Function      |
|-----|---------------|
| PD6 | QEI Phase A   |
| PD7 | QEI Phase B   |

---

## 🧠 Software Architecture

### 🔧 Modules

- **Window Control Module:** Handles manual and auto control from driver and passenger sides.
- **Sensor & Safety Module:** Processes limit switches, lock switch, and obstacle detection.
- **Encoder Module:** Computes window position using rotary encoder and maps it to a 0–100% range.
- **LCD Module:** Displays real-time status (window state, lock icon, percentage open).
- **Configuration Module:** Initializes ports, system clock, and defines macros and pin assignments.

### 🧵 Tasks (FreeRTOS)

- `vDriverWindowElevateTask`
- `vDriverWindowLowerTask`
- `vPassengerWindowElevateTask`
- `vPassengerWindowLowerTask`
- `vLockWindowsTask`
- `vUpperLimitTask`
- `vLowerLimitTask`
- `vObstacleDetection`
- `vEncoderMonitorTask`
- `vStatusProducerTask`
- `vLCDTask`

---

## 🛡️ Safety Features

| Feature             | Description                                                                 |
|---------------------|-----------------------------------------------------------------------------|
| **Anti-Pinch**      | IR sensor detects obstacles and reverses motor to prevent injury            |
| **Window Lock**     | Driver can lock/unlock passenger control                                     |
| **Limit Protection**| Stops motor at top and bottom window positions                              |
| **Auto Mode Cancel**| Any button press cancels auto movement immediately                          |
| **Encoder Clamping**| Window position is clamped between 0–100% to prevent invalid states         |

---

## ⚙️ Build & Flash Instructions

1. Open the project in **Keil uVision5**
2. Ensure **TivaWare** and **FreeRTOS** are properly linked.
3. Connect TM4C123GH6PM via **USB Debug Interface**.
4. Build the project (`Rebuild all`).
5. Flash to board (`Load` or `Ctrl + F8`).

---

## 📦 Dependencies

- [TivaWare Peripheral Driver Library](https://www.ti.com/tool/SW-TM4C)
- [FreeRTOS Kernel](https://freertos.org/)
- Keil MDK ARM IDE (or compatible)

---

## ✅ Future Improvements

- Add QEI direction filtering to discard noise.
- Add velocity monitoring for smoother motion.
- Enable EEPROM/Flash storage to retain calibration values.
