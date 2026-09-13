# ROS 2 Arm Controller: Subsystem Specification & Status Guide

This document details the ROS 2 software architecture, topic interfaces, serial protocol, error handling, and verification status for the robotic arm subsystem in the **Autonomous Conveyor-Based Robotic Sorting Cell** (Review 2).

---

## 1. Subsystem Architecture

```
                   ROS 2 Humble (Ubuntu 22.04)
┌──────────────────────────────────────────────────────────────┐
│  /arm/command (std_msgs/msg/String)                          │
│        │                                                     │
│        ▼                                                     │
│  [arm_controller node]                                       │
│        ├── ArmCommandParser (validation & configurable limits)│
│        ├── ArmControllerCore (orchestration & status)        │
│        └── SerialInterface (pyserial & mock mode)            │
│        │                                                     │
│        ▼                                                     │
│  /arm/status (std_msgs/msg/String)                           │
└──────────────────────────────────────────────────────────────┘
                         │
                         │ USB Serial (115200 baud, 8N1, '\n')
                         ▼
┌──────────────────────────────────────────────────────────────┐
│  [Arduino Uno]                                               │
│        │                                                     │
│        │ I2C (50 Hz PWM)                                     │
│        ▼                                                     │
│  [Adafruit PCA9685 16-Channel 12-Bit PWM Driver]             │
│        │                                                     │
│        ▼                                                     │
│  6× RC Servos (3× MG996R + 3× MG90S)                         │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. ROS 2 Interfaces

### 2.1 Command Topic: `/arm/command`
- **Message Type**: `std_msgs/msg/String`
- **Supported Commands**:
  - `HOME`
  - `BASE <angle>`
  - `SHOULDER <angle>`
  - `ELBOW <angle>`
  - `WRIST <angle>`
  - `GRIPPER OPEN`
  - `GRIPPER CLOSE`
- **Example Usage**:
  ```bash
  ros2 topic pub --once /arm/command std_msgs/msg/String "data: 'BASE 90'"
  ros2 topic pub --once /arm/command std_msgs/msg/String "data: 'GRIPPER OPEN'"
  ros2 topic pub --once /arm/command std_msgs/msg/String "data: 'HOME'"
  ```

### 2.2 Status Topic: `/arm/status`
- **Message Type**: `std_msgs/msg/String`
- **Status Lifecycle & Transitions**:
  1. **Startup**: Emits `IDLE` upon successful serial/mock initialization (or `ERROR:SERIAL_CONNECTION` if connection fails).
  2. **Execution**: Emits `EXECUTING:<ACTION>` immediately when a validated command starts (e.g. `EXECUTING:BASE`, `EXECUTING:HOME`, `EXECUTING:GRIPPER`).
  3. **Success**: Emits `OK:<ACTION>` once Arduino acknowledges execution (e.g. `OK:BASE`, `OK:HOME`, `OK:GRIPPER`).
  4. **Validation Rejection**: Emits `ERROR:INVALID_COMMAND` if syntax, angle limits, or parameters are invalid.
  5. **Communication Faults**: Emits `ERROR:SERIAL_TIMEOUT`, `ERROR:ARDUINO_DISCONNECTED`, or `ERROR:SERIAL_CONNECTION`.

---

## 3. Serial Communication Protocol

- **Baud Rate**: `115200`
- **Line Ending**: Newline `\n` (CR `\r` safely ignored)

| ROS Command | Wire Format (ROS → Arduino) | Expected Arduino Response | Final `/arm/status` |
|---|---|---|---|
| `HOME` | `HOME\n` | `OK:HOME\n` | `OK:HOME` |
| `BASE 90` | `BASE:90\n` | `OK:BASE\n` | `OK:BASE` |
| `SHOULDER 60` | `SHOULDER:60\n` | `OK:SHOULDER\n` | `OK:SHOULDER` |
| `ELBOW 120` | `ELBOW:120\n` | `OK:ELBOW\n` | `OK:ELBOW` |
| `WRIST 90` | `WRIST:90\n` | `OK:WRIST\n` | `OK:WRIST` |
| `GRIPPER OPEN` | `GRIPPER:OPEN\n` | `OK:GRIPPER\n` | `OK:GRIPPER` |
| `GRIPPER CLOSE` | `GRIPPER:CLOSE\n` | `OK:GRIPPER\n` | `OK:GRIPPER` |

---

## 4. Mock Mode

Mock mode allows full functional verification of the ROS 2 node, command parser, status broadcaster, and serial abstraction without physical hardware.

### Launching in Mock Mode
```bash
ros2 run sorting_cell arm_controller --ros-args -p mock_mode:=true
```

### Simulated Behavior
- No real serial port is opened.
- The node generates simulated Arduino responses matching the protocol.
- Example console output:
  ```text
  [INFO] [arm_controller]: Received command: BASE 90
  [INFO] [arm_controller]: MOCK SERIAL -> BASE:90
  [INFO] [arm_controller]: MOCK ARDUINO -> OK:BASE
  ```

---

## 5. Robust Error Handling

The node never crashes, locks up, or throws unhandled exceptions when encountering malformed commands or communication failures.

### Command Validation Errors
| Input | Rejection Reason | Status Published |
|---|---|---|
| `BASE 999` | Angle exceeds max limit | `ERROR:INVALID_COMMAND` |
| `BASE -50` | Angle below min limit | `ERROR:INVALID_COMMAND` |
| `BASE hello` | Non-numeric parameter | `ERROR:INVALID_COMMAND` |
| `BASE` | Missing angle argument | `ERROR:INVALID_COMMAND` |
| `UNKNOWN` | Unrecognized command verb | `ERROR:INVALID_COMMAND` |
| `GRIPPER 123` | Invalid gripper state | `ERROR:INVALID_COMMAND` |
| `HOME extra` | Unexpected extra argument | `ERROR:INVALID_COMMAND` |
| `""` or `"   "` | Empty / whitespace string | `ERROR:INVALID_COMMAND` |

### Serial Communication Errors
| Condition | Trigger | Status Published |
|---|---|---|
| Serial port unavailable | Device missing or pyserial absent | `ERROR:SERIAL_CONNECTION` |
| Connection failure | OS denies access or port busy | `ERROR:SERIAL_CONNECTION` |
| Serial read timeout | Arduino fails to reply within timeout (1.0s) | `ERROR:SERIAL_TIMEOUT` |
| Arduino disconnected | Device unplugged during write or read | `ERROR:ARDUINO_DISCONNECTED` |
| Malformed Arduino reply | Unrecognized bytes returned from Arduino | `ERROR:MALFORMED_RESPONSE:...` |
| Arduino error response | Arduino reports invalid command | `ERROR:INVALID_COMMAND` |

---

## 6. Verification Status Matrix

To maintain rigorous project engineering integrity, testing status is explicitly categorized:

| Component / Feature | Environment | Status | Details |
|---|---|---|---|
| Command Parsing & Limits Validation | Windows (Python 3.13) | **VERIFIED** | 35 pytest unit tests passing covering syntax, limits, boundaries |
| Wire Protocol & Newline Termination | Windows (Python 3.13) | **VERIFIED** | 12 pytest unit tests verifying `\n` termination |
| Mock Serial Mode & Lifecycle | Windows (Python 3.13) | **VERIFIED** | 15 pytest unit tests verifying simulation, connect/disconnect |
| Serial Error Handling (Mocks) | Windows (Python 3.13) | **VERIFIED** | 11 pytest unit tests for timeouts, disconnects, write/read errors |
| Full Controller Status Flow | Windows (Python 3.13) | **VERIFIED** | 23 pytest integration tests for status transitions and error flows |
| ROS 2 Humble Package Build | Ubuntu 22.04 LTS | **PENDING** | Requires `colcon build --packages-select sorting_cell` on Ubuntu |
| Live ROS 2 Topic Pub/Sub | Ubuntu 22.04 LTS | **PENDING** | Requires running node with live DDS middleware |
| Arduino Uno Firmware Flashing | Arduino IDE / CLI | **PENDING** | Requires physical Arduino Uno via USB |
| PCA9685 I2C Communication | Physical Hardware | **PENDING** | Requires physical PCA9685 wired to Arduino A4/A5 |
| External 12V Buck Power Supply | Physical Hardware | **PENDING** | Requires high-current buck converter powering PCA9685 V+ |
| Servo Mechanical Calibration | Physical Arm Hardware | **PENDING** | Hardware team must calibrate physical joint limits (1 → 2 → 3 → 6 servos) |
