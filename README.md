# Autonomous Conveyor-Based Robotic Sorting Cell

Welcome to the **Autonomous Conveyor-Based Robotic Sorting Cell** project. This repository contains the ROS 2 software packages, low-level microcontroller firmware, hardware documentation, and test suites for the college capstone engineering project.

---

## Current Status: Review 2 — Robotic Arm Subsystem

Review 2 focuses on establishing the communication and control pipeline for the 6-DOF robotic arm subsystem:
```
Ubuntu 22.04 (ROS 2 Humble)
       │
       │ /arm/command & /arm/status (std_msgs/msg/String)
       ▼
[sorting_cell / arm_controller node]
       │
       │ USB Serial (115200 baud, newline-terminated ASCII)
       ▼
[Arduino Uno arm_controller.ino]
       │
       │ I2C (50 Hz PWM)
       ▼
[PCA9685 16-Channel 12-Bit PWM Driver]
       │
       ▼
6× RC Servos (3× MG996R + 3× MG90S)
```

---

## Repository Structure

```
autonomous-conveyor-sorting-cell/
├── README.md
├── .gitignore
├── ros2_ws/
│   └── src/
│       └── sorting_cell/             # ROS 2 Python package (ament_python)
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           ├── resource/
│           ├── sorting_cell/
│           │   ├── __init__.py
│           │   ├── arm_controller.py  # ROS 2 node & status management
│           │   └── serial_interface.py # Hardware serial & mock mode
│           └── test/
│               └── test_arm_controller.py # 96 automated unit/integration tests
├── arduino/
│   └── arm_controller/
│       └── arm_controller.ino        # Arduino Uno firmware for PCA9685 control
├── hardware/                         # Hardware schematics and mechanical specs
└── docs/
    ├── ros2_arm_controller.md        # ROS 2 topic specs, error matrix, verification guide
    └── arduino_pca9685_subsystem.md  # Electrical wiring, power isolation, bring-up checklist
```

---

## Documentation Quick Links

- [ROS 2 Arm Controller Specification](docs/ros2_arm_controller.md) (Topics, Statuses, Mock Mode, Testing Matrix)
- [Arduino & PCA9685 Hardware Guide](docs/arduino_pca9685_subsystem.md) (Electrical Safety, Wiring, Channel Map, Bring-Up)

---

## Running Unit & Integration Tests

The test suite runs on any machine with Python 3.8+ and pytest:

```bash
pytest ros2_ws/src/sorting_cell/test/test_arm_controller.py -v
```

All 96 tests pass without requiring ROS 2, an Arduino, or physical servos connected.