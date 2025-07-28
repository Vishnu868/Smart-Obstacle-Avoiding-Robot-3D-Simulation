#  Smart Obstacle-Avoiding Robot with 3D Simulation

This project focuses on building a **smart autonomous robot** capable of navigating dynamic environments using **ultrasonic sensors, a real-time camera**, and **ADMM-based convex optimization** for intelligent path planning. The robot includes both **autonomous** and **manual** (Bluetooth & voice command) control modes and supports a **3D simulation environment** for virtual testing.

---

##  Project Highlights

-  Autonomous obstacle avoidance using ultrasonic sensors and computer vision
-  Real-time camera-based detection for unknown obstacles
-  Servo-based scanning system for 360° environment sensing
-  Forward and inverse kinematics for accurate path control
-  Convex optimization using ADMM (Alternating Direction Method of Multipliers)
-  Real-time path re-calculation during motion
-  Manual control via Bluetooth and voice commands
-  3D robot simulation with dynamic environment interaction

---

##  Technologies Used

- **Hardware:**
  - Ultrasonic sensors (HC-SR04)
  - Camera module (ESP32-CAM or Pi Cam)
  - Servo motors (for scanning)
  - DC motors (4-wheeled base)
  - Bluetooth module (HC-05)
  - Arduino Uno / NodeMCU / Raspberry Pi

- **Software/Simulators:**
  - Python / C++
  - CoppeliaSim
  - OpenCV (for camera object detection)
  - Bluetooth Serial Communication
  - Speech Recognition (Voice control)

- **Optimization:**
  - ADMM (Alternating Direction Method of Multipliers)
  - Convex optimization libraries: `cvxpy`, `scipy.optimize`

---

##  Core Concepts

###  Real-Time Obstacle Detection
- **Ultrasonic sensors** detect obstacles within range
- **Servo motor** scans the area left-right to build a range map
- **Camera feed** processes visual obstacles (optional object classification)

###  Kinematic Control
- **Forward Kinematics** for estimating position
- **Inverse Kinematics** to determine motor speeds and turning angles

###  Path Planning with ADMM
- ADMM is used for solving convex optimization problems:
  - Minimize trajectory cost
  - Avoid dynamic obstacles
  - Re-plan in real-time when environment changes

###  Voice and Bluetooth Control
- Commands like `forward`, `left`, `stop` via mobile app or microphone
- Uses Android Serial Bluetooth terminal or custom voice app

---

##  3D Simulation Environment

Simulate the robot in:
- `Coppeliasim`
- `Unity3D` or `Unreal Engine` (for advanced 3D visuals)

Simulation includes:
- Dynamic obstacles
- Sensor emulation
- Route optimization

---
## Visualization
![360](Screenshot2025-07-28113228.png)

