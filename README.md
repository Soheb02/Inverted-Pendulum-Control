# Inverted-Pendulum-Control
Version Control for Inverted Pendulum Controller Project (MEC830)

# Inverted Pendulum Stabilization Project
This repository contains the code and design files for an inverted pendulum stabilization project, completed as part of a systems design course. The objective is to stabilize a pendulum mounted on a cart using sensors and motor control, implementing closed-loop feedback to maintain balance.

# Project Overview
In this project, we designed and developed a real-life inverted pendulum system controlled by an Arduino Uno. The pendulum is stabilized through motor-driven corrections, guided by feedback from an IMU sensor and, potentially, infrared sensors to track cart position. The system can operate in either open-loop or closed-loop control configurations, depending on the hardware and control requirements.

# Hardware Components
Arduino Uno: Main microcontroller for handling sensors and controlling motors.
Brushed DC or Brushless DC Motors: Provides the necessary speed and torque to control the cart's position and stabilize the pendulum.
Motor Driver/ESC: Drives the motors and receives PWM signals from the Arduino Uno.
IMU Sensor (e.g., MPU6050): Measures the angle of the pendulum to provide real-time orientation feedback.
Infrared Sensors (Optional): Tracks the position of the cart on the platform.
Power Supply: 3V or 9V batteries to power the motors and components.
Project Design
The system is designed to handle small disturbances by adjusting the position of the cart. The Arduino receives data from the IMU sensor, which measures the pendulum’s tilt angle. Based on this data, the Arduino sends PWM signals to the motors to correct the pendulum’s tilt.

# Control Modes
Closed-Loop Control:
Uses feedback from an encoder attached to the motors (if available) to monitor motor speed or position, adjusting the PWM signal to precisely control the motor's output.
Improves stability and responsiveness in the system.

# Design Constraints
Motor Control: The motor cannot be directly connected to the pendulum pivot. The design allows for motors placed near the base, applying forces indirectly to stabilize the pendulum.
Budget Constraints: The project is budget-limited, requiring cost-effective components and motor options.
Microcontroller: Arduino Uno is the required microcontroller for all control tasks.

# Repository Structure
/code: Arduino code files for both open-loop and closed-loop control, including IMU integration and PID algorithms.
/designs: CAD files for mechanical parts, 3D-printed components, and linkage setup.
README.md: Project overview, setup instructions, and explanations of the control strategies.
/docs: Documentation files, including wiring diagrams and system schematics.

# Getting Started
Prerequisites
Arduino IDE: Download and install the Arduino IDE.
MPU6050 Library: Install the MPU6050 library via Arduino’s Library Manager.
Motor Driver Library: Install any necessary libraries for your specific motor driver or ESC.
Hardware Setup
Connect the IMU Sensor: Wire the IMU to the Arduino’s SDA and SCL pins.
Motor Wiring: 
Connect the motors to the motor driver or ESC.
Ensure the motor driver or ESC has a suitable power source.
Encoder Setup (for Closed-Loop Mode): Attach encoders to the motors if using closed-loop control and connect them to digital pins on the Arduino.

# Code Overview
Closed-Loop Control:
Implements a PID algorithm to adjust motor speed based on feedback from both the IMU and encoders (if available).
Code file: code/closed_loop_control.ino

# Uploading Code
Connect the Arduino Uno to your computer via USB.
Open the appropriate .ino file in the Arduino IDE.
Select Tools > Board > Arduino Uno and choose the correct Port.
Click Upload to transfer the code to the Arduino.
Usage
Stabilize the Pendulum: After setup, the system should stabilize the pendulum based on IMU (and encoder, if applicable) data.
Adjusting Parameters: Modify PID gains in the code to fine-tune the response to disturbances.

# Troubleshooting
Oscillation Issues: Adjust PID values to minimize oscillations.
Motor Speed: Ensure the power supply is adequate for the motor requirements. If speed control is unresponsive, check PWM connections.
Sensor Drift: Use filtering (e.g., complementary or Kalman filter) to reduce IMU drift over time.

# Version Control
This repository is managed using GitHub for version control. Branches are used for different features (e.g., feature/closed-loop-control) and experimental setups.

# Authors and Acknowledgments
Project created by Soheb S.and Isa H., as part of the Systems Design course.
