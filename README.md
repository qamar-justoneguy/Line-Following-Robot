# **Autonomous and Manually Controlled Robot**

## **Project Overview**
This project implements an autonomous and manually controlled robot equipped with multiple sensors to achieve navigation, obstacle detection, and remote control functionalities. The robot integrates IR sensors for line following, an ultrasonic sensor for obstacle detection, and a Bluetooth module for manual control. Additional features include obstacle circumnavigation and real-time data display on the serial monitor.

## **Features**
- **Line Following**: Uses an IR sensor to detect and follow a predefined path.
- **Obstacle Detection**: Employs an ultrasonic sensor to identify obstacles and respond accordingly.
- **Bluetooth Control**: Allows remote control via a Bluetooth module for manual operation.
- **Obstacle Circumnavigation**: Enables the robot to navigate around detected obstacles.
- **Serial Monitor Data Display**: Outputs relevant data, including:
  - Obstacle count
  - Distance measured by the ultrasonic sensor
  - Number of times a perpendicular line is detected

## **Hardware Requirements**
- Microcontroller (e.g., Arduino Uno)
- IR Sensor (for line following)
- Ultrasonic Sensor (for obstacle detection)
- Bluetooth Module (e.g., HC-05/HC-06)
- Motor Driver Module
- Motors and Wheels
- Power Supply (Battery)

## **Software Requirements**
- Arduino IDE
- Serial Monitor (for debugging and data display)
- Bluetooth Control App (for manual control)

