# Simple-2-axis-Servo-Robotic-Arm


 Simple 2-Axis Servo Robotic Arm

## Overview
This project demonstrates a 2-axis servo robotic arm controlled by an MPU-6050 accelerometer and gyroscope sensor. The arm replicates real-time motion by translating the sensor's orientation data into precise servo movements. It’s a beginner-friendly robotics project designed to showcase the integration of sensors, servos, and embedded programming.

## Features
- Real-time motion sensing using the MPU-6050.
- Control of two servo motors to mimic motion based on sensor input.
- Built using Arduino for simplicity and accessibility.

## Components Used
- **MPU-6050**: Accelerometer and gyroscope sensor.
- **Servo Motors**: Two standard servo motors.
- **Arduino Uno**: Microcontroller for programming and integration.
- **Jumper wires**, breadboard, and other basic hardware.

## How It Works
1. The MPU-6050 captures tilt and orientation data.
2. The Arduino processes the sensor data and calculates the required servo angles.
3. The servo motors adjust their position to replicate the motion detected by the MPU-6050.

## Getting Started
### Prerequisites
- Arduino IDE installed on your computer.
- Basic knowledge of Arduino programming and electronics.

### Assembly
1. Connect the MPU-6050 to the Arduino Uno (refer to the circuit diagram in the repository).
2. Wire the servo motors to the Arduino.
3. Upload the code from the `src/` folder to the Arduino.

### Software Setup
1. Install the required libraries:
   - [Wire.h](https://www.arduino.cc/reference/en/libraries/wire/)
   - [Servo.h](https://www.arduino.cc/reference/en/libraries/servo/)
2. Adjust any servo or MPU-6050 calibration settings in the code if needed.

## Repository Contents
- **`src/`**: Arduino code for the robotic arm.
- **`docs/`**: Circuit diagrams and additional resources.
- **`media/`**: Videos and images of the robotic arm in action.

## Video Demonstration
Check out the robotic arm in action: [YouTube Link or Video URL]

## License
This project is open-source and available under the [MIT License](LICENSE). Feel free to modify and use it for your own projects!

## Acknowledgments
This project was inspired by [this guide](https://projecthub.arduino.cc/RucksikaaR/simple-2-axis-servo-robotic-arm-controlled-by-mpu-6050-0a31a3). Special thanks to the open-source community for the libraries and resources used.
