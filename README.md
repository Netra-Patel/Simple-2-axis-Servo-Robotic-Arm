# Simple-2-axis-Servo-Robotic-Arm



## Overview
This project demonstrates a 2-axis servo robotic arm controlled by an MPU-6050 accelerometer and gyroscope sensor. The arm replicates real-time motion by translating the sensor's orientation data into precise servo movements. It’s a beginner-friendly robotics project designed to showcase the integration of sensors, servos, and embedded programming.

## Features
- Real-time motion sensing using the MPU-6050.
- Control of two servo motors to mimic motion based on sensor input.
- Built using Arduino for simplicity and accessibility.

## Components Used
- **MPU-6050**: Accelerometer and gyroscope sensor.
- **Servo Motors**: Two standard servo motors.
- **Arduino Mega**: Microcontroller for programming and integration.
- **Jumper wires**, breadboard, and other basic hardware.

## How It Works
1. The MPU-6050 captures tilt and orientation data.
2. The Arduino processes the sensor data and calculates the required servo angles.
3. The servo motors adjust their position to replicate the motion detected by the MPU-6050.

## Getting Started
### Prerequisites
- Arduino IDE installed on your computer.
- Basic knowledge of Arduino programming and electronics.

### Software Setup
1. Install the required libraries:
   - [Wire.h](https://www.arduino.cc/reference/en/libraries/wire/)
   - [Servo.h](https://www.arduino.cc/reference/en/libraries/servo/)
   - [I2Cdev.h](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev)
   - [MPU6050_6Axis_MotionApps20.h](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
2. Adjust any servo or MPU-6050 calibration settings in the code if needed.

### Schematic Connections
1. Connections for MPU-6050 Sensor Module
- VCC  -> 3.3V on Arduino Mega 2560
- GND  -> Ground (GND) on Arduino Mega 2560
- SDA  -> D20 (SDA) on Arduino Mega 2560
- SCL  -> D21 (SCL) on Arduino Mega 2560
- INT  -> D2 on Arduino Mega 2560

2. Connections for Micro Servo Motor (Roll)
- Signal (S) (Yellow/Orange) -> D9 on Arduino Mega 2560
- Power (+) (Red)           -> 5V on Arduino Mega 2560
- Ground (-) (Black/Brown)  -> GND on Arduino Mega 2560

3. Connections for Micro Servo Motor (Pitch)
- Signal (S) (Yellow/Orange) -> D10 on Arduino Mega 2560
- Power (+) (Red)            -> 5V on Arduino Mega 2560
- Ground (-) (Black/Brown)   -> GND on Arduino Mega 2560

## Video Demonstration
[Download and watch the video](Video_demo./video.mov)

## License
This project is open-source. Feel free to modify and use it for your own projects!

## Acknowledgments
This project was inspired by [this guide](https://projecthub.arduino.cc/RucksikaaR/simple-2-axis-servo-robotic-arm-controlled-by-mpu-6050-0a31a3). Special thanks to the open-source community for the libraries and resources used.
