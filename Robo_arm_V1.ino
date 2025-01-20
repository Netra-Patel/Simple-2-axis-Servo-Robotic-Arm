#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

// Create MPU6050 object
MPU6050 mpu;

// Servo objects for controlling the two servos
Servo servo1; // Servo 1 (connected to D9)
Servo servo2; // Servo 2 (connected to D10)

// MPU6050 control/status variables
bool dmpReady = false;  // Set true if DMP init is successful
uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion variables
Quaternion q;           // [w, x, y, z] quaternion container
VectorFloat gravity;    // [x, y, z] gravity vector
float ypr[3];           // [yaw, pitch, roll] container

// Interrupt detection
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);
    while (!Serial); // Wait for Leonardo (if applicable)

    // Initialize I2C communication
    Wire.begin();

    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();

    // Verify connection
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Initialize DMP
    Serial.println("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // Adjust as needed

    // Check if DMP initialization was successful
    if (devStatus == 0) {
        Serial.println("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set DMP ready flag and get expected packet size
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        Serial.println("DMP ready! Waiting for data...");
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
        while (1); // Halt
    }

    // Attach servos to pins
    servo1.attach(9);  // Servo 1 to pin D9
    servo2.attach(10); // Servo 2 to pin D10
}

void loop() {
    // If DMP is not ready, do nothing
    if (!dmpReady) return;

    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize);

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for FIFO overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
        return;
    }

    // Check for DMP data ready interrupt
    if (mpuIntStatus & 0x02) {
        // Wait for correct available data length
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Get Yaw, Pitch, Roll values
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert pitch and roll to servo angles (map to 0–180 degrees)
        int servo1Angle = map(ypr[1] * 180 / M_PI, -90, 90, 0, 180); // Pitch
        int servo2Angle = map(ypr[0] * 180 / M_PI, -90, 90, 0, 180); // Roll

        // Constrain angles to servo range
        servo1Angle = constrain(servo1Angle, 0, 180);
        servo2Angle = constrain(servo2Angle, 0, 180);

        // Move servos
        servo1.write(servo1Angle);
        servo2.write(servo2Angle);

        // Print debug information
        Serial.print("Pitch: ");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print(" | Yaw: ");
        Serial.println(ypr[0] * 180 / M_PI);
    }
}
