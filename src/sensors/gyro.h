#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "config.h"

// GY-521 breakout board carrying the MPU-6050 IMU
// Connected via I2C on SDA_PIN / SCL_PIN
// Only the Z-axis gyroscope is used for yaw tracking
// Accelerometer data is not used — wall-relative angle comes from ultrasonics

struct Gyroscope {

    // ── Public state readable by the control loop ──
    float heading;              // Fused absolute heading in degrees (accumulates across laps)
    float yawRateDps;           // Current yaw rate in degrees per second
    int   lapCount;             // Completed full laps
    int   cornerCount;          // Detected ~90 degree corners

    Gyroscope();

    // Call once in setup(), blocks for ~3 seconds while vehicle is stationary
    void init();

    // Call every sensor cycle with current ultrasonic distances
    void update(float distL, float distR);

    // Read-only accessors
    float getHeading()     const;
    float getYawRate()     const;
    int   getLapCount()    const;
    int   getCornerCount() const;

private:
    MPU6050 mpu;
    unsigned long lastUpdateUs;
    float lastCornerHeading;

    void updateLapCount();
    void updateCornerCount();
};