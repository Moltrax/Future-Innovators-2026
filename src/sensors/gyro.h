#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "config.h"

struct Gyroscope {
    MPU6050 mpu;
    float heading;          // Fused heading in degrees
    float rawGyroHeading;   // Pure gyro-integrated heading
    int lapCount;
    int cornerCount;
    unsigned long lastUpdateUs;
    float lastCornerHeading; // Heading at last detected corner

    Gyroscope();
    void init();
    void update(float distL, float distR);
    float getYawRate();
    float getHeading() const;
    int getLapCount() const;
    int getCornerCount() const;

private:
    void updateLapCount();
    void updateCornerCount();
};