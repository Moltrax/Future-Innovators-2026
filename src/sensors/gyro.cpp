#include "gyro.h"
#include <math.h>

Gyroscope::Gyroscope()
    : mpu(Wire),
      heading(0.0f),
      rawGyroHeading(0.0f),
      lapCount(0),
      cornerCount(0),
      lastUpdateUs(0),
      lastCornerHeading(0.0f) {}

void Gyroscope::init() {
    mpu.begin();
    Serial.printf("[GYRO] Calibrating offsets... keep vehicle still\n");
    mpu.calcGyroOffsets();
    Serial.printf("[GYRO] Calibration done\n");
    lastUpdateUs = micros();
    heading = 0.0f;
    rawGyroHeading = 0.0f;
    lapCount = 0;
    cornerCount = 0;
    lastCornerHeading = 0.0f;
}

void Gyroscope::update(float distL, float distR) {
    mpu.update();

    unsigned long nowUs = micros();
    float dt = (float)(nowUs - lastUpdateUs) * 1e-6f;
    lastUpdateUs = nowUs;

    // Clamp dt to avoid huge jumps
    if (dt > 0.1f) dt = 0.1f;
    if (dt <= 0.0f) return;

    float gyroRate = mpu.getAngleZ() - rawGyroHeading;
    // Actually use the library's integrated Z, but we want rate
    // MPU6050_light getGyroZ() gives deg/s
    float gyroZ = mpu.getGyroZ(); // deg/s

    // Integrate gyro for raw heading
    rawGyroHeading += gyroZ * dt;

    // Complementary filter: fuse gyro with ultrasonic-based angle estimate
    // usAngle estimates vehicle yaw offset from wall-parallel
    float usAngle = 0.0f;
    if (distL > 2.0f && distL < (float)US_MAX_CM &&
        distR > 2.0f && distR < (float)US_MAX_CM) {
        usAngle = atan2f(distL - distR, SENSOR_SPACING_CM) * (180.0f / M_PI);
    }

    // Complementary filter fuses gyro integration with US-based local angle
    // heading tracks absolute rotation; usAngle is local wall-relative offset
    // We apply correction only to the local offset portion
    float gyroHeading = heading + gyroZ * dt;
    // usAngle is a local correction, not absolute heading
    // Apply complementary filter as offset correction
    heading = COMP_FILTER_ALPHA * gyroHeading + (1.0f - COMP_FILTER_ALPHA) * (gyroHeading + usAngle - heading);

    // Simplified: heading mostly follows gyro, nudged by US
    heading = COMP_FILTER_ALPHA * (heading + gyroZ * dt) + (1.0f - COMP_FILTER_ALPHA) * (heading + usAngle * dt);

    updateLapCount();
    updateCornerCount();
}

float Gyroscope::getYawRate() {
    return mpu.getGyroZ();
}

float Gyroscope::getHeading() const {
    return heading;
}

int Gyroscope::getLapCount() const {
    return lapCount;
}

int Gyroscope::getCornerCount() const {
    return cornerCount;
}

void Gyroscope::updateLapCount() {
    // CW: heading accumulates positively, CCW: negatively
    float threshold = 360.0f * (float)DRIVING_DIRECTION;
    float accumulated = heading;

    // Check if we crossed a full lap boundary
    if (DRIVING_DIRECTION > 0 && accumulated >= (lapCount + 1) * 360.0f) {
        lapCount++;
    } else if (DRIVING_DIRECTION < 0 && accumulated <= (lapCount + 1) * -360.0f) {
        lapCount++;
    }
}

void Gyroscope::updateCornerCount() {
    // Detect ~90 deg heading changes from last corner
    float diff = fabsf(heading - lastCornerHeading);
    if (diff >= CORNER_HEADING_DEG) {
        cornerCount++;
        lastCornerHeading = heading;
    }
}