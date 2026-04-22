#include "gyro.h"
#include <math.h>

Gyroscope::Gyroscope()
    : heading(0.0f),
      yawRateDps(0.0f),
      lapCount(0),
      cornerCount(0),
      mpu(Wire),
      lastUpdateUs(0),
      lastCornerHeading(0.0f) {}

void Gyroscope::init() {
    // MPU6050_light talks to the MPU-6050 on the GY-521 board over I2C
    // Default I2C address is 0x68 (AD0 pin low on GY-521)
    byte status = mpu.begin();

    if (status != 0) {
        Serial.printf("[GYRO] MPU-6050 init FAILED (status %d). Check GY-521 wiring.\n", status);
        Serial.printf("[GYRO]   SDA = GPIO %d\n", SDA_PIN);
        Serial.printf("[GYRO]   SCL = GPIO %d\n", SCL_PIN);
        Serial.printf("[GYRO]   Expected I2C address: 0x68\n");
        // Continue anyway — the vehicle can still drive with degraded heading
    } else {
        Serial.printf("[GYRO] MPU-6050 found on GY-521 board\n");
    }

    // Calibrate gyro offsets — vehicle MUST be stationary
    // The library samples the gyro for a few seconds and averages the readings
    // to find the zero-rate offset for each axis
    Serial.printf("[GYRO] Calibrating gyro offsets... keep vehicle still\n");
    mpu.calcOffsets(false, true);  // gyro only, not accelerometer
    Serial.printf("[GYRO] Calibration complete\n");

    heading = 0.0f;
    yawRateDps = 0.0f;
    lapCount = 0;
    cornerCount = 0;
    lastCornerHeading = 0.0f;
    lastUpdateUs = micros();
}

void Gyroscope::update(float distL, float distR) {
    // Let the library read fresh data from the MPU-6050 registers
    mpu.update();

    // Calculate time delta since last update
    unsigned long nowUs = micros();
    float dt = (float)(nowUs - lastUpdateUs) * 1e-6f;
    lastUpdateUs = nowUs;

    // Guard against huge dt after long pauses (startup, debugging)
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.025f; // Fall back to expected sensor period
    }

    // ── Read yaw rate from the Z-axis gyroscope ──
    // MPU-6050 Z-axis points up when the GY-521 board is mounted flat
    // Positive = counter-clockwise when viewed from above
    // getGyroZ() returns degrees per second, already offset-corrected by the library
    yawRateDps = mpu.getGyroZ();

    // Dead-zone: ignore tiny rates that are just residual noise after calibration
    if (fabsf(yawRateDps) < GYRO_DRIFT_THRESHOLD) {
        yawRateDps = 0.0f;
    }

    // ── Integrate gyro to get heading change ──
    float gyroHeading = heading + yawRateDps * dt;

    // ── Compute wall-relative angle from ultrasonic sensors ──
    // If both readings are valid, the difference in distances tells us
    // how much the vehicle is angled relative to the walls
    // This has no long-term drift but is noisy cycle-to-cycle
    float usAngle = 0.0f;
    bool usValid = (distL > 2.0f && distL < (float)US_MAX_CM &&
                    distR > 2.0f && distR < (float)US_MAX_CM);

    if (usValid) {
        // Positive usAngle = vehicle nose pointed toward left wall
        usAngle = atan2f(distL - distR, SENSOR_SPACING_CM) * (180.0f / (float)M_PI);
    }

    // ── Complementary filter ──
    // Fuse gyro (good short-term, drifts long-term) with
    // ultrasonic angle (good long-term, noisy short-term)
    //
    // heading tracks total accumulated rotation (can exceed 360)
    // usAngle is a small local correction (-30 to +30 degrees typically)
    //
    // We apply the US correction as a nudge toward the gyro-integrated heading
    // rather than replacing the absolute heading, because usAngle is local
    // (relative to current wall segment) not absolute

    if (usValid) {
        // The US angle represents the local offset from wall-parallel
        // We use it to correct drift in the gyro integration
        // correctedHeading nudges gyroHeading toward agreeing with usAngle locally
        float localGyroAngle = fmodf(gyroHeading, 90.0f); // Local angle within current straight/corner
        float correction = usAngle - localGyroAngle;

        // Apply correction weighted by (1 - alpha)
        heading = gyroHeading + (1.0f - COMP_FILTER_ALPHA) * correction;
    } else {
        // No valid US data — pure gyro integration (will drift slowly)
        heading = gyroHeading;
    }

    // ── Update lap and corner counters ──
    updateLapCount();
    updateCornerCount();
}

float Gyroscope::getHeading() const {
    return heading;
}

float Gyroscope::getYawRate() const {
    return yawRateDps;
}

int Gyroscope::getLapCount() const {
    return lapCount;
}

int Gyroscope::getCornerCount() const {
    return cornerCount;
}

void Gyroscope::updateLapCount() {
    // Each full lap is 360 degrees of accumulated heading
    // CW (DRIVING_DIRECTION=1): heading increases, lap at +360, +720, +1080
    // CCW (DRIVING_DIRECTION=-1): heading decreases, lap at -360, -720, -1080

    float nextLapThreshold = (float)(lapCount + 1) * 360.0f * (float)DRIVING_DIRECTION;

    if (DRIVING_DIRECTION > 0) {
        if (heading >= nextLapThreshold) {
            lapCount++;
            Serial.printf("[GYRO] Lap %d completed (heading=%.1f)\n", lapCount, heading);
        }
    } else {
        if (heading <= nextLapThreshold) {
            lapCount++;
            Serial.printf("[GYRO] Lap %d completed (heading=%.1f)\n", lapCount, heading);
        }
    }
}

void Gyroscope::updateCornerCount() {
    // A corner is detected when the heading changes by ~90 degrees
    // from the heading at the last detected corner
    // Threshold is set below 90 (CORNER_HEADING_DEG=80) to tolerate
    // imperfect turns while being high enough to ignore PD corrections

    float diff = fabsf(heading - lastCornerHeading);
    if (diff >= CORNER_HEADING_DEG) {
        cornerCount++;
        lastCornerHeading = heading;
        Serial.printf("[GYRO] Corner %d detected (heading=%.1f)\n", cornerCount, heading);
    }
}