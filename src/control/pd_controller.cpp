#include "pd_controller.h"

PDController::PDController()
    : kp(0.0f), kd(0.0f), prevMeasured(0.0f), initialized(false) {}

PDController::PDController(float p, float d)
    : kp(p), kd(d), prevMeasured(0.0f), initialized(false) {}

float PDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // Derivative on measurement to avoid derivative kick on setpoint changes
    float derivative = 0.0f;
    if (initialized && dt > 0.0f) {
        derivative = (measured - prevMeasured) / dt;
    }
    prevMeasured = measured;
    initialized = true;

    // Note: subtract derivative (on measurement) instead of adding derivative (on error)
    return kp * error - kd * derivative;
}

void PDController::reset() {
    prevMeasured = 0.0f;
    initialized = false;
}