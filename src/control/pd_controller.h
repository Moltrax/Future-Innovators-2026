#pragma once
#include <Arduino.h>

struct PDController {
    float kp;
    float kd;
    float prevMeasured;
    bool initialized;

    PDController();
    PDController(float p, float d);
    float compute(float setpoint, float measured, float dt);
    void reset();
};