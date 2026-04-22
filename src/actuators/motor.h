#pragma once
#include <Arduino.h>
#include "config.h"

struct Motor {
    int currentPWM;

    Motor();
    void init();
    void setMotorPWM(int targetPWM);
    void rampTo(int targetPWM);
    void stopMotors();

private:
    void applyPWM(int pwm);
    int applyDeadzone(int pwm);
};