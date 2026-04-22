#pragma once
#include <Arduino.h>
#include "config.h"

// Single rear DC motor driven via H-Bridge
// DIR_A HIGH + DIR_B LOW  = forward
// DIR_A LOW  + DIR_B HIGH = reverse
// Both LOW                = coast
// PWM pin controls speed

struct Motor {
    int currentPWM;

    Motor();
    void init();
    void setMotorPWM(int targetPWM);
    void rampTo(int targetPWM);
    void stopMotor();

private:
    void applyPWM(int pwm);
    int applyDeadzone(int pwm);
};