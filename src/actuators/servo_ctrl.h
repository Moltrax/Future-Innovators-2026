#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

struct ServoCtrl {
    Servo servo;
    float currentAngle;

    ServoCtrl();
    void init();
    void setServoAngle(float target);
    void center();
};