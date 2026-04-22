#pragma once
#include <Arduino.h>
#include "config.h"
#include "utils/filters.h"

struct UltrasonicSensor {
    int trigPin;
    int echoPin;
    MedianFilter filter;

    UltrasonicSensor();
    void init(int trig, int echo);
    float measureRawCm();
    float readFilteredCm();
};