#pragma once
#include <Arduino.h>
#include "config.h"

struct MedianFilter {
    float buffer[MEDIAN_WINDOW];
    int index;
    int count;

    MedianFilter();
    void push(float value);
    float get() const;
};

struct SchmittTrigger {
    float threshLow;
    float threshHigh;
    bool state;

    SchmittTrigger();
    SchmittTrigger(float low, float high);
    bool update(float value);
};