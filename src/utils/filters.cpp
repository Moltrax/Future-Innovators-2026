#include "filters.h"
#include <algorithm>

MedianFilter::MedianFilter()
    : index(0), count(0) {
    for (int i = 0; i < MEDIAN_WINDOW; i++) {
        buffer[i] = 0.0f;
    }
}

void MedianFilter::push(float value) {
    buffer[index] = value;
    index = (index + 1) % MEDIAN_WINDOW;
    if (count < MEDIAN_WINDOW) {
        count++;
    }
}

float MedianFilter::get() const {
    if (count == 0) return 0.0f;

    float sorted[MEDIAN_WINDOW];
    for (int i = 0; i < count; i++) {
        sorted[i] = buffer[i];
    }
    // Simple insertion sort for small window
    for (int i = 1; i < count; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[count / 2];
}

SchmittTrigger::SchmittTrigger()
    : threshLow(0.0f), threshHigh(0.0f), state(false) {}

SchmittTrigger::SchmittTrigger(float low, float high)
    : threshLow(low), threshHigh(high), state(false) {}

bool SchmittTrigger::update(float value) {
    if (state && value < threshLow) {
        state = false;
    } else if (!state && value > threshHigh) {
        state = true;
    }
    return state;
}