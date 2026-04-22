#include "ultrasonic.h"

UltrasonicSensor::UltrasonicSensor()
    : trigPin(-1), echoPin(-1) {}

void UltrasonicSensor::init(int trig, int echo) {
    trigPin = trig;
    echoPin = echo;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
}

float UltrasonicSensor::measureRawCm() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);

    if (duration == 0) {
        // Timeout: no echo received
        return (float)US_MAX_CM;
    }

    float cm = (float)duration * 0.0343f * 0.5f;
    return constrain(cm, 2.0f, (float)US_MAX_CM);
}

float UltrasonicSensor::readFilteredCm() {
    float raw = measureRawCm();
    filter.push(raw);
    return filter.get();
}