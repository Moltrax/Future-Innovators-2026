#include "servo_ctrl.h"

ServoCtrl::ServoCtrl() : currentAngle((float)SERVO_CENTER) {}

void ServoCtrl::init() {
    servo.attach(SERVO_PIN);
    currentAngle = (float)SERVO_CENTER;
    servo.write((int)currentAngle);
}

void ServoCtrl::setServoAngle(float target) {
    target = constrain(target, (float)SERVO_MIN_ANGLE, (float)SERVO_MAX_ANGLE);

    float diff = target - currentAngle;
    diff = constrain(diff, -MAX_SLEW_RATE, MAX_SLEW_RATE);
    currentAngle += diff;

    currentAngle = constrain(currentAngle, (float)SERVO_MIN_ANGLE, (float)SERVO_MAX_ANGLE);
    servo.write((int)currentAngle);
}

void ServoCtrl::center() {
    setServoAngle((float)SERVO_CENTER);
}