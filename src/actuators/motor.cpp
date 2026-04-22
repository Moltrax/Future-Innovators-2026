#include "motor.h"

Motor::Motor() : currentPWM(0) {}

void Motor::init() {
    pinMode(MOTOR_DIR1, OUTPUT);
    pinMode(MOTOR_DIR2, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    digitalWrite(MOTOR_DIR1, LOW);
    digitalWrite(MOTOR_DIR2, LOW);
    analogWrite(MOTOR_PWM, 0);
    currentPWM = 0;
}

int Motor::applyDeadzone(int pwm) {
    if (pwm == 0) return 0;
    // Remap [1..255] to [MOTOR_DEADZONE..255]
    int absPwm = abs(pwm);
    int remapped = map(absPwm, 1, 255, MOTOR_DEADZONE, 255);
    return (pwm > 0) ? remapped : -remapped;
}

void Motor::applyPWM(int pwm) {
    int compensated = applyDeadzone(pwm);

    if (compensated > 0) {
        digitalWrite(MOTOR_DIR1, HIGH);
        digitalWrite(MOTOR_DIR2, LOW);
        analogWrite(MOTOR_PWM, compensated);
    } else if (compensated < 0) {
        digitalWrite(MOTOR_DIR1, LOW);
        digitalWrite(MOTOR_DIR2, HIGH);
        analogWrite(MOTOR_PWM, -compensated);
    } else {
        digitalWrite(MOTOR_DIR1, LOW);
        digitalWrite(MOTOR_DIR2, LOW);
        analogWrite(MOTOR_PWM, 0);
    }
}

void Motor::setMotorPWM(int targetPWM) {
    targetPWM = constrain(targetPWM, -255, 255);
    currentPWM = targetPWM;
    applyPWM(currentPWM);
}

void Motor::rampTo(int targetPWM) {
    targetPWM = constrain(targetPWM, -255, 255);
    int diff = targetPWM - currentPWM;

    if (diff > 0) {
        // Accelerating or reducing braking
        int step = (abs(targetPWM) > abs(currentPWM)) ? RAMP_UP_STEP : RAMP_DOWN_STEP;
        currentPWM += min(diff, step);
    } else if (diff < 0) {
        int step = (abs(targetPWM) < abs(currentPWM)) ? RAMP_DOWN_STEP : RAMP_UP_STEP;
        currentPWM += max(diff, -step);
    }

    applyPWM(currentPWM);
}

void Motor::stopMotors() {
    currentPWM = 0;
    applyPWM(0);
}