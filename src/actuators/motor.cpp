#include "motor.h"

Motor::Motor() : currentPWM(0) {}

void Motor::init() {
    pinMode(MOTOR_DIR_A, OUTPUT);
    pinMode(MOTOR_DIR_B, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_A, LOW);
    digitalWrite(MOTOR_DIR_B, LOW);
    analogWrite(MOTOR_PWM_PIN, 0);
    currentPWM = 0;
}

int Motor::applyDeadzone(int pwm) {
    if (pwm == 0) return 0;
    // Remap [1..255] to [MOTOR_DEADZONE..255] so any nonzero input
    // actually produces enough voltage to overcome static friction
    int absPwm = abs(pwm);
    int remapped = map(absPwm, 1, 255, MOTOR_DEADZONE, 255);
    return (pwm > 0) ? remapped : -remapped;
}

void Motor::applyPWM(int pwm) {
    int compensated = applyDeadzone(pwm);

    if (compensated > 0) {
        // Forward
        digitalWrite(MOTOR_DIR_A, HIGH);
        digitalWrite(MOTOR_DIR_B, LOW);
        analogWrite(MOTOR_PWM_PIN, compensated);
    } else if (compensated < 0) {
        // Reverse
        digitalWrite(MOTOR_DIR_A, LOW);
        digitalWrite(MOTOR_DIR_B, HIGH);
        analogWrite(MOTOR_PWM_PIN, -compensated);
    } else {
        // Coast (both direction pins low)
        digitalWrite(MOTOR_DIR_A, LOW);
        digitalWrite(MOTOR_DIR_B, LOW);
        analogWrite(MOTOR_PWM_PIN, 0);
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
        // Speeding up or reducing braking
        int step = (abs(targetPWM) > abs(currentPWM)) ? RAMP_UP_STEP : RAMP_DOWN_STEP;
        currentPWM += min(diff, step);
    } else if (diff < 0) {
        // Slowing down or increasing braking
        int step = (abs(targetPWM) < abs(currentPWM)) ? RAMP_DOWN_STEP : RAMP_UP_STEP;
        currentPWM += max(diff, -step);
    }

    applyPWM(currentPWM);
}

void Motor::stopMotor() {
    // Immediate stop, bypass ramp
    currentPWM = 0;
    applyPWM(0);
}