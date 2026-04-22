#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "sensors/ultrasonic.h"
#include "sensors/pixy_cam.h"
#include "sensors/gyro.h"
#include "control/pd_controller.h"
#include "control/state_machine.h"
#include "actuators/motor.h"
#include "actuators/servo_ctrl.h"

// ═══ GLOBAL OBJECTS ═══
static UltrasonicSensor usLeft;
static UltrasonicSensor usRight;
static PixyCam pixyCam;
static Gyroscope gyro;
static Motor motor;
static ServoCtrl steeringServo;
static StateMachine fsm;

// ═══ SHARED DATA ═══
static SensorData sharedSensorData;
static SemaphoreHandle_t sensorMutex = NULL;

// ═══ DEBUG TIMING ═══
static unsigned long lastDebugMs = 0;

// ═══ SENSOR TASK (Core 0) ═══
void sensorTask(void* param) {
    (void)param;
    for (;;) {
        float dL = usLeft.readFilteredCm();
        float dR = usRight.readFilteredCm();

        pixyCam.update();
        BlockInfo pillar = pixyCam.getClosestPillar();
        BlockInfo parkMarker = pixyCam.getParkingMarker();

        gyro.update(dL, dR);

        // Write to shared struct under mutex
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            sharedSensorData.distL = dL;
            sharedSensorData.distR = dR;
            sharedSensorData.pillar = pillar;
            sharedSensorData.parkingMarker = parkMarker;
            sharedSensorData.heading = gyro.getHeading();
            sharedSensorData.yawRate = gyro.getYawRate();
            sharedSensorData.lapCount = gyro.getLapCount();
            sharedSensorData.cornerCount = gyro.getCornerCount();
            xSemaphoreGive(sensorMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_MS));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.printf("\n[BOOT] WRO 2026 Future Engineers\n");
    Serial.printf("[BOOT] Challenge=%d Direction=%d\n", CHALLENGE_MODE, DRIVING_DIRECTION);

    // I2C for gyro
    Wire.begin(SDA_PIN, SCL_PIN);

    // Gyro init + calibration (blocking ~3s, vehicle must be still)
    gyro.init();

    // PixyCam2 init
    pixyCam.init();
    Serial.printf("[BOOT] PixyCam2 initialized\n");

    // Servo init
    steeringServo.init();
    Serial.printf("[BOOT] Servo centered\n");

    // Motor init
    motor.init();
    Serial.printf("[BOOT] Motor ready\n");

    // Ultrasonic init
    usLeft.init(TRIG_L, ECHO_L);
    usRight.init(TRIG_R, ECHO_R);
    Serial.printf("[BOOT] Ultrasonics ready\n");

    // Start button
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);

    // Create mutex
    sensorMutex = xSemaphoreCreateMutex();

    // Init shared data
    memset((void*)&sharedSensorData, 0, sizeof(SensorData));

    // Create sensor task on Core 0
    xTaskCreatePinnedToCore(
        sensorTask,
        "Sensors",
        4096,
        NULL,
        1,       // priority
        NULL,
        0        // Core 0
    );

    // FSM starts in wait state
    fsm = StateMachine();

    Serial.printf("[BOOT] Setup complete. Waiting for start button...\n");
}

void loop() {
    // Local copy of sensor data
    SensorData local;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(&local, (const void*)&sharedSensorData, sizeof(SensorData));
        xSemaphoreGive(sensorMutex);
    }

    float dt = (float)CONTROL_LOOP_MS * 0.001f;

    // Run state machine
    fsm.update(local, dt);

    // Apply actuators
    motor.rampTo(fsm.targetSpeed);
    steeringServo.setServoAngle(fsm.targetSteerAngle);

    // Debug output
    unsigned long now = millis();
    if (now - lastDebugMs >= DEBUG_PRINT_MS) {
        lastDebugMs = now;
        Serial.printf("[%s] L=%.0f R=%.0f H=%.1f Lap=%d Spd=%d Str=%.0f",
            fsm.getStateName(),
            local.distL, local.distR,
            local.heading, local.lapCount,
            fsm.targetSpeed, fsm.targetSteerAngle);
        if (local.pillar.detected) {
            Serial.printf(" P:s%d d=%.0f", local.pillar.signature, local.pillar.distanceEst);
        }
        Serial.printf("\n");
    }

    delay(CONTROL_LOOP_MS);
}