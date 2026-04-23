#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include "sensors/ultrasonic.h"
#include "sensors/pixy_cam.h"
#include "sensors/gyro.h"
#include "control/pd_controller.h"
#include "control/state_machine.h"
#include "actuators/motor.h"
#include "actuators/servo_ctrl.h"
#include "utils/bt_debug.h"

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

        // Compute perpendicular (corrected) distances using vehicle yaw inferred
        // from the difference between side ultrasonics. If valid, the local
        // angle (radians) is atan2(dL - dR, sensor_spacing). The measured
        // ultrasonic distance is along the sensor beam, so the perpendicular
        // distance = measured * cos(phi).
        float dL_corr = dL;
        float dR_corr = dR;
        bool usValid = (dL > 2.0f && dL < (float)US_MAX_CM && dR > 2.0f && dR < (float)US_MAX_CM);
        if (usValid) {
            float phi = atan2f(dL - dR, SENSOR_SPACING_CM); // radians
            float c = cosf(phi);
            dL_corr = dL * c;
            dR_corr = dR * c;
        }
        // Write to shared struct under mutex
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            sharedSensorData.distL = dL;
            sharedSensorData.distR = dR;
            sharedSensorData.distL_corr = dL_corr;
            sharedSensorData.distR_corr = dR_corr;
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

    // I2C bus for the GY-521 (MPU-6050) module
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // 400kHz fast mode — MPU-6050 supports up to 400kHz

    // GY-521 / MPU-6050 init + gyro offset calibration
    // Blocks for ~3 seconds, vehicle MUST be stationary on flat surface
    gyro.init();

    // PixyCam2 init
    pixyCam.init();
    Serial.printf("[BOOT] PixyCam2 initialized\n");

    // Servo init — centers steering
    steeringServo.init();
    Serial.printf("[BOOT] Servo centered\n");

    // Motor init — single rear DC motor via H-bridge
    motor.init();
    Serial.printf("[BOOT] Motor ready\n");

    // Ultrasonic sensors init
    usLeft.init(TRIG_L, ECHO_L);
    usRight.init(TRIG_R, ECHO_R);
    Serial.printf("[BOOT] Ultrasonics ready\n");

    // Start button
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);

    // Create mutex for shared sensor data
    sensorMutex = xSemaphoreCreateMutex();

    // Zero out shared data
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

#ifdef DEBUG_BT_ENABLED
    BtDebug::init();
#endif

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
        Serial.printf("[%s] L=%.0f R=%.0f Lc=%.0f Rc=%.0f H=%.1f Y=%.1f Lap=%d C=%d Spd=%d Str=%.0f",
            fsm.getStateName(),
            local.distL, local.distR, local.distL_corr, local.distR_corr,
            local.heading, local.yawRate,
            local.lapCount, local.cornerCount,
            fsm.targetSpeed, fsm.targetSteerAngle);
        if (local.pillar.detected) {
            Serial.printf(" P:s%d d=%.0f", local.pillar.signature, local.pillar.distanceEst);
        }
        if (local.parkingMarker.detected) {
            Serial.printf(" M:d=%.0f", local.parkingMarker.distanceEst);
        }
        Serial.printf("\n");
    }

#ifdef DEBUG_BT_ENABLED
    BtDebug::processIncoming();
    BtDebug::update(local, fsm);
#endif

    delay(CONTROL_LOOP_MS);
}