#include "bt_debug.h"

#ifdef DEBUG_BT_ENABLED

#include <BluetoothSerial.h>
#include "config.h"
#include "control/state_machine.h"

static BluetoothSerial btSerial;
static unsigned long lastSendMs = 0;
static char txBuf[256];
static char rxBuf[128];
static int rxIndex = 0;

// Mirrors of tuning values that can be changed at runtime via BT
// These start at the config.h defaults and can be overwritten by commands
static float liveKpSteer = KP_STEER;
static float liveKdSteer = KD_STEER;
static float liveKpPark = KP_PARK;
static float liveKdPark = KD_PARK;
static int liveSpeedMax = SPEED_MAX;
static int liveSpeedPark = SPEED_PARK;
static float liveEvadeTrigger = EVADE_TRIGGER_CM;

// Expose live values so main code can read them if desired
float btGetKpSteer()       { return liveKpSteer; }
float btGetKdSteer()       { return liveKdSteer; }
float btGetKpPark()        { return liveKpPark; }
float btGetKdPark()        { return liveKdPark; }
int   btGetSpeedMax()      { return liveSpeedMax; }
int   btGetSpeedPark()     { return liveSpeedPark; }
float btGetEvadeTrigger()  { return liveEvadeTrigger; }

static void printHelp() {
    btSerial.println("=== WRO2026 BT Debug ===");
    btSerial.println("Commands:");
    btSerial.println("  help              Show this message");
    btSerial.println("  status            Print current tuning values");
    btSerial.println("  kp <value>        Set KP_STEER");
    btSerial.println("  kd <value>        Set KD_STEER");
    btSerial.println("  kpp <value>       Set KP_PARK");
    btSerial.println("  kdp <value>       Set KD_PARK");
    btSerial.println("  smax <value>      Set SPEED_MAX (0-255)");
    btSerial.println("  spark <value>     Set SPEED_PARK (0-255)");
    btSerial.println("  evade <value>     Set EVADE_TRIGGER_CM");
    btSerial.println("  reset             Reset all to config.h defaults");
}

static void printStatus() {
    btSerial.printf("KP_STEER=%.2f  KD_STEER=%.2f\n", liveKpSteer, liveKdSteer);
    btSerial.printf("KP_PARK=%.2f   KD_PARK=%.2f\n", liveKpPark, liveKdPark);
    btSerial.printf("SPEED_MAX=%d   SPEED_PARK=%d\n", liveSpeedMax, liveSpeedPark);
    btSerial.printf("EVADE_TRIGGER=%.1f cm\n", liveEvadeTrigger);
}

static void handleCommand(const char* cmd) {
    float fval;
    int ival;

    if (strncmp(cmd, "help", 4) == 0) {
        printHelp();
    }
    else if (strncmp(cmd, "status", 6) == 0) {
        printStatus();
    }
    else if (sscanf(cmd, "kpp %f", &fval) == 1) {
        liveKpPark = fval;
        btSerial.printf("KP_PARK -> %.2f\n", liveKpPark);
    }
    else if (sscanf(cmd, "kdp %f", &fval) == 1) {
        liveKdPark = fval;
        btSerial.printf("KD_PARK -> %.2f\n", liveKdPark);
    }
    else if (sscanf(cmd, "kp %f", &fval) == 1) {
        liveKpSteer = fval;
        btSerial.printf("KP_STEER -> %.2f\n", liveKpSteer);
    }
    else if (sscanf(cmd, "kd %f", &fval) == 1) {
        liveKdSteer = fval;
        btSerial.printf("KD_STEER -> %.2f\n", liveKdSteer);
    }
    else if (sscanf(cmd, "smax %d", &ival) == 1) {
        liveSpeedMax = constrain(ival, 0, 255);
        btSerial.printf("SPEED_MAX -> %d\n", liveSpeedMax);
    }
    else if (sscanf(cmd, "spark %d", &ival) == 1) {
        liveSpeedPark = constrain(ival, 0, 255);
        btSerial.printf("SPEED_PARK -> %d\n", liveSpeedPark);
    }
    else if (sscanf(cmd, "evade %f", &fval) == 1) {
        liveEvadeTrigger = fval;
        btSerial.printf("EVADE_TRIGGER -> %.1f cm\n", liveEvadeTrigger);
    }
    else if (strncmp(cmd, "reset", 5) == 0) {
        liveKpSteer = KP_STEER;
        liveKdSteer = KD_STEER;
        liveKpPark = KP_PARK;
        liveKdPark = KD_PARK;
        liveSpeedMax = SPEED_MAX;
        liveSpeedPark = SPEED_PARK;
        liveEvadeTrigger = EVADE_TRIGGER_CM;
        btSerial.println("All values reset to defaults");
        printStatus();
    }
    else {
        btSerial.printf("Unknown command: %s\n", cmd);
        btSerial.println("Type 'help' for available commands");
    }
}

namespace BtDebug {

void init() {
    btSerial.begin(BT_DEVICE_NAME);
    Serial.printf("[BT] Bluetooth started as '%s'\n", BT_DEVICE_NAME);
    // Give BT stack time to initialize
    delay(100);
}

void update(const SensorData& sd, const StateMachine& fsm) {
    unsigned long now = millis();
    if (now - lastSendMs < BT_SEND_INTERVAL_MS) return;
    lastSendMs = now;

    // Build telemetry string
    int len = snprintf(txBuf, sizeof(txBuf),
        "%lu,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%.0f",
        now,
        fsm.getStateName(),
        sd.distL,
        sd.distR,
        sd.distL_corr,
        sd.distR_corr,
        sd.heading,
        sd.yawRate,
        sd.lapCount,
        sd.cornerCount,
        fsm.targetSpeed,
        fsm.targetSteerAngle
    );

    // Append pillar info if detected
    if (sd.pillar.detected) {
        len += snprintf(txBuf + len, sizeof(txBuf) - len,
            ",P%d:%.0f@%d",
            sd.pillar.signature,
            sd.pillar.distanceEst,
            sd.pillar.x
        );
    }

    // Append parking marker info if detected
    if (sd.parkingMarker.detected) {
        len += snprintf(txBuf + len, sizeof(txBuf) - len,
            ",M:%.0f@%d",
            sd.parkingMarker.distanceEst,
            sd.parkingMarker.x
        );
    }

    btSerial.println(txBuf);
}

void processIncoming() {
    while (btSerial.available()) {
        char c = btSerial.read();
        if (c == '\n' || c == '\r') {
            if (rxIndex > 0) {
                rxBuf[rxIndex] = '\0';
                handleCommand(rxBuf);
                rxIndex = 0;
            }
        } else if (rxIndex < (int)(sizeof(rxBuf) - 1)) {
            rxBuf[rxIndex++] = c;
        }
    }
}

} // namespace BtDebug

#endif