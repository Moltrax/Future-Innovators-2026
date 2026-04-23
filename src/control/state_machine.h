#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensors/pixy_cam.h"

enum FSMState {
    WAIT_FOR_START,
    DRIVE,
    APPROACH_PILLAR,
    EVADE_LEFT,
    EVADE_RIGHT,
    STRAIGHTEN,
    FIND_PARK,
    PARKING_ALIGN,
    PARKING_ENTER,
    PARKING_CENTER,
    PARK_DONE,
    OPEN_FINISH,
    DONE
};

struct SensorData {
    volatile float distL;
    volatile float distR;
    // Distances corrected for vehicle yaw relative to wall (perpendicular distance)
    volatile float distL_corr;
    volatile float distR_corr;
    BlockInfo pillar;
    BlockInfo parkingMarker;
    volatile float heading;
    volatile float yawRate;
    volatile int lapCount;
    volatile int cornerCount;
};

struct StateMachine {
    FSMState state;
    unsigned long stateEntryMs;
    float evadeEntryHeading;
    float parkingEntryHeading;
    int targetSpeed;
    float targetSteerAngle;

    StateMachine();
    void update(const SensorData& sd, float dt);
    const char* getStateName() const;

private:
    float reactionDistance(int speed);
    int speedForPillarDist(float dist);
    void enterState(FSMState newState);
};