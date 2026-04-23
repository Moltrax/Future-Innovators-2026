#include "state_machine.h"
#include "control/pd_controller.h"

// Local PD controllers for wall centering and parking
static PDController pdSteer(KP_STEER, KD_STEER);
static PDController pdPark(KP_PARK, KD_PARK);

StateMachine::StateMachine()
    : state(WAIT_FOR_START),
      stateEntryMs(0),
      evadeEntryHeading(0.0f),
      parkingEntryHeading(0.0f),
      targetSpeed(0),
      targetSteerAngle((float)SERVO_CENTER) {}

void StateMachine::enterState(FSMState newState) {
    state = newState;
    stateEntryMs = millis();
    Serial.printf("[FSM] -> %s\n", getStateName());
}

float StateMachine::reactionDistance(int speed) {
    // Higher speed = need more look-ahead distance
    if (speed >= SPEED_MAX) return REACTION_DIST_FAST;
    if (speed >= SPEED_MEDIUM) return REACTION_DIST_MEDIUM;
    return REACTION_DIST_SLOW;
}

int StateMachine::speedForPillarDist(float dist) {
    // Linear map: far=SPEED_MAX, close=SPEED_MIN
    if (dist >= REACTION_DIST_FAST) return SPEED_MAX;
    if (dist <= EVADE_TRIGGER_CM) return SPEED_MIN;
    float ratio = (dist - EVADE_TRIGGER_CM) / (REACTION_DIST_FAST - EVADE_TRIGGER_CM);
    return SPEED_MIN + (int)(ratio * (float)(SPEED_MAX - SPEED_MIN));
}

void StateMachine::update(const SensorData& sd, float dt) {
    // Keep both raw and corrected (perpendicular) distances
    float rawDistL = sd.distL;
    float rawDistR = sd.distR;
    // Use corrected distances (perpendicular to wall) for PD centering
    float distL = sd.distL_corr;
    float distR = sd.distR_corr;
    BlockInfo pillar = sd.pillar;
    BlockInfo parking = sd.parkingMarker;
    float heading = sd.heading;
    int lapCount = sd.lapCount;
    unsigned long now = millis();

    switch (state) {

    case WAIT_FOR_START:
        targetSpeed = 0;
        targetSteerAngle = (float)SERVO_CENTER;
        if (digitalRead(START_BUTTON_PIN) == LOW) {
            enterState(DRIVE);
            pdSteer.reset();
        }
        break;

    case DRIVE: {
        // Check lap completion
        if (lapCount >= TOTAL_LAPS) {
            if (CHALLENGE_MODE == 1) {
                enterState(FIND_PARK);
                break;
            } else {
                enterState(OPEN_FINISH);
                break;
            }
        }

        targetSpeed = SPEED_MAX;

        // PD wall centering: setpoint=0 means equal distance to both walls
        float wallError = distL - distR;
        float steerCorrection = pdSteer.compute(0.0f, wallError, dt);
        targetSteerAngle = (float)SERVO_CENTER + steerCorrection;

        // Check for pillars (obstacle mode only)
        if (CHALLENGE_MODE == 1 && pillar.detected) {
            float reactDist = reactionDistance(targetSpeed);
            if (pillar.distanceEst < reactDist) {
                enterState(APPROACH_PILLAR);
            }
        }
    } break;

    case APPROACH_PILLAR: {
        if (!pillar.detected) {
            // Lost sight, return to driving
            enterState(DRIVE);
            break;
        }

        // Reduce speed as we approach
        targetSpeed = speedForPillarDist(pillar.distanceEst);

        // Pre-position: steer to opposite side of pillar
        // Green (sig 2): pass on LEFT, so move vehicle RIGHT first
        // Red (sig 1): pass on RIGHT, so move vehicle LEFT first
        if (pillar.signature == PIXY_SIG_GREEN) {
            // Move right to create room on left
            float offset = (float)SERVO_CENTER + 15.0f;
            targetSteerAngle = offset;
        } else {
            // Move left to create room on right
            float offset = (float)SERVO_CENTER - 15.0f;
            targetSteerAngle = offset;
        }

        // Transition to evade when close enough
        if (pillar.distanceEst < EVADE_TRIGGER_CM) {
            evadeEntryHeading = heading;
            if (pillar.signature == PIXY_SIG_GREEN) {
                enterState(EVADE_LEFT);
            } else {
                enterState(EVADE_RIGHT);
            }
        }
    } break;

    case EVADE_LEFT: {
        targetSpeed = SPEED_SLOW;
        targetSteerAngle = (float)SERVO_MIN_ANGLE; // Hard left

        bool timeout = (now - stateEntryMs) > EVADE_TIMEOUT_MS;
        // Pillar passed: it's no longer in front or distance increased significantly
        bool pillarPassed = !pillar.detected || pillar.distanceEst > EVADE_TRIGGER_CM * 2.0f;

        if (timeout || pillarPassed) {
            enterState(STRAIGHTEN);
            pdSteer.reset();
        }
    } break;

    case EVADE_RIGHT: {
        targetSpeed = SPEED_SLOW;
        targetSteerAngle = (float)SERVO_MAX_ANGLE; // Hard right

        bool timeout = (now - stateEntryMs) > EVADE_TIMEOUT_MS;
        bool pillarPassed = !pillar.detected || pillar.distanceEst > EVADE_TRIGGER_CM * 2.0f;

        if (timeout || pillarPassed) {
            enterState(STRAIGHTEN);
            pdSteer.reset();
        }
    } break;

    case STRAIGHTEN: {
        targetSpeed = SPEED_MEDIUM;

        float wallError = distL - distR;
        float steerCorrection = pdSteer.compute(0.0f, wallError, dt);
        targetSteerAngle = (float)SERVO_CENTER + steerCorrection;

        if (fabsf(wallError) < CENTER_TOLERANCE_CM) {
            enterState(DRIVE);
        }
    } break;

    case FIND_PARK: {
        targetSpeed = SPEED_SLOW;

        // Keep centering while scanning
        float wallError = distL - distR;
        float steerCorrection = pdSteer.compute(0.0f, wallError, dt);
        targetSteerAngle = (float)SERVO_CENTER + steerCorrection;

        // Detect parking lot: magenta marker via Pixy OR sudden gap in US reading
        bool magentaSeen = parking.detected && parking.distanceEst < REACTION_DIST_FAST;
        // Gap detection: use raw ultrasonic readings because thresholds are based
        // on direct sensor measurements (not corrected perpendicular values)
        bool gapDetected = (rawDistR > GAP_SCHMITT_HIGH) || (rawDistL > GAP_SCHMITT_HIGH);

        if (magentaSeen || gapDetected) {
            enterState(PARKING_ALIGN);
            pdPark.reset();
        }
    } break;

    case PARKING_ALIGN: {
        targetSpeed = SPEED_PARK;

        // Drive alongside parking lot, use magenta marker to position
        if (parking.detected) {
            // Steer so magenta marker is centered-ish in frame (we want to pass it)
            float pixError = (float)(parking.x - PIXY_FRAME_CENTER);
            targetSteerAngle = (float)SERVO_CENTER + pixError * 0.15f;

            // When marker is to our side/behind, begin turn into lot
            if (parking.x > PIXY_FRAME_WIDTH - 40 || !parking.detected) {
                parkingEntryHeading = heading;
                enterState(PARKING_ENTER);
            }
        } else {
            // Lost marker, use US to detect gap and turn in
            parkingEntryHeading = heading;
            enterState(PARKING_ENTER);
        }
    } break;

    case PARKING_ENTER: {
        targetSpeed = SPEED_PARK;

        // Turn ~90deg toward outer wall to enter parking lot
        // Direction depends on which side the parking is
        targetSteerAngle = (float)SERVO_MAX_ANGLE; // Turn right toward outer wall

        float headingChange = fabsf(heading - parkingEntryHeading);
        if (headingChange >= PARKING_TURN_DEG) {
            enterState(PARKING_CENTER);
            pdPark.reset();
        }
    } break;

    case PARKING_CENTER: {
        targetSpeed = SPEED_PARK;

        // Center between walls/markers, align parallel
        float wallError = distL - distR;
        float steerCorrection = pdPark.compute(0.0f, wallError, dt);
        targetSteerAngle = (float)SERVO_CENTER + steerCorrection;

        // Check if parallel: both US readings close to equal
        bool isParallel = fabsf(distL - distR) < PARKING_PARALLEL_TOL;
        // Check if inside lot: both corrected readings are within reasonable range
        bool insideLot = (distL < 30.0f && distR < 30.0f);

        if (isParallel && insideLot) {
            enterState(PARK_DONE);
        }
    } break;

    case PARK_DONE:
        targetSpeed = 0;
        targetSteerAngle = (float)SERVO_CENTER;
        enterState(DONE);
        break;

    case OPEN_FINISH:
        // Stop in starting section after 3 laps
        targetSpeed = 0;
        targetSteerAngle = (float)SERVO_CENTER;
        enterState(DONE);
        break;

    case DONE:
        targetSpeed = 0;
        targetSteerAngle = (float)SERVO_CENTER;
        break;
    }
}

const char* StateMachine::getStateName() const {
    switch (state) {
        case WAIT_FOR_START:    return "WAIT_FOR_START";
        case DRIVE:             return "DRIVE";
        case APPROACH_PILLAR:   return "APPROACH_PILLAR";
        case EVADE_LEFT:        return "EVADE_LEFT";
        case EVADE_RIGHT:       return "EVADE_RIGHT";
        case STRAIGHTEN:        return "STRAIGHTEN";
        case FIND_PARK:         return "FIND_PARK";
        case PARKING_ALIGN:     return "PARKING_ALIGN";
        case PARKING_ENTER:     return "PARKING_ENTER";
        case PARKING_CENTER:    return "PARKING_CENTER";
        case PARK_DONE:         return "PARK_DONE";
        case OPEN_FINISH:       return "OPEN_FINISH";
        case DONE:              return "DONE";
        default:                return "UNKNOWN";
    }
}