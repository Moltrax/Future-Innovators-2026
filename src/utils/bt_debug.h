#pragma once
#include "config.h"

#ifdef DEBUG_BT_ENABLED

#include <Arduino.h>
#include "control/state_machine.h"

namespace BtDebug {

    void init();
    void update(const SensorData& sd, const StateMachine& fsm);
    void processIncoming();

}

#endif