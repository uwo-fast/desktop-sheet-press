#ifndef CONTROL_H
#define CONTROL_H

#include "temp.h"
#include <PID_v1.h>

struct ControlData {
    float outputs[NUM_SENSORS];
};

extern double setpoint[NUM_SENSORS];
extern PID* pidControllers[NUM_SENSORS];

void initializePIDs();
ControlData controlLogic(const TempData& tempData);

#endif
