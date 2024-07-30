#ifndef CONTROL_H
#define CONTROL_H

#include "PID_v3.h"
#include "mainStates.h"
#include "temp.h"

struct ControlData
{
    float outputs[NUM_SENSORS];
};

extern ControlData controlData;

extern PID *pidControllers[NUM_SENSORS];

void initializePIDs(ProgramData &pData);
ControlData controlLogic(const TempData &tempData, State *currentState);

#endif
