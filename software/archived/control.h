#ifndef CONTROL_H
#define CONTROL_H

#include "temp.h"
#include "progdata.h"
#include <PID_v1.h>

struct ControlData {
    double outputs[NUM_SENSORS];
};

extern PID* controllers[NUM_SENSORS];
extern ProgramData pData;

void initializePIDs(ProgramData& pData);
ControlData controlLogic(const TempData& tempData);

#endif
