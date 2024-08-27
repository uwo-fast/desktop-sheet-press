#ifndef CONTROL_H
#define CONTROL_H

#include <PID_v1.h>
#include "config.h"
#include "temp.h"

struct ControlData
{
    double outputs[NUM_SENSORS];
};

extern ControlData controlData;

extern PID *pidControllers[NUM_SENSORS];

extern double input[NUM_SENSORS], output[NUM_SENSORS], setpoint[NUM_SENSORS];

void setPIDTuning(int n, double Kp, double Ki, double Kd);
void setPIDPoint(int n, double inputs);
ControlData controlLogic(const TempData &tempData);
ControlData noOutputs(ControlData &controlData);
#endif
