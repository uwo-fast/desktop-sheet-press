#include "control.h"
#include <PID_v1.h>

double input[NUM_SENSORS], output[NUM_SENSORS];
PID* controllers[NUM_SENSORS];

void initializePIDs(ProgramData& pData) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        controllers[i] = new PID(&input[i], &output[i], &pData.setpoint[i], pData.Kp[i], pData.Ki[i], pData.Kd[i], DIRECT);
        controllers[i]->SetMode(AUTOMATIC);
        controllers[i]->SetSampleTime(CONTROL_INTERVAL);
        controllers[i]->SetOutputLimits(0, 255);
    }
}

ControlData controlLogic(const TempData& tempData) {
    ControlData controlData;

    for (int i = 0; i < NUM_SENSORS; i++) {
        input[i] = tempData.temperatures[i];
        controllers[i]->Compute();
        controlData.outputs[i] = output[i];
    }

    return controlData;
}
