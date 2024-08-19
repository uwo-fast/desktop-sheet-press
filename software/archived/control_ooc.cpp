#include "control.h"

double input[NUM_SENSORS], output[NUM_SENSORS];

#ifdef USE_PID
#include <PID_v1.h>

PID* pidControllers[NUM_SENSORS];

void initializePIDs(ProgramData& pData) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        pidControllers[i] = new PID(&input[i], &output[i], &pData.setpoint[i], pData.Kp[i], pData.Ki[i], pData.Kd[i], DIRECT);
        pidControllers[i]->SetMode(AUTOMATIC);
        pidControllers[i]->SetSampleTime(CONTROL_INTERVAL);
        pidControllers[i]->SetOutputLimits(0, 255);
    }
}
#endif // USE_PID

ControlData controlLogic(const TempData& tempData) {
    ControlData controlData;

    for (int i = 0; i < NUM_SENSORS; i++) {
        input[i] = tempData.temperatures[i];
        #ifdef USE_PID
        pidControllers[i]->Compute();
        #endif // USE_PID
        #ifdef USE_OOC
        output[i] = tempData.temperatures[i] > pData.setpoint[i] ? 0 : 255;
        #endif // USE_OOC
        controlData.outputs[i] = output[i];
    }

    return controlData;
}
