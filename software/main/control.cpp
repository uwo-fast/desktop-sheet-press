#include "control.h"

float input[NUM_SENSORS], output[NUM_SENSORS];
PID *pidControllers[NUM_SENSORS];

void initializePIDs(ProgramData &pData)
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        pidControllers[i] = new PID(&input[i], &output[i], &pData.setpoint[i], pData.Kp[i], pData.Ki[i], pData.Kd[i], DIRECT);
        pidControllers[i]->SetMode(AUTOMATIC);
        pidControllers[i]->SetSampleTime(CONTROL_INTERVAL);
        pidControllers[i]->SetOutputLimits(0, 255);
    }
}

ControlData controlLogic(const TempData &tempData, State *currentState)
{
    ControlData controlData;

    if (currentState == preheatingState || heatingState)
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            input[i] = tempData.temperatures[i];
            pidControllers[i]->Compute();
            controlData.outputs[i] = output[i];
        }
    }
    else
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            controlData.outputs[i] = 0;
        }
    }
    return controlData;
}
