#include "control.h"
#include "config.h"

ControlData controlData;

double input[NUM_SENSORS], output[NUM_SENSORS], setpoint[NUM_SENSORS];
PID *pidControllers[NUM_SENSORS];

void setPIDTuning(int n, double Kp, double Ki, double Kd)
{
        pidControllers[n]->SetTunings(Kp, Ki, Kd);
}

void setPIDPoint(int n, double newSetpoint)
{
        setpoint[n] = newSetpoint;
}

ControlData controlLogic(const TempData &tempData, const char *stateName)
{
    if (strcmp(stateName, "preheating") == 0 || strcmp(stateName, "heating") == 0)
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
