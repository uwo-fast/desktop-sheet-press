#include "control.h"
#include <PID_v1.h>

// Define the PID control objects and variables
double setpoint[NUM_SENSORS];  // Desired temperature
double input[NUM_SENSORS], output[NUM_SENSORS];
PID* pidControllers[NUM_SENSORS];

void initializePIDs() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        setpoint[i] = 25.0;  // Initial setpoint value
        pidControllers[i] = new PID(&input[i], &output[i], &setpoint[i], 50, 0.2, 0.05, DIRECT);
        pidControllers[i]->SetMode(AUTOMATIC);  // Ensure the PID controller is in automatic mode
        pidControllers[i]->SetSampleTime(HIGH_FREQ_INTERVAL);  // Set the sample time
        pidControllers[i]->SetOutputLimits(0, 255);  // Set the output limits
    }
}

ControlData controlLogic(const TempData& tempData) {
    ControlData controlData;

    for (int i = 0; i < NUM_SENSORS; i++) {
        input[i] = tempData.temperatures[i];
        pidControllers[i]->Compute();
        controlData.outputs[i] = output[i];
    }

    return controlData;
}
