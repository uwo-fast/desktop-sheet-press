#include "monitoring.h"
#include "config.h"
#include "progdata.h"

// Variables to keep track of thermal runaway detection
static int runawayCycles[NUM_SENSORS] = {0};

bool checkThermalRunaway(const TempData &tempData)
{
    unsigned long currentMillis = millis();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        float currentTemperature = tempData.temperatures[i];
        float currentSetpoint = pData.setpoint[i];

        // Check if the current temperature exceeds the pData.setpoint by more than the delta threshold
        if (currentTemperature > currentSetpoint + THRM_RUNAWAY_DELTA)
        {
            runawayCycles[i]++;
            // Check if the number of cycles has reached the maximum allowed cycles
            if (runawayCycles[i] >= THRM_RUNAWAY_CYCLES)
            {
                // Set the system to error state
                return true;
            }
        }
        else
        {
            // Reset the runaway cycle count if the temperature is within the threshold
            runawayCycles[i] = 0;
            return false;
        }
    }
}
