#include "temp.h"

MAX31855soft thermocouples[NUM_SENSORS] = {
    MAX31855soft(PIN_TC_CS1, PIN_TC_DO, PIN_TC_CLK),
    MAX31855soft(PIN_TC_CS2, PIN_TC_DO, PIN_TC_CLK)};

void initTCs()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        thermocouples[i].begin();
    }
}

TempData readTemps()
{
    TempData tempData;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        float tempRead = thermocouples[i].getTemperature();
        tempData.temperatures[i] = tempRead;
        uint8_t error = thermocouples[i].detectThermocouple();
        tempData.errorFlags[i] = error;
    }
    return tempData;
}

// Static variable to store the last valid data
static TempData lastValidData = {};

// Function to process temperature data
TempData processTempData(const TempData &newData)
{
    TempData processedData = newData;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (newData.temperatures[i] == 0 || newData.errorFlags[i])
        {
            // Replace with last valid temperature if current reading is zero or an error
            processedData.temperatures[i] = lastValidData.temperatures[i];
        }
        else
        {
            // Update last valid temperature
            lastValidData.temperatures[i] = newData.temperatures[i];
            lastValidData.errorFlags[i] = newData.errorFlags[i];
        }
    }

    return processedData;
}
