#ifndef TEMP_H
#define TEMP_H

#include "config.h"
#include <Adafruit_MAX31855.h>

struct TempData
{
    double temperatures[NUM_SENSORS];
    uint8_t errorFlags[NUM_SENSORS];

    double getTemp(int sensorIndex) const
    {
        if (sensorIndex >= 0 && sensorIndex < NUM_SENSORS)
        {
            return temperatures[sensorIndex];
        }
        return 0.0;
    }

    uint8_t getEFlag(int sensorIndex) const
    {
        if (sensorIndex >= 0 && sensorIndex < NUM_SENSORS)
        {
            return errorFlags[sensorIndex];
        }
        return 0;
    }
};


void initTCs();

TempData readTemps();
TempData processTempData(const TempData &newData);

extern TempData tempData;

#endif // TEMP_H
