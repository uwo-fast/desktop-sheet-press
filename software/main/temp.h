#ifndef TEMP_H
#define TEMP_H

#include "config.h"
#include <Adafruit_MAX31855.h>

struct TempData
{
    double temperatures[NUM_SENSORS];
    uint8_t errorFlags[NUM_SENSORS];
};

void initTCs();

TempData readTemps();
TempData processTempData(const TempData &newData);

extern TempData tempData;

#endif // TEMP_H
