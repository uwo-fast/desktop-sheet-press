#ifndef TEMP_H
#define TEMP_H

#include "config.h"

struct TempData {
    float temperatures[NUM_SENSORS];
    bool errorFlags[NUM_SENSORS];
};

TempData readTemps();

#endif // TEMP_H
