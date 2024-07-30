#ifndef SDCARD_H
#define SDCARD_H

#include <SD.h>
#include "temp.h"
#include "control.h"
#include "timing.h"

bool createNewLogFile();
void logData(const TempData &tempData, const ControlData &controlData, const Timing &timing, const char *stateName);

#endif
