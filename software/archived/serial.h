#ifndef SERIAL_H
#define SERIAL_H

#include "temp.h"
#include "control.h"
#include "timing.h"

void handleSerialCommands();
void printData(const TempData &tempData, const ControlData &controlData, const Timing &timing, const char *stateName);

#endif
