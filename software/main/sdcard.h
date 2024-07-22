#ifndef SDCARD_H
#define SDCARD_H

#include "temp.h"
#include "control.h"
#include "state.h"
#include "system_health.h"

void logData(TempData tempData, ControlData controlData, StateMachineInfo stateInfo, SystemHealthInfo systemHealthInfo);

#endif // SDCARD_H
