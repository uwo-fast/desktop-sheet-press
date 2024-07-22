#ifndef MONITORING_H
#define MONITORING_H

#include "temp.h"
#include "control.h"
#include "state.h"
#include "system_health.h"

SystemHealthInfo checkSystemHealth(TempData tempData, ControlData controlData, StateMachineInfo stateInfo, SystemHealthInfo previousHealthInfo);

#endif // MONITORING_H
