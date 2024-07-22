#ifndef SERIAL_H
#define SERIAL_H

#include "temp.h"
#include "control.h"
#include "StateMachine.h"
#include "_states.h"

void handleSerialCommands();
void printData(const TempData& tempData, const ControlData& controlData, const char* currentStateName, const Timing& timing);
const char* getStateName(State* state);

#endif
