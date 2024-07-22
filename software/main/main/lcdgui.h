#ifndef LCDGUI_H
#define LCDGUI_H

#include <LiquidCrystal_I2C.h>
#include "temp.h"
#include "control.h"
#include "_states.h"
#include "encodergui.h"

extern LiquidCrystal_I2C lcd;

void initializeLcdGui();
void updateLcdGui(const TempData& tempData, const ControlData& controlData, const char* stateSymbol, MachineState currentState);
void updateSettingsGui();

void createCustomCharacters(); 

const char* getStateSymbol(State* state);
const char* getStateName(State* state);

extern bool cursorOnStart;
extern bool cursorOnPID;
extern int selectedPIDIndex; 
extern CursorState cursorState;

#endif
