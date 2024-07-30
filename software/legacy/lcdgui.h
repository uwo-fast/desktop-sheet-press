#ifndef LCDGUI_H
#define LCDGUI_H

#include <LiquidCrystal_I2C.h>
#include "temp.h"
#include "control.h"
#include "mainStates.h"
#include "encodergui.h"

#include "progdata.h"

extern LiquidCrystal_I2C lcd;

void initializeLcdGui();
void updateLcdGui(const TempData& tempData, const ControlData& controlData, State* state);

const char* getStateSymbol(State* state);

extern bool cursorOnStart;
extern CursorState cursorState;

extern ProgramData pData;

#endif
