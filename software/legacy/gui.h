#ifndef GUI_H
#define GUI_H

#include <LiquidCrystal_I2C.h>
#include <ClickEncoder.h>
#include <TimerOne.h>

#include <LiquidMenu.h>

#include "mainStates.h"

#include "temp.h"
#include "control.h"

class CursorState
{
public:
    int position;
    int maxPositions;
    bool isSelected;

    CursorState(int maxPos) : position(0), maxPositions(maxPos), isSelected(false) {}

    void moveCursor(int direction)
    {
        position = (position + direction + maxPositions) % maxPositions;
    }
};

extern CursorState cursorState;

extern LiquidCrystal_I2C lcd;

void initializeLcdGui();
void updateLcdGui(const TempData &tempData, const ControlData &controlData, State *state);

const char *getStateSymbol(State *state);

extern ProgramData pData;

extern ClickEncoder *encoder;
extern MachineState stateCommand;

void timerIsr();
void initializeEncoder();
void processEncoderEvents(State *state);

// Define possible events
enum UserEvent
{
    EV_NONE,
    EV_BTN_CLICKED,
    EV_BTN_2CLICKED,
    EV_BTN_HELD,
    EV_BTN_RELEASED,
    EV_ENCUP,
    EV_ENCDN
};

extern UserEvent uEvent;

#endif // GUI_H
