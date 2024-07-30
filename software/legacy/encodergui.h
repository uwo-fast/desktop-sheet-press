#ifndef ENCODERGUI_H
#define ENCODERGUI_H

#include "config.h"
#include "progdata.h"

#include <ClickEncoder.h>
#include <TimerOne.h>

#include "mainStates.h" 

#include "progdata.h"

extern ClickEncoder *encoder;

extern String stateCommand; 

void timerIsr();

void initializeEncoder();
void processEncoderEvents(State* state);

class CursorState {
public:
    int position;
    int maxPositions;
    bool isSelected;

    CursorState(int maxPos) : position(0), maxPositions(maxPos), isSelected(false) {}

    void moveCursor(int direction) {
        position = (position + direction + maxPositions) % maxPositions;
    }
};
extern CursorState cursorState;

// Define possible events
enum UserEvent {
    EV_NONE,
    EV_BTN_CLICKED,
    EV_BTN_2CLICKED,
    EV_BTN_HELD,
    EV_BTN_RELEASED,
    EV_ENCUP,
    EV_ENCDN
};

extern UserEvent uEvent; 

#endif
