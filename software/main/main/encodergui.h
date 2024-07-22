#ifndef ENCODERGUI_H
#define ENCODERGUI_H

#include <ClickEncoder.h>
#include <TimerOne.h>
#include "_states.h" // Include the states header

extern ClickEncoder *encoder;

extern bool cursorOnStart; // Declare cursorOnStart as extern
extern String stateCommand; // Declare stateCommand as extern

void initializeEncoder();
void encoderEvent(State* currentState);
void timerIsr();
void handleUserEvents(State* currentState); // Declare the new function

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
