#include "encodergui.h"
#include "lcdgui.h"
#include <TimerOne.h>
#include "serial.h"
#include "control.h"
#include "config.h"
#include "_states.h"

ClickEncoder *encoder;
int16_t encLastPos, encNewPos; // Encoder position variables
UserEvent uEvent = EV_NONE; // Define uEvent

extern bool cursorOnStart; // Use extern declaration for cursorOnStart
extern String stateCommand; // Use extern declaration for stateCommand

CursorState cursorState(2); // Default to 2 positions for standby screen

void initializeEncoder() {
    encoder = new ClickEncoder(PIN_ENC_DT, PIN_ENC_CLK, PIN_ENC_SW, 4, true); // Enable pullup resistors
    Timer1.initialize(1000); // 1 ms
    Timer1.attachInterrupt(timerIsr);
    encLastPos = encNewPos = 0;
}

void encoderEvent(State* currentState) {
    static unsigned long lastEncTime = 0;

    encNewPos += encoder->getValue();

    if (encNewPos != encLastPos) {
        lastEncTime = millis();

        int direction = (encNewPos < encLastPos) ? 1 : -1;
    #ifdef REVERSE_ENCODER
        direction = -direction;
    #endif

        uEvent = (direction == 1) ? EV_ENCDN : EV_ENCUP;

        switch (getCurrentState(currentState)) {
            case SETTINGS:
                if (cursorState.isSelected) {
                    double currentValue;
                    if (cursorState.position == 0) {
                        currentValue = pidControllers[0]->GetKp();
                        pidControllers[0]->SetTunings(currentValue + direction, pidControllers[0]->GetKi(), pidControllers[0]->GetKd());
                    } else if (cursorState.position == 1) {
                        currentValue = pidControllers[0]->GetKi();
                        pidControllers[0]->SetTunings(pidControllers[0]->GetKp(), currentValue + direction, pidControllers[0]->GetKd());
                    } else if (cursorState.position == 2) {
                        currentValue = pidControllers[0]->GetKd();
                        pidControllers[0]->SetTunings(pidControllers[0]->GetKp(), pidControllers[0]->GetKi(), currentValue + direction);
                    } else if (cursorState.position == 4) {
                        setpoint[0] = static_cast<int>(setpoint[0]) + 5 * direction; // Change setpoint in increments of 5
                        setpoint[1] = setpoint[0]; // Assuming both setpoints are the same
                    }
                } else {
                    cursorState.moveCursor(direction);
                }
                break;

            case STANDBY:
                cursorOnStart = !cursorOnStart;
                break;

            case PREHEATING:
            case HEATING:
            case COOLING:
                if (cursorState.isSelected) {
                    if (cursorState.position == 0) {
                        setpoint[0] = static_cast<int>(setpoint[0]) + 5 * direction; // Change setpoint in increments of 5
                        setpoint[1] = setpoint[0]; // Assuming both setpoints are the same
                    }
                } else {
                    cursorState.moveCursor(direction);
                }
                break;

            default:
                break;
        }

        encLastPos = encNewPos;
    }

    ClickEncoder::Button b = encoder->getButton();
    if (b != ClickEncoder::Open) {
        switch (b) {
            case ClickEncoder::Clicked:
                uEvent = EV_BTN_CLICKED;
                break;
            case ClickEncoder::DoubleClicked:
                uEvent = EV_BTN_2CLICKED;
                break;
            case ClickEncoder::Held:
                uEvent = EV_BTN_HELD;
                break;
            case ClickEncoder::Released:
                uEvent = EV_BTN_RELEASED;
                break;
            default:
                uEvent = EV_NONE;
                break;
        }
    }
}

void timerIsr() {
    encoder->service();
}

void handleUserEvents(State* currentState) {
    if (uEvent == EV_BTN_CLICKED) {
        switch (getCurrentState(currentState)) {
            case STANDBY:
                if (cursorOnStart) {
                    stateCommand = "preheat";
                } else {
                    stateCommand = "settings";
                }
                break;

            case SETTINGS:
                if (cursorState.isSelected) {
                    if (cursorState.position == 5) { // If ^ is selected
                        stateCommand = "standby";
                    } else {
                        cursorState.isSelected = false;
                    }
                } else {
                    cursorState.isSelected = true;
                }
                break;

            case PREHEATING:
            case HEATING:
            case COOLING:
                if (cursorState.isSelected) {
                    cursorState.isSelected = false;
                } else {
                    cursorState.isSelected = true;
                }
                break;

            default:
                break;
        }
    } else if (uEvent == EV_BTN_HELD) {
        stateCommand = "standby";
    }
    // Clear the event after handling
    uEvent = EV_NONE;
}
