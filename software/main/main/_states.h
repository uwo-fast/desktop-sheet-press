#ifndef _STATES_H
#define _STATES_H

#include <Arduino.h>
#include <StateMachine.h>
#include "config.h"
#include "temp.h"
#include "dataproc.h"
#include "control.h"

enum MachineState {
    STANDBY,
    PREHEATING,
    HEATING,
    COOLING,
    ERROR,
    SETTINGS
};

struct Timing {
    unsigned long preStartTime;
    unsigned long elapsedTime;
    unsigned long heatingDuration;
    unsigned long heatStartTime;
    unsigned long durationRemaining;
    unsigned long lastUpdateTime; 
    unsigned long lastHighFreqUpdate;
};

extern Timing timing;

void updateTiming(MachineState currentState);

// Declare state functions
void settings();
void standby();
void preheating();
void heating();
void cooling();
void error();

// Declare transition functions
bool toSettings(); 
bool toPreheating();
bool toHeating();
bool toCooling();
bool toStandby();
bool toError();

extern StateMachine machine;

// Declare states
extern State* settingsState; 
extern State* standbyState;
extern State* preheatingState;
extern State* heatingState;
extern State* coolingState;
extern State* errorState;

extern String stateCommand;

void initializeSystem();  // Declaration of initializeSystem
void initializeTransitions();  // Declaration of initializeTransitions

MachineState getCurrentState(State* currentState);

#endif
