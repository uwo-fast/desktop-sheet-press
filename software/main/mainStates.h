#ifndef MAINSTATES_H
#define MAINSTATES_H

#include "StateMachine.h"
#include "progdata.h"

enum MachineState
{
    STANDBY,
    PREHEATING,
    HEATING,
    ERROR,
};

// Declare main program
void mainProgram();

// Declare state functions
void standby();
void preheating();
void heating();
void error();

// Declare transition functions
bool toPreheating();
bool toHeating();
bool toStandby();
bool toError();

// Declare states
extern State *standbyState;
extern State *preheatingState;
extern State *heatingState;
extern State *errorState;

extern MachineState stateCommand;
extern StateMachine machine;
extern ProgramData pData;

void initializeSystem();
void initializeTransitions();

#endif // MAINSTATES_H
