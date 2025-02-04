// states.h
#ifndef STATES_H
#define STATES_H

enum MachineState
{
    STANDBY,
    PREPARING,
    ACTIVE,
    TERMINATING,
    ERROR_STATE,
    MACHINE_STATE_COUNT
};

#endif // STATES_H
