#ifndef TIMING_H
#define TIMING_H

#include "mainStates.h"

struct CountTimers
{
    unsigned long elapsed;
    unsigned long durationRemaining;
};

struct PointInTime
{
    unsigned long preStart;
    unsigned long heatStart;
    unsigned long coolStart;
};

struct LastUpdateTime
{
    unsigned long master;
    unsigned long control;
    unsigned long gui;
    unsigned long log;
};

struct Timing
{
    CountTimers ct;
    PointInTime pit;
    LastUpdateTime lut;
};

extern Timing timing;

void updateTiming(State *state);

#endif // TIMING_H