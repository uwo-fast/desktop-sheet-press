#include "timing.h"
#include "main.h"

Timing timing = {0};

void updateTiming(State *currentState)
{
    unsigned long currentMillis = millis();
    if (currentState == prepState || currentState == activeState)
    {
        timing.ct.elapsed = currentMillis - timing.pit.preStart;
    }
    if (currentState == activeState)
    {
        if (timing.lut.master > 0)
        {
            unsigned long elapsedSinceLastUpdate = currentMillis - timing.lut.master;
            timing.ct.durationRemaining = (timing.ct.durationRemaining > elapsedSinceLastUpdate) ? (timing.ct.durationRemaining - elapsedSinceLastUpdate) : 0;
        }
        timing.lut.master = currentMillis;
    }
    if (timing.ct.durationRemaining <= 0)
    {
        stateCommand = STANDBY;
    }
}