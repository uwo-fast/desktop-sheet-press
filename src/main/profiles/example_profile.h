#ifndef EXAMPLE_PROFILE_H
#define EXAMPLE_PROFILE_H

#include "../profile.h"

// Define the temperature profile using the TempProfile structure
// MUST start with a SOAK even if it's negligible to ensure the RAMP has a starting point
// type, targetTemp, rate, duration
TempProfile tempProfile = {
    {
        {SOAK, 100.0, 0.0, 180000},  // Soak at 100°C for 3 minutes (180000ms)
        {RAMP, 250.0, 10.0, 0},      // Ramp to 250°C at 10°C/min
        {SOAK, 250.0, 0.0, 300000}, // Soak at 250°C for 5 minutes (600000ms)
        {RAMP, 400.0, 5.0, 0},      // Ramp to 400°C at 5°C/min
        {FALL, 200.0, 0.0, 0},      // Fall to 200°C (no cooling rate defined)
        {SOAK, 200.0, 0.0, 300000}  // Soak at 200°C for 5 minutes (300000ms)
    },
    0, // Start with the first step, index starts at 0
    6, // Total number of steps in this profile, this should be the integer number of steps defined above
    0  // Remaining duration for the current process phase in milliseconds, initialized to 0
};

#endif // EXAMPLE_PROFILE_H
