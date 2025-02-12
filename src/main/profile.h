#ifndef PROFILE_H
#define PROFILE_H

// Step structure for temperature profile
typedef enum
{
    RAMP, // Ramp step (heating up)
    FALL, // Fall step (cooling down)
    SOAK  // Soak step (hold at constant temp)
} ProfileStepType;

// Type 'Soak' must define a target temperature and duration; ignores rate
// Type 'Ramp' has two methods (method 1: rate, method 2: duration):
// - Method 1: define target temperature and rate
// - Method 2: define a target temperature and duration
// - Method 1 takes precedence over method 2
// Type 'Fall' must define a target temperature; ignores rate and duration
typedef struct
{
    ProfileStepType type;   // Type of step (RAMP, FALL, SOAK)
    float targetTemp;       // Target temperature (°C); relevant to RAMP, FALL, SOAK
    float rate;             // Ramp rate (°C/second) for RAMP only
    unsigned long duration; // Duration in seconds for RAMP and SOAK only
} TempProfileStep;

// Temperature profile structure, holding steps and tracking the current step
typedef struct
{
    TempProfileStep steps[10]; // Array to hold up to 10 steps
    int currentStep;           // Tracks the current step (index)
    int totalSteps;            // Total number of steps in the profile
    unsigned long remainingDuration; // Remaining duration for the current process phase
} TempProfile;

// Function prototypes
extern double updateTempProfileStep(TempProfile *profile, unsigned long elapsed);
double getRampTemp(unsigned long duration, unsigned long elapsed, double rate, double startTemp, double targetTemp);

#endif // PROFILE_H
