#include "profile.h"

double updateTempProfileStep(TempProfile *profile, unsigned long elapsed)
{

    if(profile->remainingDuration == 0)
    {
        profile->currentStep++;
        if(profile->currentStep > profile->totalSteps)
        {
            return -1.0; // change to null or NaN? !!!!!!!!!
        }
        TempProfileStep *step = &profile->steps[profile->currentStep];
        profile->remainingDuration = step->duration;
    }
    else
    {
        profile->remainingDuration -= elapsed;
    }    

    // Get the current step from the profile
    TempProfileStep *step = &profile->steps[profile->currentStep];

    // Check the type of step and return the target temperature
    switch (step->type)
    {
    case RAMP:
        if (profile->currentStep != 0)
        {
            TempProfileStep *prevStep = &profile->steps[profile->currentStep - 1];
            return getRampTemp(step->duration, profile->remainingDuration, step->rate, prevStep->targetTemp, step->targetTemp);
        }
        else
        {
            return getRampTemp(step->duration, profile->remainingDuration, step->rate, 0.0, step->targetTemp);
        }
    case FALL:
        return step->targetTemp;
    case SOAK:
        return step->targetTemp;
    default:
        return 0.0;
    }
}

double getRampTemp(unsigned long duration, unsigned long elapsed, double rate, double startTemp, double targetTemp)
{
    // Calculate the temperature based on the elapsed time and ramp rate
    double tempChange = rate * (elapsed / 1000.0);
    double currentTemp = startTemp + tempChange;

    // Ensure the temperature does not exceed the target temperature
    if (rate > 0 && currentTemp > targetTemp)
    {
        currentTemp = targetTemp;
    }
    else if (rate < 0 && currentTemp < targetTemp)
    {
        currentTemp = targetTemp;
    }

    return currentTemp;
}