#include "relay.h"
#include <Arduino.h>

// Function to control relays using slow PWM
unsigned long slowPWM(int SSRn, unsigned long &cycleStart, double value)
{
    // Get current time in milliseconds
    unsigned long currentMillis = millis();

    double dutyCycle = value / 255.0;
    
    // If the cycle is complete, reset the cycle start time
    if (currentMillis - cycleStart > RELAY_PWM_PERIOD)
    {
        cycleStart = currentMillis;
    }

    // If within the ON part of the cycle, turn the SSR on
    if (currentMillis - cycleStart < RELAY_PWM_PERIOD * dutyCycle)
    {
        digitalWrite(SSRn, HIGH);
    }
    // If within the OFF part of the cycle, turn the SSR off
    else if (currentMillis - cycleStart >= RELAY_PWM_PERIOD * dutyCycle && currentMillis - cycleStart < RELAY_PWM_PERIOD)
    {
        digitalWrite(SSRn, LOW);
    }
    return cycleStart;
}

// Wrapper function to write control data to relays
void writeRelays(const double values[], int pins[])
{
    static unsigned long cycleStarts[NUM_RELAY] = {0};

    for (int i = 0; i < NUM_RELAY; i++)
    {
        cycleStarts[i] = slowPWM(pins[i], cycleStarts[i], values[i]);
    }
}
