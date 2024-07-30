#include "relay.h"
#include <Arduino.h>

// Function to control relays using slow PWM
void slowPWM(int SSRn, unsigned long &cycleStart, int output)
{
    // Get current time in milliseconds
    unsigned long currentMillis = millis();

    int dutyCycle = output / 255;

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
    // If the cycle is complete, reset the cycle start time
    else
    {
        cycleStart = currentMillis;
    }
}

// Wrapper function to write control data to relays
void writeRelays(const ControlData &controlData)
{
    static unsigned long cycleStart1 = 0;
    static unsigned long cycleStart2 = 0;

    slowPWM(PIN_SSR1, cycleStart1, controlData.outputs[0]);
    slowPWM(PIN_SSR2, cycleStart2, controlData.outputs[1]);
}
