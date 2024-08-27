#include "relay.h"
#include <Arduino.h>

// Function to control relays using slow PWM
unsigned long slowPWM(int SSR, unsigned long &cycleStart, double output)
{
    int SSRn;
    switch (SSR)
    {
    case 0:
        SSRn=PIN_SSR1;
        break;
    case 1:
        SSRn=PIN_SSR2;
        break;
    default:
        while(1)
        {
            delay(500);
            Serial.println("Invalid SSR number");
        }
        break;
    }
    // Get current time in milliseconds
    unsigned long currentMillis = millis();

    double dutyCycle = output / 255.0;
    

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
void writeRelays(const double outputs[])
{
    static unsigned long cycleStarts[NUM_RELAYS] = {0};

    for (int i = 0; i < NUM_RELAYS; i++)
    {
        cycleStarts[i] = slowPWM(i, cycleStarts[i], outputs[i]);
    }
}
