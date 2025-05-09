#include "relay.h"
#include <Arduino.h>

/**
 * @brief Controls a relay using slow PWM.
 *
 * This function uses a software PWM technique to control the relay's duty cycle.
 * @param SSRn The pin number of the relay.
 * @param cycleStart A reference to the cycle start time.
 * @param value The desired duty cycle (0-255).
 */
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

/**
 * @brief Writes values to multiple relays using slow PWM.
 *
 * This function iterates through an array of values and corresponding pin numbers,
 *
 * @param values An array of duty cycle values (0-255) for each relay.
 * @param pins An array of pin numbers corresponding to each relay.
 */
void writeRelays(const double values[], const int pins[])
{
    static unsigned long cycleStarts[NUM_RELAY] = {0};

    for (int i = 0; i < NUM_RELAY; i++)
    {
        cycleStarts[i] = slowPWM(pins[i], cycleStarts[i], values[i]);
    }
}
