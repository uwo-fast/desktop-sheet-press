#include "relay.h"
#include <Arduino.h>

// Function to control relays using slow PWM
void slowPWM(int SSRn, unsigned long &cycleStart, double period, double output) {
    // Get current time in milliseconds
    unsigned long currentMillis = millis();

    double dutyCycle = output / 255.0;

    // If within the ON part of the cycle, turn the SSR on
    if (currentMillis - cycleStart < period * dutyCycle) {
        digitalWrite(SSRn, HIGH);
    }
    // If within the OFF part of the cycle, turn the SSR off
    else if (currentMillis - cycleStart >= period * dutyCycle && currentMillis - cycleStart < period) {
        digitalWrite(SSRn, LOW);
    }
    // If the cycle is complete, reset the cycle start time
    else {
        cycleStart = currentMillis;
    }
}

// Wrapper function to write control data to relays
void writeRelays(const ControlData& controlData) {
    static unsigned long cycleStart1 = 0;
    static unsigned long cycleStart2 = 0;

    slowPWM(PIN_SSR1, cycleStart1, 1000, controlData.outputs[0]);  // Example period of 1000ms (1 second)
    slowPWM(PIN_SSR2, cycleStart2, 1000, controlData.outputs[1]);  // Example period of 1000ms (1 second)
}
