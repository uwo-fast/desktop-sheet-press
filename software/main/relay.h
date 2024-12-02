#ifndef RELAY_H
#define RELAY_H

#define NUM_RELAY 2
#define RELAY_PWM_PERIOD 1000 // 1 Hz (1000 ms)

void writeRelays(const double values[], int pins[]);

#endif
