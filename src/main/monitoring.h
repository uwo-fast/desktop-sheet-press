#ifndef MONITORING_H
#define MONITORING_H

#include <Arduino.h>
#include "config.h"

// Thresholds
#define THRM_RUNAWAY_DELTA 20.0
#define THRM_RUNAWAY_TIME 10000 // 10 seconds
#define HISTORY_SIZE (THRM_RUNAWAY_TIME / CONTROL_INTERVAL)
#define NUM_CTRL 2

enum traFlags
{
    TRA_NONE = 0,
    TRA_STARTED = -1,
    TRA_IMPENDING = -2
};

class ThermalRunawayMonitor
{
public:
    ThermalRunawayMonitor(int historySize = HISTORY_SIZE);

    void initialize();
    int updateThermalRunaway(const double setpoints[NUM_CTRL], const double temps[NUM_CTRL]);
    void addTemperatureReading(int sensorIndex, double temperature);

private:
    unsigned long runawayCycles[NUM_CTRL];
    double temperatureHistory[NUM_CTRL][HISTORY_SIZE];
    int historySize;
    int headIndex[NUM_CTRL]; // Tracks the start of the circular buffer for each sensor

    double getTemperatureAt(int sensorIndex, int offset);
};

#endif // MONITORING_H
