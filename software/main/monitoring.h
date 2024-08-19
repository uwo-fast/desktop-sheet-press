#ifndef MONITORING_H
#define MONITORING_H

#include <Arduino.h>
#include "config.h"

// Thresholds
#define THRM_RUNAWAY_DELTA 5.0  // Example threshold in degrees
#define THRM_RUNAWAY_CYCLES 3   // Number of cycles to consider for thermal runaway
#define DEFAULT_HISTORY_SIZE 10 // Default number of temperature readings to store

class ThermalRunawayMonitor {
public:
    ThermalRunawayMonitor(int historySize = DEFAULT_HISTORY_SIZE);

    void initialize();
    bool checkThermalRunaway(const float setpoints[NUM_SENSORS], const float temps[NUM_SENSORS]);
    void addTemperatureReading(int sensorIndex, float temperature, unsigned long timestamp);

private:
    int runawayCycles[NUM_SENSORS];
    float temperatureHistory[NUM_SENSORS][DEFAULT_HISTORY_SIZE];
    unsigned long timeHistory[NUM_SENSORS][DEFAULT_HISTORY_SIZE];
    int historySize;
    int historyIndex[NUM_SENSORS];
};

#endif // MONITORING_H
