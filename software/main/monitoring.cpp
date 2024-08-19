#include "monitoring.h"

ThermalRunawayMonitor::ThermalRunawayMonitor(int historySize)
    : historySize(historySize) {
    initialize();
}

void ThermalRunawayMonitor::initialize() {
    for (int i = 0; i < NUM_SENSORS; ++i) {
        runawayCycles[i] = 0;
        historyIndex[i] = 0;
        for (int j = 0; j < historySize; ++j) {
            temperatureHistory[i][j] = 0.0;
            timeHistory[i][j] = 0;
        }
    }
}

void ThermalRunawayMonitor::addTemperatureReading(int sensorIndex, float temperature, unsigned long timestamp) {
    if (sensorIndex >= 0 && sensorIndex < NUM_SENSORS) {
        historyIndex[sensorIndex] = (historyIndex[sensorIndex] + 1) % historySize;
        temperatureHistory[sensorIndex][historyIndex[sensorIndex]] = temperature;
        timeHistory[sensorIndex][historyIndex[sensorIndex]] = timestamp;
    }
}

bool ThermalRunawayMonitor::checkThermalRunaway(const float setpoints[NUM_SENSORS], const float temps[NUM_SENSORS]) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        float temp = temps[i];
        float setpoint = setpoints[i];

        // Check if the current temperature exceeds the setpoint by more than the delta threshold
        if (temp > setpoint + THRM_RUNAWAY_DELTA) {
            runawayCycles[i]++;
            // Check if the number of cycles has reached the maximum allowed cycles
            if (runawayCycles[i] >= THRM_RUNAWAY_CYCLES) {
                // Set the system to error state
                return true;
            }
        } else {
            // Reset the runaway cycle count if the temperature is within the threshold
            runawayCycles[i] = 0;
        }
    }
    return false;
}
