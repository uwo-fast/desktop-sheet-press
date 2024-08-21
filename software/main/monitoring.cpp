#include "monitoring.h"

ThermalRunawayMonitor::ThermalRunawayMonitor(int historySize)
    : historySize(historySize)
{
    initialize();
}

void ThermalRunawayMonitor::initialize()
{
    for (int i = 0; i < NUM_SENSORS; ++i)
    {
        runawayCycles[i] = 0;
        headIndex[i] = 0;
        for (int j = 0; j < historySize; ++j)
        {
            temperatureHistory[i][j] = 0.0;
        }
    }
}

void ThermalRunawayMonitor::addTemperatureReading(int sensorIndex, double temperature)
{
    if (sensorIndex >= 0 && sensorIndex < NUM_SENSORS)
    {
        // Insert the new temperature at the head of the circular buffer
        temperatureHistory[sensorIndex][headIndex[sensorIndex]] = temperature;

        // Move the head index forward in the circular buffer
        headIndex[sensorIndex] = (headIndex[sensorIndex] + 1) % historySize;
    }
}

double ThermalRunawayMonitor::getTemperatureAt(int sensorIndex, int offset)
{
    // Calculate the actual index in the circular buffer
    int index = (headIndex[sensorIndex] + offset) % historySize;
    return temperatureHistory[sensorIndex][index];
}

int ThermalRunawayMonitor::updateThermalRunaway(const double setpoints[NUM_SENSORS], const double temps[NUM_SENSORS])
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        // Add new temperature reading
        addTemperatureReading(i, temps[i]);

        // Check if the current temperature exceeds the setpoint by more than the delta threshold
        if (temps[i] > setpoints[i] + THRM_RUNAWAY_DELTA)
        {
            runawayCycles[i]++;
            if (runawayCycles[i] >= historySize / 2 )
            {
                return TRA_IMPENDING; // Return -2 if runaway is impending (past half the history size)
            }
            return TRA_STARTED; // Return -1 if is runaway is started
        }
        else
        {
            runawayCycles[i] = 0; // Reset cycle count if within the threshold
            return TRA_NONE;            // No alarm triggered
        }

        // Check if the number of cycles has reached the history size (indicating runaway)
        if (runawayCycles[i] >= historySize)
        {
            return i + 1; // Return the sensor index (1-based) where the alarm was triggered
        }
    }
}
