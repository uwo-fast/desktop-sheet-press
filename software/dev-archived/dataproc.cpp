#include "dataproc.h"
#include "Arduino.h"

// Static variable to store the last valid data
static TempData lastValidData = {};

// Function to process temperature data
TempData processTempData(const TempData& newData) {
    TempData processedData = newData;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (newData.temperatures[i] == 0 || newData.errorFlags[i]) {
            // Replace with last valid temperature if current reading is zero or an error
            processedData.temperatures[i] = lastValidData.temperatures[i];
        } else {
            // Update last valid temperature
            lastValidData.temperatures[i] = newData.temperatures[i];
            lastValidData.errorFlags[i] = newData.errorFlags[i];
        }
    }

    return processedData;
}
