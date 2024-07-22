#include "temp.h"
#include <Adafruit_MAX31855.h>

// Initialize thermocouples
Adafruit_MAX31855 thermocouples[NUM_SENSORS] = {
    Adafruit_MAX31855(PIN_TC_CLK, PIN_TC_CS1, PIN_TC_DO),
    Adafruit_MAX31855(PIN_TC_CLK, PIN_TC_CS2, PIN_TC_DO)
    // Add more thermocouples as needed
};

TempData readTemps() {
    TempData tempData;
    for (int i = 0; i < NUM_SENSORS; i++) {
        double tempRead = thermocouples[i].readCelsius();
        tempData.temperatures[i] = (float)tempRead;
        uint8_t error = thermocouples[i].readError();
        tempData.errorFlags[i] = (error != 0);
    }
    return tempData;
}
