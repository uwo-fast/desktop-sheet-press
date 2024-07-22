#include "sdcard.h"
#include <SD.h>

extern File dataFile;
extern String currentFileName;
extern unsigned long lastFlushTime;
const unsigned long flushInterval = 5000;

void logData(TempData tempData, ControlData controlData, StateMachineInfo stateInfo, SystemHealthInfo systemHealthInfo) {
    // Implement data logging to SD card
    if (dataFile) { // Check if the file is open
        int errorFlagsMAXSize = sizeof(tempData.errorFlags) / sizeof(tempData.errorFlags[0]);
        for (int i = 0; i < errorFlagsMAXSize; i++) {
            dataFile.print(tempData.errorFlags[i]);
        }
        dataFile.print(',');

        dataFile.print(tempData.temperatures[0]); // Add relevant data points
        dataFile.print(',');
        dataFile.print(tempData.temperatures[1]);
        dataFile.print(',');
        dataFile.print(controlData.relayStates[0]);
        dataFile.print(',');
        dataFile.print(controlData.relayStates[1]);
        dataFile.println();

        if (millis() - lastFlushTime > flushInterval) {
            dataFile.flush(); // Flush the data to the SD card every flushInterval
            lastFlushTime = millis();
        }
    }
}
