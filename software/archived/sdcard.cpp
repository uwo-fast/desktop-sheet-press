
#include "sdcard.h"


// Create a new log file
bool createNewLogFile()
{
    //String fileName = "log" + String(pData.fileCount) + ".txt";
    String fileName = "log" + String(1) + ".txt";
    //pData.fileCount++;
    File logFile = SD.open(fileName, FILE_WRITE);
    if (logFile)
    {
        logFile.close();
        return true;
    }
    else
    {
        return false;
    }
}

// Log data to the current log file
void logData(const char *stateName)
{
    //String fileName = "log" + String(pData.fileCount) + ".txt";
    String fileName = "log" + String(1) + ".txt";

    File logFile = SD.open(fileName, FILE_WRITE);
    if (logFile)
    {
        logFile.print("State:");
        logFile.print(stateName);
        logFile.print(" T1:");
        logFile.print(tempData.temperatures[0], 0);
        logFile.print(", T2:");
        logFile.print(tempData.temperatures[1], 0);
        logFile.print(", ST:");
        logFile.print(pData.setpoint[0], 0); // Assuming both setpoints are the same
        logFile.print(", O1:");
        logFile.print(controlData.outputs[0], 0);
        logFile.print(", O2:");
        logFile.print(controlData.outputs[1], 0);
        logFile.print(", Kp:");
        logFile.print(pData.Kp[0], 2);
        logFile.print(", Ki:");
        logFile.print(pData.Ki[0], 2);
        logFile.print(", Kd:");
        logFile.print(pData.Kd[0], 2);
        logFile.print(", Elapsed Time:");
        logFile.print(timing.ct.elapsed / 1000); // Print elapsed time in seconds
        logFile.print("s, Remaining Heating Time:");
        logFile.print(timing.ct.durationRemaining / 1000); // Print remaining active time in seconds
        logFile.println("s"); 
        logFile.close();
    }
    else
    {
    }
}
