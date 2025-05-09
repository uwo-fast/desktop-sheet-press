
#ifdef SDCARD
/**
 * @brief Opens a log file on the SD card.
 *
 * This function generates a filename based on the current file count, attempts to
 * open or create the file on the SD card, and writes the header line if the file
 * is successfully opened. The file is then closed.
 *
 * @return true if the file was successfully opened, false otherwise.
 */
bool openFile()
{
    char fileName[20];
    sprintf(fileName, "log%d.txt", pData.fileCount);

    bool open = file.open(fileName, O_CREAT | O_TRUNC | O_RDWR);
    // Open or create file - truncate existing file.
    if (!open)
    {
        return false;
    }
    else
    {
        // Info header
        file.print("Log interval: ");
        file.print(LOG_INTERVAL);
        file.println("ms");

        // CSV Header
        file.println("Temp1,"
                     "Temp2,"
                     "Setpoint,"
                     "Output1,"
                     "Output2,"
                     "Kp,"
                     "Ki,"
                     "Kd,"
                     "ElapsedTime,"
                     "RemainingTime");
        file.close();
        Serial.print(F("Created file: "));
        Serial.println(fileName);
    }
    return true;
}

/**
 * @brief Logs data to the currently open log file on the SD card.
 *
 * This function logs the current temperature readings, setpoints, control outputs,
 * PID parameters, and elapsed/remaining time to the file. The file is then closed.
 * If the file cannot be opened, an error is printed to the Serial monitor.
 *
 */
void logData()
{
    char fileName[20];
    sprintf(fileName, "log%d.txt", pData.fileCount);
    bool open = file.open(fileName, FILE_WRITE);

    if (open)
    {
        file.print(pData.temp[0], 0);
        file.print(",");
        file.print(pData.temp[1], 0);
        file.print(",");
        file.print(pData.getSetpoint(0), 0);
        file.print(",");
        file.print(pData.output[0], 0);
        file.print(",");
        file.print(pData.output[1], 0);
        file.print(",");
        file.print(pData.getKp(0), 2);
        file.print(",");
        file.print(pData.getKi(0), 2);
        file.print(",");
        file.print(pData.getKd(0), 2);
        file.print(",");
        file.print(timing.ct.elapsed / MINUTE); // Print elapsed time in seconds
        file.print(",");
        file.print(timing.ct.durationRemaining / MINUTE); // Print remaining active time in seconds
        file.println();
        file.close();
    }
    else
    {
        Serial.print(F("* "));
    }
}
#endif // SDCARD
