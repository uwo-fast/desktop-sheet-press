
/* -------------------------------Log Data to SD----------------------------------*/
/**
 *  \brief    Logs data to the SD card.
 *  \remarks  Logs data to the SD card.
 */
void logSD() {
  if (dataFile) { // Check if the file is open
    if (millis() - lastSerialPrint > pData.serialPrintInterval) {

      int errorFlagsMAXSize = sizeof(errorFlagsMAX) / sizeof(errorFlagsMAX[0]);
      for (int i = 0; i < errorFlagsMAXSize; i++) {
        dataFile.print(errorFlagsMAX[i]);
      }
      dataFile.print(',');

      dataFile.print(tempRunAwayAlarm1 ? "WARNING" : "SAFE");
      dataFile.print(',');

      dataFile.print(tempRunAwayAlarm2 ? "WARNING" : "SAFE");
      dataFile.print(',');

      dataFile.print(temp1);
      dataFile.print(',');

      dataFile.print(temp2);
      dataFile.print(',');

      dataFile.print(pData.setTemp);
      dataFile.print(',');

      dataFile.print((int)Output1 < 10 ? "00" : (int)Output1 < 100 ? "0" : "");
      dataFile.print((int)Output1);
      dataFile.print(',');

      dataFile.print((int)Output2 < 10 ? "00" : (int)Output2 < 100 ? "0" : "");
      dataFile.print((int)Output2);
      dataFile.print(',');

      dataFile.print((float)preheatingTime / MILLI_UNIT / 60, 2);
      dataFile.print(',');

      dataFile.print((float)heatingTime / MILLI_UNIT / 60, 2);
      dataFile.print(',');

      dataFile.print(((float)(preheatingTime + heatingTime) / MILLI_UNIT / 60), 2);
      dataFile.print(',');

      dataFile.print((float)pData.heatingDuration / MILLI_UNIT / 60);
      dataFile.print(',');

      dataFile.print((float)pData.kp / MILLI_UNIT);
      dataFile.print(',');

      dataFile.print((float)pData.ki / MILLI_UNIT);
      dataFile.print(',');

      dataFile.print((float)pData.kd / MILLI_UNIT);
      dataFile.print(',');

      dataFile.print(currentProcessState.toString());
      dataFile.print(',');

      dataFile.print(currentActiveProcessSubstate.toString());
      dataFile.println();

      if (millis() - lastFlushTime > flushInterval) {
        dataFile.flush(); // Flush the data to the SD card every flushInterval
        lastFlushTime = millis();
      }
	}
  }
}


void openNewFileIfNeeded()
{
	if (currentFileName == "")
	{
		//Serial.println("Creating new file...");
		currentFileName = createNewFile();
		//Serial.println("New file created:");
    //Serial.println(currentFileName);
		if (currentFileName != "")
		{
			dataFile = SD.open(currentFileName.c_str(), FILE_WRITE);
			if (!dataFile)
			{
				//Serial.println("Error opening file for writing.");
				currentFileName = "";
			}
		}
	}
}

void closeFileIfNeeded()
{
	if (currentFileName != "")
	{
		dataFile.close();
		currentFileName = "";
	}
}

String createNewFile()
{
	
	char fileName[13]; // "data0000.txt" + null terminator
	int fileNumber = 0;

	while (fileNumber < 10000)
	{
		snprintf(fileName, sizeof(fileName), "data%04d.txt", fileNumber);
		if (!SD.exists(fileName))
		{
			File newFile = SD.open(fileName, FILE_WRITE);
			if (newFile)
			{
				//newFile.print("MAX_flags,TRA1_Alarm,TRA2_Alarm,t1,t2,st,o1,o2,Preheat_t,Heat_t,Total_t,dt,Kp,Ki,Kd,State,Substate\n");
				newFile.close();
				return String(fileName);
			}
			else
			{
				// Serial.println("Error creating file.");
				return "";
			}
		}
		fileNumber++;
	}

	// Serial.println("Error: Too many files.");
	return "";
}
