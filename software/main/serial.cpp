
// Timing variable for the serial print interval, for serial command mode
#ifdef _SERIALCMD_
unsigned long lastSerialPrint = 0;
#endif /* _SERIALCMD_ */



/* --------------------------------------------------------------------------------------*/
/*------------------------------------- _SERIALCMD_--------------------------------------*/
/* --------------------------------------------------------------------------------------*/
#ifdef _SERIALCMD_ || _DEVELOPMENT_
// Function to allow serial navigation and control
void handleSerialCommands()
{
	static String received = "";
	while (Serial.available() > 0)
	{
		char inChar = (char)Serial.read();
		received += inChar;
		if (inChar == '\n')
		{					 // when a complete command is received
			received.trim(); // remove potential leading/trailing white space
			if (received.startsWith("t1="))
			{
				temp1 = (int16_t)received.substring(3).toInt();
			}
			else if (received.startsWith("t2="))
			{
				temp2 = (int16_t)received.substring(3).toInt();
			}
			else if (received.startsWith("st="))
			{
				pData.setTemp = (uint16_t)received.substring(3).toInt();
			}
			else if (received.startsWith("o1="))
			{
				Output1 = (uint8_t)received.substring(3).toInt();
			}
			else if (received.startsWith("o2="))
			{
				Output2 = (uint8_t)received.substring(3).toInt();
			}
			else if (received.startsWith("dt="))
			{
				pData.heatingDuration = (received.substring(3).toDouble() * MILLI_UNIT * 60);
			}
			else if (received.startsWith("kp="))
			{
				pData.kp = (uint16_t)(received.substring(3).toFloat() * MILLI_UNIT);
			}
			else if (received.startsWith("ki="))
			{
				pData.ki = (uint16_t)(received.substring(3).toFloat() * MILLI_UNIT);
			}
			else if (received.startsWith("kd="))
			{
				pData.kd = (uint16_t)(received.substring(3).toFloat() * MILLI_UNIT);
			}
			else if (received == "ON")
			{
				currentProcessState.setState(ACTIVE_PROCESS);
				currentActiveProcessSubstate.setSubstate(PREHEATING);
			}
			else if (received == "OFF")
			{
				currentProcessState.setState(INACTIVE_PROCESS);
				currentActiveProcessSubstate.setSubstate(UNKNOWN);
			}
			else if (received == "relay1=off")
			{
				relay1ManualOff = true;
			}
			else if (received == "relay1=on")
			{
				relay1ManualOff = false;
			}
			else if (received == "relay2=off")
			{
				relay2ManualOff = true;
			}
			else if (received == "relay2=on")
			{
				relay2ManualOff = false;
			}

			received = ""; // clear received data
		}
	}
}

void printSerialData()
{
	if (millis() - lastSerialPrint > pData.serialPrintInterval)
	{
		Serial.print("MAX flags: ");

		int errorFlagsMAXSize = sizeof(errorFlagsMAX) / sizeof(errorFlagsMAX[0]);
		// Print error flags
		for (int i = 0; i < errorFlagsMAXSize; i++)
		{
			Serial.print(errorFlagsMAX[i]);
		}
		Serial.print(F(", TRA1 Alarm: "));
		Serial.print(tempRunAwayAlarm1 ? "WARNING" : "SAFE");
		Serial.print(", TRA2 Alarm: ");
		Serial.print(tempRunAwayAlarm2 ? "WARNING" : "SAFE");
		Serial.print(F(", t1: "));
		Serial.print(temp1);
		Serial.print(F("C, t2: "));
		Serial.print(temp2);
		Serial.print("C, st: ");
		Serial.print(pData.setTemp);
		Serial.print(F("C, o1: "));
		if (relay1ManualOff)
		{
			Serial.print("off");
		}
		else
		{
			Serial.print((int)Output1 < 10 ? "00" : (int)Output1 < 100 ? "0"
																	   : "");
			Serial.print((int)Output1);
		}
		Serial.print(", o2: ");
		if (relay2ManualOff)
		{
			Serial.print("off");
		}
		else
		{
			Serial.print((int)Output2 < 10 ? "00" : (int)Output2 < 100 ? "0"
																	   : "");
			Serial.print((int)Output2);
		}
		Serial.print(F(", Preheat t: "));
		// \todo instead of floats, use a function to convert to string and shift float point
		Serial.print((float)preheatingTime / MILLI_UNIT / 60, 2);
		Serial.print(F("m, Heat t: "));
		Serial.print((float)heatingTime / MILLI_UNIT / 60, 2);
		Serial.print(F("m, Total t: "));
		Serial.print(((float)(preheatingTime + heatingTime) / MILLI_UNIT / 60), 2); // Total time in minutes
		Serial.print(F("m, dt: "));
		Serial.print((float)pData.heatingDuration / MILLI_UNIT / 60);
		Serial.print(F("m, Kp: "));
		Serial.print((float)pData.kp / MILLI_UNIT);
		Serial.print(F(", Ki: "));
		Serial.print((float)pData.ki / MILLI_UNIT);
		Serial.print(F(", Kd: "));
		Serial.print((float)pData.kd / MILLI_UNIT); // Use println to add newline at the end
		Serial.print(F(", State: "));
		Serial.print(currentProcessState.toString());
		Serial.print(F(", Substate: "));
		Serial.print(currentActiveProcessSubstate.toString());
		Serial.println();

		lastSerialPrint = millis();
	}
}
#endif /* _SERIALCMD_ || _DEVELOPMENT_ */
