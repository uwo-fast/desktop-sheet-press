// Structures and objects
progData pData; /**< Program operating data */

ProcessStateWrapper currentProcessState(INACTIVE_PROCESS);

ActiveProcessSubstateWrapper currentActiveProcessSubstate(UNKNOWN);

const char *getCurrentProcessState()
{
	return currentProcessState.toChar();
}

const char *getCurrentActiveProcessSubstate()
{
	return currentActiveProcessSubstate.toChar();
}

const char *getStateInfo()
{
	static char buffer[16];
	unsigned long output = static_cast<unsigned long>(Output1) * 0.5 + static_cast<unsigned long>(Output2) * 0.5;

	if (strcmp(getCurrentProcessState(), "ACTIVE") == 0)
	{
		// Concatenate first two letters, setTemp, "C", and output without spaces
		sprintf(buffer, "%dC %lu", pData.setTemp, output);
	}
	else
	{
		// For other states, just show the first two letters of the state
		sprintf(buffer, "%s", getCurrentProcessState());
	}
	return buffer;
}

const char *getStatusInfo()
{
	static char buffer[10]; // Buffer to hold the formatted string

	if (strcmp(getCurrentProcessState(), "ACTIVE") == 0)
	{
		// Ensure heatingDuration is treated accurately as milliseconds in integer math
		unsigned long heatingDurationMillis = static_cast<unsigned long>(pData.heatingDuration);
		unsigned long remainingTime = heatingDurationMillis - heatingTime; // Remaining time in milliseconds
		unsigned long hours = remainingTime / (3600000UL);				   // 1000 * 60 * 60
		unsigned long minutes = (remainingTime % (3600000UL)) / (60000UL); // (1000 * 60)
		unsigned long percentCompletion = (heatingTime * 100UL) / heatingDurationMillis;

		sprintf(buffer, "%lu%% %02lu:%02lu", percentCompletion, hours, minutes); // Format as "XX% HH:MM"
	}
	else
	{
		strcpy(buffer, "---% --:--"); // Set the string for non-ACTIVE states
	}

	return buffer;
}

const char *getDurationInfo()
{
	static char buffer[10];

	// Ensure heatingDuration is treated accurately as milliseconds in integer math
	unsigned long heatingDurationMillis = static_cast<unsigned long>(pData.heatingDuration);
	unsigned long hours = heatingDurationMillis / (3600000UL);				   // 1000 * 60 * 60
	unsigned long minutes = (heatingDurationMillis % (3600000UL)) / (60000UL); // (1000 * 60)

	sprintf(buffer, "%02lu:%02lu", hours, minutes); // Format as "XX% HH:MM"
	return buffer;
}

const char *getSetTempInfo()
{
	static char buffer[20];

	// Get the current set temperature from pData structure
	unsigned int setTemp = pData.setTemp;

	sprintf(buffer, "Set Temp: %u C", setTemp); // Format as "Set Temp: ### C"
	return buffer;
}

/* -------------------------------Signal Error----------------------------------*/
/**
 *  \brief    Signals an error condition.
 *  \remarks
 *
 */
void signalError() // TODO
{
	// Implement continuous error signaling in LCD corner and serial output
	// on first pass log dumb to SD card and serial error message. record in eeprom
}
