
/***************************************************************************************************
 * Utility EEPROM Functions                                                                         *
 ***************************************************************************************************/
/**
 *  \brief                    Reset the EEPROM and program data to factory default settings.
 *  \remarks                  EEPROM data is only written if the new data is different to the
 *                            existing data to limit EEPROM wearout.
 *  \param [in] boolean full  True to reset the weld count, battery offset, and screen inversion.
 */
void resetEeprom(boolean full)
{

	// Write the factory default data to the eeprom. In the case of
	// \todo
	// [vals], these are zeroed, otherwise they are left unchanged.

	pData.tempRunAwayDelta = DEF_TEMP_RUNA_DELTA;
	pData.tempRunAwayCycles = DEF_TEMP_RUNA_CYCLES;
	pData.setTemp = DEF_SET_TEMP;
	pData.controlPeriod = DEF_CONTROL_PERIOD;
	pData.processInterval = DEF_PROCESS_INTERVAL;
	pData.heatingDuration = DEF_HEATING_DURATION;
	pData.preToHeatTempOffset = DEF_PRE_TO_HEAT_TEMP_OFFSET;
	pData.preToHeatHoldTime = DEF_PRE_TO_HEAT_HOLD_TIME;
	pData.serialPrintInterval = DEF_SERIAL_PRINT_INTERVAL;
	pData.kp = DEF_KP;
	pData.ki = DEF_KI;
	pData.kd = DEF_KD;
	pData.cp = DEF_CP;
	pData.ci = DEF_CI;
	pData.cd = DEF_CD;
	pData.gapThreshold = DEF_GAP_THRESHOLD;

	// The put function does not write new data if the existing data is the same thereby
	// limiting eeprom wearout.
	EEPROM.put(EEA_PDATA, pData);

	// The unique id is a simple method to ensure that a valid data set exists in the eeprom
	// (there are much better methods but we don't have the code space to spare).
	EEPROM.put(EEA_ID, EE_UNIQUEID);

#ifdef _SERIALCMD_ || _DEVELOPMENT_

	if (full)
		Serial.print(F("EEPROM Full Reset"));
	else
		Serial.println(F("EEPROM Reset"));
#endif /* _SERIALCMD_ || _DEVELOPMENT_ */
}

/**
 *  \brief    Loads the program data from the EEPROM.
 *  \remarks  The EEPROM data is only read if the unique id is present and correct. If the unique
 *            id is not present or is incorrect then the eeprom is reset to factory default.
 */
void loadEeprom()
{
	// Check the eeprom integrity by reading a magic number. If it is corrupt then the eeprom
	// is given a full factory reset, otherwise program data is loaded from the eeprom.
	uint32_t uniqueID;

	EEPROM.get(EEA_ID, uniqueID);

	// If the unique id is not present (first upload or change of program) or is
	// incorrect (corrupted, falsely overwritten) then the eeprom is reset to factory
	if (uniqueID != EE_UNIQUEID)
		resetEeprom(EE_FULL_RESET);
	else
		EEPROM.get(EEA_PDATA, pData);
}

/**
 *  \brief    Udates the EEPROM data with local program data structure.
 *  \remarks  EEPROM data is only written if the new data is different to the
 *            existing data to limit EEPROM wearout.
 */
void updateEeprom()
{
	static unsigned long lastEEUpdatetime = 0;

	// Do not do this too often to prevent premature eeprom wearout.
	if (millis() - lastEEUpdatetime > EEPROM_UPDATE_T)
	{
		lastEEUpdatetime = millis();

		// Write the current program data to the eeprom.
		EEPROM.put(EEA_PDATA, pData);

#ifdef _DEVELOPMENT_
		Serial.println(F("Updated EEPROM"));
#endif /* _DEVELOPMENT_ */
	}
}
