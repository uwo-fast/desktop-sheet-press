Adafruit_MAX31855 thermocouple1(PIN_TC_CLK, PIN_TC_CS1, PIN_TC_DO); /**< Thermocouple 1 object */
Adafruit_MAX31855 thermocouple2(PIN_TC_CLK, PIN_TC_CS2, PIN_TC_DO); /**< Thermocouple 2 object */


/* -------------------------------Read and Check Temperature----------------------------------*/
/**
 *  \brief    Reads the temperature from the thermocouples and checks for errors.
 *  \remarks
 *  \todo implement a function to compute and report on temperature readings error ratio
 */
void readCheckTemp()
{
	// TEMP 1
	double temp1Read = thermocouple1.readCelsius();
	temp1 = (int16_t)temp1Read;

	// Read error flags from thermocouple 1 and generate error code
	uint8_t e1 = thermocouple1.readError();
	errorFlagsMAX[0] = (e1 & MAX31855_FAULT_OPEN) ? 1 : 0;
	errorFlagsMAX[1] = (e1 & MAX31855_FAULT_SHORT_GND) ? 1 : 0;
	errorFlagsMAX[2] = (e1 & MAX31855_FAULT_SHORT_VCC) ? 1 : 0;

	// TEMP 2
	double temp2Read = thermocouple2.readCelsius();
	temp2 = (int16_t)temp2Read;

	// Read error flags from thermocouple 2 and generate error code
	uint8_t e2 = thermocouple2.readError();
	errorFlagsMAX[3] = (e2 & MAX31855_FAULT_OPEN) ? 1 : 0;
	errorFlagsMAX[4] = (e2 & MAX31855_FAULT_SHORT_GND) ? 1 : 0;
	errorFlagsMAX[5] = (e2 & MAX31855_FAULT_SHORT_VCC) ? 1 : 0;
}

/* -------------------------------Thermal Runaway Check----------------------------------*/
/**
 *  \brief    Checks for thermal runaway.
 *  \remarks This function checks for thermal runaway by comparing the temperature readings
 *  to the setpoint plus a delta value. If the temperature readings exceed the setpoint plus
 *  the delta value for a certain number of cycles and if the output for that temperature
 *  is not issuing a zero output, a thermal runaway event is issued.
 * 	The drivingDeltaCounter holds how many cycles the temperature has been above the setpoint plus the
 *  delta value while still driving the output. If the temperature is below the setpoint plus the delta
 *  value, the counter is reset to zero. If the counter reaches the number of cycles set in the program
 *  data, a thermal runaway event is issued. drivingDeltaCounter only increases once per control period.
 */
void thermalRunawayCheck() // TODO
{

	unsigned long currentMillis = millis();
	// Check if enough time has passed for the next thermal runaway check
	if (currentMillis - lastThermalRunawayCheck >= pData.controlPeriod)
	{
		// The actual thermal runaway check logic remains the same
		if (temp1 > pData.setTemp + pData.tempRunAwayDelta || temp2 > pData.setTemp + pData.tempRunAwayDelta)
		{
			// Check if the output is not zero
			if ((int)Output1 > 0)
			{
				tempRunAwayAlarm1 = true;
				drivingDeltaCounter1++;
			}

			if ((int)Output2 > 0)
			{
				tempRunAwayAlarm2 = true;
				drivingDeltaCounter2++;
			}
		}
		else
		{
			tempRunAwayAlarm1 = false;
			tempRunAwayAlarm2 = false;
			drivingDeltaCounter1 = 0;
			drivingDeltaCounter2 = 0;
		}

		if (drivingDeltaCounter1 >= pData.tempRunAwayCycles || drivingDeltaCounter2 >= pData.tempRunAwayCycles)
		{
			currentProcessState.setState(ERROR_PROCESS);
			// Signal error condition
			signalError();
		}

		// Update the last check time
		lastThermalRunawayCheck = currentMillis;
	}
}
