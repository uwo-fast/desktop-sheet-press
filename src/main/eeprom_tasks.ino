
/**
 * @brief Resets EEPROM to default program values.
 *
 * This function is called to reset the EEPROM to default program values. It includes
 * a mechanism to determine whether the reset should be partial or full. A partial reset
 * only resets the program data, while a full reset also resets historical data and other
 * program-related values.
 *
 * The function writes the default program data to the EEPROM and sets a unique identifier
 * (magic number) at the beginning of the EEPROM to validate the data's integrity. The unique
 * identifier is used to ensure that the EEPROM contains valid data for this specific program.
 *
 * @param full A boolean value indicating whether the reset should be full or partial.
 *
 * @note The unique identifier is defined in the config.h file as EE_UNIQUEID.
 */
void resetEeprom(boolean full)
{
    initializeDefaultProgramData(full);

    EEPROM.put(EEA_PDATA, pData);

    EEPROM.put(EEA_ID, EE_UNIQUEID);

    if (full)
        Serial.println(F("EEPROM Full Reset"));
    else
        Serial.println(F("EEPROM Reset"));
}

/**
 * @brief Loads program data from EEPROM on startup.
 *
 * This function is called during the setup phase of the Arduino sketch to initialize
 * the system with user settings and history stored in EEPROM. It includes a mechanism
 * to determine whether the EEPROM contains valid data for this program.
 *
 * A unique identifier (magic number) is used at the beginning of the EEPROM to verify
 * the data's validity. If the magic number does not match (indicating either a first-time
 * program upload, data corruption, or previous use of the EEPROM by another program),
 * the function resets the EEPROM to default program values.
 *
 * The use of a unique ID helps ensure that the EEPROM contains a valid data set belonging
 * to this specific program. However, it is acknowledged that there are more robust methods
 * for EEPROM data validation, which are not used here due to code space constraints.
 *
 * It's important to note that the reliability of this mechanism is based on the convention
 * of starting EEPROM writes at the beginning of its address space. Deviating from this
 * convention in subsequent programs might lead to a false positive validation of EEPROM
 * data integrity, as the magic number might remain unaltered.
 *
 * @note This function is guaranteed to reset data to defaults only on the first upload
 *       of the program. Subsequent uploads not adhering to the EEPROM writing convention
 *       may inadvertently preserve the unique ID, leading to incorrect data validation.
 */
void loadEeprom()
{
    uint32_t uniqueID;

    EEPROM.get(EEA_ID, uniqueID);

    if (uniqueID != EE_UNIQUEID)
    {
        resetEeprom(EE_FULL_RESET);
    }
    else
    {
        EEPROM.get(EEA_PDATA, pData);
    }
}

/**
 * @brief Updates EEPROM with program data at regular intervals.
 *
 * This function is called at regular intervals to update the EEPROM with the latest
 * program data. The update interval is defined by the constant EEPROM_UPDATE_T.
 *
 * The function is intended to reduce the number of EEPROM writes, which have a limited
 * lifespan, by batching multiple writes into a single update operation. This approach
 * helps to extend the EEPROM's lifespan and reduce the risk of data corruption.
 *
 * @note The EEPROM update interval is defined in the config.h file as EEPROM_UPDATE_T.
 */
void updateEeprom()
{
    static unsigned long lastEEUpdatetime = 0;

    if (millis() - lastEEUpdatetime > EEPROM_UPDATE_T)
    {
        lastEEUpdatetime = millis();
        EEPROM.put(EEA_PDATA, pData);
    }
}