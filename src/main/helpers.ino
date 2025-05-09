


void stateEntrySwitch(MachineState newState)
{
  switch (newState)
  {
  case STANDBY:
    enterStandbyState();
    break;

  case PREPARING:
    enterPreparingState();
    break;

  case ACTIVE:
    enterActiveState();
    break;

  case TERMINATING:
    enterTerminatingState();
    break;

  case ERROR_STATE:
    enterErrorState();
    break;

  case SUDO:
    enterSudoState();
    break;

  default:
    Serial.println(F("Unknown state encountered!"));
    break;
  }
}

// State entry functions
void enterStandbyState()
{
  SET_STANDBY();
  for (int i = 0; i < 2; i++)
    pidControllers[i].SetMode(MANUAL);
  timing = {0};
  pData.remainingDurationMillis = pData.remainingDurationMinutes = 0;
  Serial.println(F("Entering standby state..."));
}

void enterPreparingState()
{
  SET_PREPARING();
#ifdef SDCARD
  Serial.println(F("Process beginning, machine preparing..."));
  if (pData.sdActive)
  {
    Serial.println(F("Creating log file..."));
    pData.fileCount++;
    if (!openFile())
    {
      Serial.println(F("Failed to create new log file!"));
    }
  }
#endif
  for (int i = 0; i < 2; i++)
    pidControllers[i].SetMode(AUTOMATIC);
  timing.pit.preStart = millis();
  timing.ct.durationRemaining = pData.setDuration;
  Serial.println(F("Timing set."));
}

void enterActiveState()
{
  SET_ACTIVE();
  Serial.println(F("Entering active process phase..."));
  timing.pit.heatStart = millis();
  Serial.println(F("Timing set, duration begun."));
}

void enterTerminatingState()
{
  SET_TERMINATING();
  Serial.println(F("Terminating..."));
}

void enterErrorState()
{
  SET_ERROR();
  for (int i = 0; i < 2; i++)
    pidControllers[i].SetMode(MANUAL);
  Serial.println(F("Error state reached."));
}

void enterSudoState()
{
  SET_SUDO();
  for (int i = 0; i < 2; i++)
    pidControllers[i].SetMode(MANUAL);
  Serial.println(F("Sudo state reached."));
}


void setDurationSetter(unsigned long newSetDuration)
{
  unsigned long oldSetDuration = pData.setDuration;
  newSetDuration *= MINUTE; // convert from minutes input to internal milliseconds
  if (newSetDuration > MAX_DURATION)
  {
    newSetDuration = MAX_DURATION;
  }
  if (currMState == PREPARING || currMState == ACTIVE)
  {
    unsigned long deltaDuration = newSetDuration - oldSetDuration;
    if (timing.ct.durationRemaining + deltaDuration >= 0)
    {
      timing.ct.durationRemaining += deltaDuration;
    }
    else
    {
      timing.ct.durationRemaining = 0;
    }
  }
  pData.setDuration = newSetDuration;
}

void remainingDurationSetter(unsigned long newRemainingDuration)
{
  unsigned long oldRemainingDuration = timing.ct.durationRemaining;
  newRemainingDuration *= MINUTE; // convert from minutes input to internal milliseconds
  if (currMState == ACTIVE)
  {
    if (newRemainingDuration >= 0)
    {
      timing.ct.durationRemaining = newRemainingDuration;
    }
    else
    {
      timing.ct.durationRemaining = 0;
    }
    unsigned long deltaDuration = newRemainingDuration - oldRemainingDuration;
    pData.setDuration += deltaDuration;
  }
}

/**
 * The functions below are vitally important
 * for understanding the EEPROM and program
 * variable initialization.
 */

/**
 * @brief Initializes default program data.
 *
 * This function is called to initialize the program data with default values. It includes
 * a mechanism to determine whether the initialization should be partial or full. A partial
 * initialization only resets the program data, while a full initialization also resets
 * historical data and other program-related values.
 *
 * The function sets default values for all program data, including the active duration,
 * file count, setpoints, PID coefficients, control parameters, and gap thresholds. It also
 * initializes historical data, such as machine usage statistics, if a full reset is requested.
 *
 * @param data A reference to the ProgramData structure to be initialized.
 * @param full A boolean value indicating whether the initialization should be full or partial.
 */

void initializeDefaultProgramData(boolean full)
{
  // Set default values for all non full data related to the program
  pData.setDuration = DEF_HEATING_DURATION;
  pData.fileCount = 0;

  for (int i = 0; i < 2; i++)
  {
    pData.setSetpoint(i, DEF_SETPOINT, true);
    pData.setKp(i, DEF_KP, true);
    pData.setKi(i, DEF_KI, true);
    pData.setKd(i, DEF_KD, true);
    pData.setCp(i, DEF_CP, true);
    pData.setCi(i, DEF_CI, true);
    pData.setCd(i, DEF_CD, true);
    pData.setAdaptiveTempGap(i, DEF_ADAPTIVE_TEMP_GAP, true);
    pData.setSdActive(false);
    pData.setTraStatus(TRA_NONE);
  }

  // If full reset, reset all values including the historical ones
  // that will be added below such as the machine usage stats
  // Set default values for ALL data related to the program
  if (full)
  {
    // Reset all historical values too
  }
}
