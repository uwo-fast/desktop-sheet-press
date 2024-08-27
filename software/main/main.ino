#include "config.h"

#include "StateMachine.h"

#include "temp.h"
#include "monitoring.h"
#include "control.h"
#include "relay.h"

#include "gui.h"

#include <EEPROM.h>

#ifdef SDCARD
#include <SdFat.h>
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 1
SdFat32 sd;
typedef File32 file_t;
file_t file;
#endif // SDCARD

#define SECOND 1000
#define MINUTE 60000

template <typename T>
void setArrayValue(T *array, int index, T value, T minVal, T maxVal, bool &isInvalid, bool clampToLimits)
{
  if (index >= 0 && index < NUM_SENSORS)
  {
    if (value < minVal)
    {
      isInvalid = true;
      array[index] = clampToLimits ? minVal : array[index]; // Clamp to lower limit or reject
    }
    else if (value > maxVal)
    {
      isInvalid = true;
      array[index] = clampToLimits ? maxVal : array[index]; // Clamp to upper limit or reject
    }
    else
    {
      array[index] = value;
    }
  }
  else
  {
    isInvalid = true; // Index out of range
  }
}

struct ProgramData
{
  unsigned long setDuration;
  unsigned long remainingDuration;
  float setDurationMinutes;
  float remainingDurationMinutes;
  double setpoint[NUM_SENSORS];
  double Kp[NUM_SENSORS];
  double Ki[NUM_SENSORS];
  double Kd[NUM_SENSORS];
  double Cp[NUM_SENSORS];
  double Ci[NUM_SENSORS];
  double Cd[NUM_SENSORS];
  double gapThreshold[NUM_SENSORS];
  int fileCount;
  bool sdActive;
  int traStatus;
  bool isInvalid = false;
  int16_t incr;

  // Getters
  unsigned long getSetDuration() const { return setDuration; }
  unsigned long getRemainingDuration() const { return remainingDuration; }
  float getSetDurationMinutes() const { return setDuration / MINUTE; }
  double getSetpoint(int index) const { return setpoint[index]; }
  double getKp(int index) const { return Kp[index]; }
  double getKi(int index) const { return Ki[index]; }
  double getKd(int index) const { return Kd[index]; }
  double getCp(int index) const { return Cp[index]; }
  double getCi(int index) const { return Ci[index]; }
  double getCd(int index) const { return Cd[index]; }
  double getGapThreshold(int index) const { return gapThreshold[index]; }
  int getFileCount() const { return fileCount; }
  bool isSdActive() const { return sdActive; }
  int getTraStatus() const { return traStatus; }
  bool getIsInvalid() const { return isInvalid; }

  // Setters for array elements using the template with clamping option
  void setSetpoint(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(setpoint, index, value, static_cast<double>(MIN_TEMP), static_cast<double>(MAX_TEMP), isInvalid, clampToLimits);
  }

  void setKp(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(Kp, index, value, static_cast<double>(MIN_KP), static_cast<double>(MAX_KP), isInvalid, clampToLimits);
  }

  void setKi(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(Ki, index, value, static_cast<double>(MIN_KI), static_cast<double>(MAX_KI), isInvalid, clampToLimits);
  }

  void setKd(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(Kd, index, value, static_cast<double>(MIN_KD), static_cast<double>(MAX_KD), isInvalid, clampToLimits);
  }

  void setCp(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(Cp, index, value, static_cast<double>(MIN_CP), static_cast<double>(MAX_CP), isInvalid, clampToLimits);
  }

  void setCi(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(Ci, index, value, static_cast<double>(MIN_CI), static_cast<double>(MAX_CI), isInvalid, clampToLimits);
  }

  void setCd(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(Cd, index, value, static_cast<double>(MIN_CD), static_cast<double>(MAX_CD), isInvalid, clampToLimits);
  }

  void setGapThreshold(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(gapThreshold, index, value, static_cast<double>(MIN_GAP_THRESHOLD), static_cast<double>(MAX_GAP_THRESHOLD), isInvalid, clampToLimits);
  }

  void setSetDuration(unsigned long value, bool clampToLimits = false)
  {
    if (value < MIN_SET_DURATION)
    {
      isInvalid = true;
      setDuration = clampToLimits ? MIN_SET_DURATION : setDuration; // Clamp to lower limit or reject
    }
    else if (value > MAX_SET_DURATION)
    {
      isInvalid = true;
      setDuration = clampToLimits ? MAX_SET_DURATION : setDuration; // Clamp to upper limit or reject
    }
    else
    {
      setDuration = value; // Valid value within range
    }
    setDurationMinutes = setDuration / MINUTE;
  }

  void setFileCount(int value)
  {
    fileCount = value;
  }

  void setSdActive(bool value) { sdActive = value; }

  void setTraStatus(int value) { traStatus = value; }
};

ProgramData pData;

// Increment functions with intermediate variable using setters
void incrementSetpoint()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    double newValue = pData.getSetpoint(i) + pData.incr;
    pData.setSetpoint(i, newValue);
    setPIDPoint(i, pData.getSetpoint(i));
  } 
}

void incrementDuration()
{
  unsigned long newValue = pData.getSetDuration() + (pData.incr * MINUTE);
  pData.setSetDuration(newValue);
}

void incrementKp()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    double newValue = pData.getKp(i) + pData.incr;
    pData.setKp(i, newValue);
    setPIDTuning(i, pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }
}

void incrementKi()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    double newValue = pData.getKi(i) + pData.incr;
    pData.setKi(i, newValue);
    setPIDTuning(i, pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }
}

void incrementKd()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    double newValue = pData.getKd(i) + pData.incr;
    pData.setKd(i, newValue);
    setPIDTuning(i, pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }
}

struct CountTimers
{
  unsigned long elapsed;
  unsigned long durationRemaining;
};

struct PointInTime
{
  unsigned long preStart;
  unsigned long heatStart;
};

struct LastUpdateTime
{
  unsigned long master;
  unsigned long control;
  unsigned long gui;
  unsigned long log;
};

struct Timing
{
  CountTimers ct;
  PointInTime pit;
  LastUpdateTime lut;
};

Timing timing = {0};

StateMachine machine = StateMachine();

State *standbyState = machine.addState(&standby, "standby");
State *prepState = machine.addState(&preparing, "preheat");
State *activeState = machine.addState(&active, "heating");
State *termState = machine.addState(&terminating, "cooling");
State *errorState = machine.addState(&error, "error");

enum MachineState
{
  STANDBY,
  PREPARING,
  ACTIVE,
  TERMINATING,
  ERROR,
};

MachineState currMState = STANDBY;

#define SET_STATE(state) (currMState = (state))

#define SET_STANDBY() SET_STATE(STANDBY)
#define SET_PREPARING() SET_STATE(PREPARING)
#define SET_ACTIVE() SET_STATE(ACTIVE)
#define SET_TERMINATING() SET_STATE(TERMINATING)
#define SET_ERROR() SET_STATE(ERROR)

ThermalRunawayMonitor monitor;

// Blank function, it is attached to the lines so that they become focusable.
void startProcess()
{
 if (machine.getCurrentState() == standbyState)
 {
   SET_PREPARING();
 }
}

void skipSubProcess()
{
  if (machine.getCurrentState() == prepState)
  {
    SET_ACTIVE();
  }
  else if (machine.getCurrentState() == activeState)
  {
    SET_TERMINATING();
  }
}

void returnToStandby()
{
  SET_STANDBY();
}

char currStateName[8] = "standby";

LiquidLine tempsLine(1, 0, "T:", tempData.temperatures[0], ",", tempData.temperatures[1]);
LiquidLine setpointLine(10, 0, " S:", pData.setpoint[0]);
LiquidLine outputsLine(1, 1, "O:", controlData.outputs[0], ",", controlData.outputs[1]);
LiquidLine remDurationLine(10, 1, " R:", pData.remainingDurationMinutes);
LiquidScreen screenMain(tempsLine, setpointLine, outputsLine, remDurationLine);

LiquidLine backLine(1, 0, "Back ^");
LiquidLine currStateLine(1, 1, "State: ", currStateName);
LiquidLine setTempLine(1, 2, "ST:  ", pData.setpoint[0]);
LiquidLine setDurationLine(1, 3, "D:   ", pData.setDurationMinutes);
LiquidLine setKpLine(1, 4, "Kp:  ", pData.Kp[0]);
LiquidLine setKiLine(1, 5, "Ki:  ", pData.Ki[0]);
LiquidLine setKdLine(1, 6, "Kd:  ", pData.Kd[0]);

LiquidScreen screenOptions(backLine);

LiquidLine error1(0, 0, "Error, see log.");
LiquidLine error2(0, 1, "Reset Device...");
LiquidScreen screenError(error1, error2);

void setup()
{
  Serial.begin(115200);

  loadEeprom();

#ifdef SDCARD
  if (!sd.begin(SD_CS))
  {
    Serial.println(F("SD initialization failed!"));
    Serial.println(F("Card inserted correctly?"));
    Serial.println(F("Card formatted as FAT16/FAT32?"));
    Serial.println(F("CS pin correct?"));
    Serial.println(F("Wiring correct?"));
    pData.sdActive = false;
  }
  else
  {
    Serial.println(F("Card successfully initialized."));
    pData.sdActive = true;
  }
#endif // SDCARD

  // Add transitions
  standbyState->addTransition(&toPreheating, prepState);
  standbyState->addTransition(&toError, errorState);

  prepState->addTransition(&toHeating, activeState);
  prepState->addTransition(&toError, errorState);

  activeState->addTransition(&toTerminating, termState);
  activeState->addTransition(&toError, errorState);

  termState->addTransition(&toStandby, standbyState);
  termState->addTransition(&toError, errorState);

  errorState->addTransition(&toStandby, standbyState);

  // Add transition to standby from any state on button hold
  prepState->addTransition(&toStandby, standbyState);
  activeState->addTransition(&toStandby, standbyState);
  errorState->addTransition(&toStandby, standbyState);

  initTCs();
  monitor.initialize();

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    pidControllers[i] = new PID(&input[i], &output[i], &pData.setpoint[i], pData.Kp[i], pData.Ki[i], pData.Kd[i], DIRECT);
    pidControllers[i]->SetMode(AUTOMATIC);
    pidControllers[i]->SetSampleTime(CONTROL_INTERVAL);
    pidControllers[i]->SetOutputLimits(0, 255);
  }

  pinMode(PIN_SSR1, OUTPUT);
  pinMode(PIN_SSR2, OUTPUT);

  initializeEncoder(PIN_ENC_DT, PIN_ENC_CLK, PIN_ENC_SW, ENCSTEPS);
  initializeLCD();

  // Attach functions to lines for options
  backLine.attach_function(FUNC_USE, goToMainScreen);

  currStateLine.attach_function(FUNC_USE, startProcess);
  currStateLine.attach_function(FUNC_SKIP, skipSubProcess);
  currStateLine.attach_function(FUNC_BACK, returnToStandby);

  setTempLine.attach_function(FUNC_INCRT, incrementSetpoint);
  setTempLine.attach_function(FUNC_USE, toggleToggler);

  setDurationLine.attach_function(FUNC_INCRT, incrementDuration);
  setDurationLine.attach_function(FUNC_USE, toggleToggler);

  setKpLine.attach_function(FUNC_INCRT, incrementKp);
  setKpLine.attach_function(FUNC_USE, toggleToggler);

  setKiLine.attach_function(FUNC_INCRT, incrementKi);
  setKiLine.attach_function(FUNC_USE, toggleToggler);

  setKdLine.attach_function(FUNC_INCRT, incrementKd);
  setKdLine.attach_function(FUNC_USE, toggleToggler); 

  // Set decimal places for main screen
  tempsLine.set_decimalPlaces(0);
  setpointLine.set_decimalPlaces(0);
  outputsLine.set_decimalPlaces(0);
  remDurationLine.set_decimalPlaces(0);

  // Set decimal places for options screen
  setTempLine.set_decimalPlaces(2);
  setDurationLine.set_decimalPlaces(2);
  setKpLine.set_decimalPlaces(2);
  setKiLine.set_decimalPlaces(2);
  setKdLine.set_decimalPlaces(2);

  // Set focus position for options screen
  backLine.set_focusPosition(Position::LEFT);
  currStateLine.set_focusPosition(Position::LEFT);
  setTempLine.set_focusPosition(Position::LEFT);
  setDurationLine.set_focusPosition(Position::LEFT);
  setKpLine.set_focusPosition(Position::LEFT);
  setKiLine.set_focusPosition(Position::LEFT);
  setKdLine.set_focusPosition(Position::LEFT);

  // Add lines to the options screen
  screenOptions.add_line(currStateLine);
  screenOptions.add_line(setTempLine);
  screenOptions.add_line(setDurationLine);
  screenOptions.add_line(setKpLine);
  screenOptions.add_line(setKiLine);
  screenOptions.add_line(setKdLine);
  screenOptions.set_displayLineCount(2);

  initializeLcdGui();

  Serial.println(F("System initialized"));
}

void loop()
{
  machine.run();
}

void updateTiming(State *currentState)
{
  unsigned long lutMasterOld = timing.lut.master;
  timing.lut.master = millis();

  timing.ct.elapsed = timing.lut.master - timing.pit.preStart;
  if (timing.lut.master > 0 && currentState == activeState)
  {
    unsigned long elapsedSinceLastUpdate = timing.lut.master - lutMasterOld;
    timing.ct.durationRemaining = (timing.ct.durationRemaining > elapsedSinceLastUpdate) ? (timing.ct.durationRemaining - elapsedSinceLastUpdate) : 0;
  }
  pData.remainingDuration = timing.ct.durationRemaining;
  pData.remainingDurationMinutes = pData.remainingDuration / MINUTE;
}

void mainProgram()
{
  updateEeprom();

  State *currentState = machine.getCurrentState();

  if (currentState == prepState || currentState == activeState)
  {
    updateTiming(currentState);
  }

#ifdef SERIALCMD
  handleSerialCommands();
#endif

  if (millis() - timing.lut.control >= CONTROL_INTERVAL)
  {
    timing.lut.control = millis();
    tempData = readTemps();
    tempData = processTempData(tempData);
    pData.traStatus = monitor.updateThermalRunaway(pData.setpoint, tempData.temperatures);
    if (currentState == prepState || currentState == activeState)
    {
      controlData = controlLogic(tempData);
    }
    else
    {
      noOutputs(controlData);
    }
    writeRelays(controlData.outputs);
  }

  if (millis() - timing.lut.gui >= GUI_INTERVAL)
  {
    timing.lut.gui = millis();
    strncpy(currStateName, machine.getStateName(), sizeof(currStateName) - 1);
    currStateName[sizeof(currStateName) - 1] = '\0'; // null-termination

    pData.incr = processEncoderEvents();
    updateLcdGui();
  }

  if (millis() - timing.lut.log >= LOG_INTERVAL)
  {
    timing.lut.log = millis();
#ifdef SERIALCMD
    printData(machine.getStateName());
#endif // SERIALCMD
    if (pData.sdActive && (currentState == prepState || currentState == activeState))
    {
#ifdef SDCARD
      logData(machine.getStateName());
#endif
    }
  }
}

// State functions
void standby()
{
  // State entry logic
  if (machine.executeOnce)
  {
    SET_STANDBY();
    timing = {0};
    pData.remainingDuration = pData.remainingDurationMinutes = 0;
    Serial.println(F("Entering standby state..."));
  }

  // Main program logic
  mainProgram();
}

void preparing()
{
  if (machine.executeOnce)
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
    timing.pit.preStart = millis();
    timing.ct.durationRemaining = pData.setDuration;
    Serial.println(F("Timing set."));
  }

  // Main program logic
  mainProgram();
}

void active()
{
  // State entry logic
  if (machine.executeOnce)
  {
    SET_ACTIVE();
    Serial.println(F("Entering active process phase..."));
    timing.pit.heatStart = millis();
    Serial.println(F("Timing set, duration begun."));
  }

  // Main program logic
  mainProgram();
}

void terminating()
{
  // State entry logic
  if (machine.executeOnce)
  {
    currMState = TERMINATING;
    Serial.println(F("Terminating..."));
  }

  // Main program logic
  mainProgram();
}

void error()
{
  // State entry logic
  if (machine.executeOnce)
  {
    SET_ERROR();
    Serial.println(F("Error state reached."));
  }

  // Main program logic
  mainProgram();
}

// Transition functions
bool toPreheating()
{
  return currMState == PREPARING;
}

bool toHeating()
{
  // Transition to Heating if the current state is manually set to ACTIVE or,
  // if the criteria is met to exit preeating and enter heating, that is when
  // both temperatures have reached or exceeded their respective setpoints
  return currMState == ACTIVE ||
         (machine.getCurrentState() == prepState && tempData.temperatures[0] >= pData.setpoint[0] &&
          tempData.temperatures[1] >= pData.setpoint[1]);
}

bool toTerminating()
{
  // Transition to Terminating if the current state is manually set to TERMINATING
  // or, if the heating duration has elapsed (durationRemaining reaches 0)
  return currMState == TERMINATING ||
         (machine.getCurrentState() == activeState && timing.ct.durationRemaining == 0);
}

bool toStandby()
{
  // Transition to Standby if the current state is manually set to Standby or,
  // if both temperatures have cooled are below the termination threshold
  return currMState == STANDBY ||
         (machine.getCurrentState() == termState && tempData.temperatures[0] <= TERM_TEMP && tempData.temperatures[1] <= TERM_TEMP);
}

bool toError()
{
  // Transition to Error if the current state is manually set to Error or,
  // if a thermal runaway is detected
  return currMState == ERROR || pData.traStatus > 0;
}

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
 * @param stateName The name of the current state (not used in this implementation).
 */
void logData(const char *stateName)
{
  char fileName[20];
  sprintf(fileName, "log%d.txt", pData.fileCount);
  bool open = file.open(fileName, FILE_WRITE);

  if (open)
  {
    file.print(tempData.temperatures[0], 0);
    file.print(",");
    file.print(tempData.temperatures[1], 0);
    file.print(",");
    file.print(pData.getSetpoint(0), 0);
    file.print(",");
    file.print(controlData.outputs[0], 0);
    file.print(",");
    file.print(controlData.outputs[1], 0);
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

void setDurationSetter(unsigned long newSetDuration)
{
  unsigned long oldSetDuration = pData.setDuration;
  newSetDuration *= MINUTE; // convert from minutes input to internal milliseconds
  if (newSetDuration > MAX_DURATION)
  {
    newSetDuration = MAX_DURATION;
  }
  if (machine.getCurrentState() == prepState || machine.getCurrentState() == activeState)
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
  if (machine.getCurrentState() == activeState)
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

#ifdef SERIALCMD
/**
 * @brief Handles incoming serial commands.
 *
 * This function reads characters from the serial input, builds a command string,
 * and executes the corresponding actions when a full command is received. Supported
 * commands include setting a new setpoint, adjusting the duration, changing the
 * machine state, and resetting EEPROM data.
 */
void handleSerialCommands()
{
  static char received[32] = "";
  static byte idx = 0;

  while (Serial.available() > 0)
  {
    char inChar = (char)Serial.read();
    // when a complete command is received or buffer is full
    if (inChar == '\n' || idx >= sizeof(received) - 1)
    {
      received[idx] = '\0'; // null-terminate the string

      if (strncmp(received, "st=", 3) == 0)
      {
        double newSetpoint = atof(received + 3);
        for (int i = 0; i < NUM_SENSORS; i++)
        {
          pData.setSetpoint(i, newSetpoint);
          setPIDPoint(i, pData.setpoint[i]);
        }
      }
      else if (strncmp(received, "dt=", 3) == 0)
      {
        unsigned long newSetDuration = atof(received + 3);
        setDurationSetter(newSetDuration);
      }
      else if (strncmp(received, "dt+", 3) == 0)
      {
        unsigned long durationDelta = atof(received + 3);
        setDurationSetter(pData.getSetDuration() / MINUTE + durationDelta);
      }
      else if (strncmp(received, "dt-", 3) == 0)
      {
        unsigned long durationDelta = atof(received + 3);
        setDurationSetter(pData.getSetDuration() / MINUTE - durationDelta);
      }
      else if (strncmp(received, "rt=", 3) == 0)
      {
        unsigned long newRemainingDuration = atof(received + 3);
        remainingDurationSetter(newRemainingDuration);
      }
      else if (strncmp(received, "kp=", 3) == 0)
      {
        double newKp = atof(received + 3);
        for (int i = 0; i < NUM_SENSORS; i++)
        {
          pData.setKp(i, newKp);
          setPIDTuning(i, newKp, pData.getKi(i), pData.getKd(i));
        }
      }
      else if (strncmp(received, "ki=", 3) == 0)
      {
        double newKi = atof(received + 3);
        for (int i = 0; i < NUM_SENSORS; i++)
        {
          pData.setKi(i, newKi);
          setPIDTuning(i, pData.getKp(i), newKi, pData.getKd(i));
        }
      }
      else if (strncmp(received, "kd=", 3) == 0)
      {
        double newKd = atof(received + 3);
        for (int i = 0; i < NUM_SENSORS; i++)
        {
          pData.setKd(i, newKd);
          setPIDTuning(i, pData.getKp(i), pData.getKi(i), newKd);
        }
      }
      else if (strcmp(received, "prep") == 0)
      {
        SET_PREPARING();
      }
      else if (strcmp(received, "active") == 0)
      {
        SET_ACTIVE();
      }
      else if (strcmp(received, "term") == 0)
      {
        SET_TERMINATING();
      }
      else if (strcmp(received, "standby") == 0)
      {
        SET_STANDBY();
      }
      else if (strcmp(received, "eeprom=reset") == 0)
      {
        resetEeprom(EE_PARTIAL_RESET);
      }
      else if (strcmp(received, "eeprom=fullreset") == 0)
      {
        resetEeprom(EE_FULL_RESET);
      }

      idx = 0; // clear index
    }
    else
    {
      received[idx++] = inChar;
    }
  }
}

/**
 * @brief Prints the current system state and relevant data to the serial monitor.
 *
 * This function outputs the current state, temperatures, setpoint, control outputs,
 * elapsed time, remaining time, and PID parameters to the serial monitor.
 *
 * @param stateName The name of the current state to be printed.
 */
void printData(const char *stateName)
{
  Serial.print(stateName);
  Serial.print(F(", T1:"));
  Serial.print(tempData.temperatures[0], 2);
  Serial.print(F(", T2:"));
  Serial.print(tempData.temperatures[1], 2);
  Serial.print(F(", ST:"));
  Serial.print(pData.getSetpoint(0), 0);
  if (pData.getTraStatus() == -1)
  {
    Serial.print(F("*"));
  }
  else if (pData.traStatus == -2)
  {
    Serial.print(F("**"));
  }
  Serial.print(F(", O1:"));
  Serial.print(controlData.outputs[0], 0);
  Serial.print(F(", O2:"));
  Serial.print(controlData.outputs[1], 0);
  Serial.print(F(", Elapsed:"));
  Serial.print((double)timing.ct.elapsed / MINUTE, 2);
  Serial.print(F("m, Remaining:"));
  Serial.print((double)timing.ct.durationRemaining / MINUTE, 2);
  Serial.print(F("/"));
  Serial.print((double)pData.getSetDuration() / MINUTE, 2);
  Serial.print(F("m"));
  Serial.print(F(", Kp:"));
  Serial.print(pData.getKp(0), 2);
  Serial.print(F(", Ki:"));
  Serial.print(pData.getKi(0), 2);
  Serial.print(F(", Kd:"));
  Serial.print(pData.getKd(0), 2);
  Serial.println(".");
}
#endif // SERIALCMD

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

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    pData.setSetpoint(i, DEF_SETPOINT, true);
    pData.setKp(i, DEF_KP, true);
    pData.setKi(i, DEF_KI, true);
    pData.setKd(i, DEF_KD, true);
    pData.setCp(i, DEF_CP, true);
    pData.setCi(i, DEF_CI, true);
    pData.setCd(i, DEF_CD, true);
    pData.setGapThreshold(i, DEF_GAP_THRESHOLD, true);
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