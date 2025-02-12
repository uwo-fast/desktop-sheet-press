#include "config.h"
#include "profile.h"
#include "monitoring.h"
#include "relay.h"

#include "states.h"

#include <EEPROM.h>

#include <Adafruit_MAX31855.h>
#include <PID_v1.h>

#ifdef GUI
#include "gui.h"
#endif

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
  if (index >= 0 && index < NUM_CTRL)
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
  double temp[NUM_CTRL];       // Temp is the current temperature read by the TCs
  uint8_t tempError[NUM_CTRL]; // TempError is the error flag for the TCs
  double output[NUM_CTRL];     // Output is the control signal for the PID controllers

  unsigned long remainingDuration; // Set duration in milliseconds for active state
  float remainingDurationMinutes;  // Remaining duration in minutes for active state
  double setpoint[NUM_CTRL];       // Setpoint is the target temperature for the PID controllers
  double Kp[NUM_CTRL];             // Kp is the proportional gain the PID controllers
  double Ki[NUM_CTRL];             // Ki is the integral gain the PID controllers
  double Kd[NUM_CTRL];             // Kd is the derivative gain the PID controllers
  int16_t fileCount;               // Number of log files created, used for naming
  bool sdActive;                   // Indicates that the SD card is active
  int8_t traStatus;                // Thermal Runaway status, 0 for normal, -1 for impending, -2 for runaway
  bool isInvalid = false;          // Indicates that the last operation was invalid
  int16_t incr;                    // Increment value for encoder, used for setting values in GUI
                                   // for setpoint, Kp, Ki, Kd and other parameters dependent on NUM_CTRL this leads
                                   // to them being set to the same value for all control loops in the current implementation
  // Getters
  unsigned long getRemainingDuration() const
  {
    return remainingDuration;
  }
  float getRemainingDurationMinutes() const
  {
    return remainingDuration / MINUTE;
  }
  double getSetpoint(int index) const
  {
    return setpoint[index];
  }
  double getKp(int index) const
  {
    return Kp[index];
  }
  double getKi(int index) const
  {
    return Ki[index];
  }
  double getKd(int index) const
  {
    return Kd[index];
  }
  int16_t getFileCount() const
  {
    return fileCount;
  }
  bool isSdActive() const
  {
    return sdActive;
  }
  int8_t getTraStatus() const
  {
    return traStatus;
  }
  bool getIsInvalid() const
  {
    return isInvalid;
  }

  // Setters for array elements using the template with clamping option
  void setSetpoint(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(setpoint, index, value, static_cast<double>(MIN_TEMP), static_cast<double>(MAX_TEMP), isInvalid, clampToLimits);
  }

  void setRemainingDuration(unsigned long value, bool clampToLimits = false)
  {
    if (value < MIN_DURATION)
    {
      isInvalid = true;
      remainingDuration = clampToLimits ? MIN_DURATION : remainingDuration; // Clamp to lower limit or reject
    }
    else if (value > MAX_DURATION)
    {
      isInvalid = true;
      remainingDuration = clampToLimits ? MAX_DURATION : remainingDuration; // Clamp to upper limit or reject
    }
    else
    {
      remainingDuration = value; // Valid value within range
    }
    remainingDurationMinutes = remainingDuration / MINUTE;
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

  void setFileCount(int16_t value)
  {
    fileCount = value;
  }

  void setSdActive(bool value)
  {
    sdActive = value;
  }

  void setTraStatus(int8_t value)
  {
    traStatus = value;
  }
};

ProgramData pData;
// Initialize the thermocouples
Adafruit_MAX31855 thermocouples[NUM_CTRL] = {
    Adafruit_MAX31855(PIN_TC_CLK, PIN_TC_CS1, PIN_TC_DO),
    Adafruit_MAX31855(PIN_TC_CLK, PIN_TC_CS2, PIN_TC_DO)};

// PID
PID pidControllers[NUM_CTRL] = {PID(&pData.temp[0], &pData.output[0], &pData.setpoint[0], DEF_KP, DEF_KI, DEF_KD, P_ON_M, DIRECT),
                                PID(&pData.temp[1], &pData.output[1], &pData.setpoint[1], DEF_KP, DEF_KI, DEF_KD, P_ON_M, DIRECT)};

// Relay pins
const int relayPins[NUM_CTRL] = {PIN_SSR1, PIN_SSR2};
struct PointInTime
{
  unsigned long prepStart;    // Time when the process starts preparing the initial conditions
  unsigned long activeStart;  // Time when the process becomes active
  unsigned long currSubStart; // Time when the current subphase of the active process started
  unsigned long termStart;    // Time when the process starts terminating
};

struct LastUpdateTime
{
  unsigned long master;
  unsigned long control;
  unsigned long gui;
  unsigned long log;
};

PointInTime timePoint = {0};     // Recording of points in time for the process
LastUpdateTime lastUpdate = {0}; // Last update time for time tracking in the program

const char standbyStr[] PROGMEM = "standby";
const char preparingStr[] PROGMEM = "preparing";
const char activeStr[] PROGMEM = "active";
const char terminatingStr[] PROGMEM = "terminating";
const char errorStr[] PROGMEM = "error";
const char sudoStr[] PROGMEM = "sudo";

const char *const MachineStateNames[MACHINE_STATE_COUNT] PROGMEM = {
    standbyStr,
    preparingStr,
    activeStr,
    terminatingStr,
    errorStr,
    sudoStr};

const char *getMachineStateName(MachineState state)
{
  static char buffer[16]; // Adjust size as needed
  if (state >= 0 && state < MACHINE_STATE_COUNT)
  {
    strcpy_P(buffer, (char *)pgm_read_word(&(MachineStateNames[state])));
    return buffer;
  }
  else
  {
    return "unknown";
  }
}

MachineState currMState = STANDBY;
MachineState prevMState = MACHINE_STATE_COUNT; // Initialize to an invalid state

ThermalRunawayMonitor monitor;

#ifdef GUI
// Functions to wrap the state transitions for GUI
void startProcess()
{
  if (currMState == STANDBY)
  {
    SET_PREPARING();
  }
}

void skipProcessPhase()
{
  if (currMState == PREPARING)
  {
    SET_ACTIVE();
  }
  else if (currMState == ACTIVE)
  {
    SET_TERMINATING();
  }
}

void returnToStandby()
{
  SET_STANDBY();
}

// Increment functions with intermediate variable using setters for encoder
void incrementSetpoint()
{
  for (int i = 0; i < NUM_CTRL; i++)
  {
    double newValue = pData.getSetpoint(i) + pData.incr;
    pData.setSetpoint(i, newValue);
  }
}

void incrementKp()
{
  for (int i = 0; i < NUM_CTRL; i++)
  {
    double newValue = pData.getKp(i) + pData.incr;
    pData.setKp(i, newValue);
    pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }
}

void incrementKi()
{
  for (int i = 0; i < NUM_CTRL; i++)
  {
    double newValue = pData.getKi(i) + pData.incr;
    pData.setKi(i, newValue);
    pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }
}

void incrementKd()
{
  for (int i = 0; i < NUM_CTRL; i++)
  {
    double newValue = pData.getKd(i) + pData.incr;
    pData.setKd(i, newValue);
    pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }
}

char currStateName[16] = "standby";

bool reverseEncoder = false;

LiquidLine tempsLine(1, 0, "T:", pData.temp[0], ",", pData.temp[1]);
LiquidLine setpointLine(10, 0, " S:", pData.setpoint[0]);
LiquidLine outputsLine(1, 1, "O:", pData.output[0], ",", pData.output[1]);
LiquidLine remDurationLine(10, 1, " R:", pData.remainingDurationMinutes);
LiquidScreen screenMain(tempsLine, setpointLine, outputsLine, remDurationLine);

LiquidLine backLine(1, 0, "Back ^");
LiquidLine currStateLine(1, 1, "State: ", currStateName);
LiquidLine setTempLine(1, 2, "ST:  ", pData.setpoint[0]);
LiquidLine durationLine(1, 3, "R:   ", pData.remainingDurationMinutes);
LiquidLine setKpLine(1, 4, "Kp:  ", pData.Kp[0]);
LiquidLine setKiLine(1, 5, "Ki:  ", pData.Ki[0]);
LiquidLine setKdLine(1, 6, "Kd:  ", pData.Kd[0]);

LiquidScreen screenOptions(backLine);

LiquidLine error1(0, 0, "Error, see log.");
LiquidLine error2(0, 1, "Reset Device...");
LiquidScreen screenError(error1, error2);
#endif

void setup()
{
  Serial.begin(115200);

  loadEeprom();

#ifdef REVERSE_ENCODER
  reverseEncoder = true;
#endif

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

  for (int i = 0; i < NUM_CTRL; i++)
  {
    // Initialize thermocouples
    thermocouples[i].begin();

    // Initialize PID controllers
    pidControllers[i].SetMode(AUTOMATIC);
    pidControllers[i].SetOutputLimits(0, 255);
    pidControllers[i].SetSampleTime(PID_INTERVAL);
    pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }

  pinMode(relayPins[0], OUTPUT);
  pinMode(relayPins[1], OUTPUT);

  monitor.initialize();

#ifdef GUI
  initializeEncoder(PIN_ENC_DT, PIN_ENC_CLK, PIN_ENC_SW, ENC_STEPS, ENC_PULLUP, ENC_INTERRUP_MS);
  initializeLCD();

  // Attach functions to lines for options
  backLine.attach_function(FUNC_USE, goToMainScreen);

  currStateLine.attach_function(FUNC_USE, startProcess);
  currStateLine.attach_function(FUNC_SKIP, skipProcessPhase);
  currStateLine.attach_function(FUNC_BACK, returnToStandby);

  setTempLine.attach_function(FUNC_INCRT, incrementSetpoint);
  setTempLine.attach_function(FUNC_USE, toggleToggler);

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
  durationLine.set_decimalPlaces(2);
  setKpLine.set_decimalPlaces(2);
  setKiLine.set_decimalPlaces(2);
  setKdLine.set_decimalPlaces(2);

  // Set focus position for options screen
  backLine.set_focusPosition(Position::LEFT);
  currStateLine.set_focusPosition(Position::LEFT);
  setTempLine.set_focusPosition(Position::LEFT);
  durationLine.set_focusPosition(Position::LEFT);
  setKpLine.set_focusPosition(Position::LEFT);
  setKiLine.set_focusPosition(Position::LEFT);
  setKdLine.set_focusPosition(Position::LEFT);

  // Add lines to the options screen
  screenOptions.add_line(currStateLine);
  screenOptions.add_line(setTempLine);
  screenOptions.add_line(durationLine);
  screenOptions.add_line(setKpLine);
  screenOptions.add_line(setKiLine);
  screenOptions.add_line(setKdLine);
  screenOptions.set_displayLineCount(2);

  initializeLcdGui();
#endif

  if (RESET_EEPROM)
  {
    resetEeprom(EE_PARTIAL_RESET);
  }

  Serial.println(F("System initialized"));
}

// -------------------------------
// ------ Main program loop ------
// -------------------------------

int currPhase = 0;
int lastPhase = 0;

// START OF MAIN LOOP
void loop()
{

  // Check for state change and execute state entry functions
  stateEntrySwitch();

  pData.temp[1] = pData.temp[0]; // For testing purposes

  // -----------------
  // CONTROL LOGIC
  // -----------------

  // Handle tasks related to the process
  if (currMState == PREPARING || currMState == ACTIVE)
  {

    // Compute the PID output
    // First is the routine to check for NaN in the PID output and safeguard against it
    for (int i = 0; i < NUM_CTRL; i++)
    {
      // This is to stop nan from propogating in event of a TC error
      // See 'Safeguard against NaN' in docs for more info
      if (isnan(pData.output[i]))
      {
        pidControllers[i].SetMode(MANUAL);
        pData.output[i] = 0;
        pidControllers[i].SetMode(AUTOMATIC);
      }

      // Compute the PID output
      if (currMState == PREPARING || currMState == ACTIVE)
        pidControllers[i].Compute();
    }

    // Save the previous master time for elapsed time calculation
    unsigned long lutMasterPrev = lastUpdate.master;

    // Update the master time
    lastUpdate.master = millis();

    // Calculate the elapsed time since the last update
    unsigned long elapsedSinceLastUpdate = lastUpdate.master - lutMasterPrev;

    // Update the temperature profile, if we're in preparation state we want to set it to the initial temperature i.e. the temp of the first step
    double currentSetpoint;
    if (currMState == PREPARING)
    {
      currentSetpoint = tempProfile.steps[0].targetTemp;
    }
    else
    {
      currentSetpoint = updateTempProfileStep(&tempProfile, elapsedSinceLastUpdate);
    }

    // eventually make this  to individual control loops
    for (int i = 0; i < NUM_CTRL; i++)
    {
      // pass the fetched value
      pData.setSetpoint(i, currentSetpoint, true);
    }

    //----------------------
    //--------temp----------
    currPhase = tempProfile.currentStep;

    if (currPhase != lastPhase)
    {
      Serial.print(F("Phase changed to: "));
      Serial.println(currPhase);
      lastPhase = currPhase;
    }
    //---------------------
    //---------------------

    // Retrieve and set the remaining duration for the current step; this can probably be cleaned up by adding getter (profile) / setter (main) function for remaining duration, etc.
    pData.remainingDuration = tempProfile.remainingDuration;

    // -----------------
    // Transition Logic
    // -----------------

    if (currMState == PREPARING)
    {
      // check if we've reached the initial temperature
      if (pData.temp[0] >= pData.setpoint[0] && pData.temp[1] >= pData.setpoint[1])
      {
        SET_ACTIVE();
      }
    }

    if (currMState == ACTIVE)
    {
      // check if we've reached the end of the process
      if (pData.setpoint[0] == -1 && pData.setpoint[1] == -1)
      {
        SET_TERMINATING();
      }
    }
  }
  else if (currMState == SUDO)
  {
    // If in sudo state, allow the output to be set manually
  }
  else // If not in a process state, turn off the relays
  {
    for (int i = 0; i < NUM_CTRL; i++)
    {
      pData.output[i] = 0;
    }
  }

  // -----------------
  // CONTROL ACTIONS
  // -----------------

  // Update control every CONTROL_INTERVAL milliseconds
  if (millis() - lastUpdate.control >= CONTROL_INTERVAL)
  {
    // Update the control timing
    lastUpdate.control = millis();
    readTemps();
    pData.traStatus = monitor.updateThermalRunaway(pData.setpoint, pData.temp);

    writeRelays(pData.output, relayPins);
  }

  // ------------------
  // SYSTEM TASKS
  // ------------------

  // Update the EEPROM data every EEPROM_UPDATE_T milliseconds
  updateEeprom();

// Handle serial commands if enabled
#ifdef SERIALCMD
  handleSerialCommands();
#endif

#ifdef GUI
  // Update GUI every GUI_INTERVAL milliseconds
  if (millis() - lastUpdate.gui >= GUI_INTERVAL)
  {
    lastUpdate.gui = millis();
    strncpy(currStateName, getMachineStateName(currMState), sizeof(currStateName) - 1);
    currStateName[sizeof(currStateName) - 1] = '\0'; // null-termination

    pData.incr = processEncoderEvents(reverseEncoder);
    updateLcdGui();
  }
#endif

  // Log data every LOG_INTERVAL milliseconds
  if (millis() - lastUpdate.log >= LOG_INTERVAL)
  {
    lastUpdate.log = millis();

// Always print the serial commands if enabled
#ifdef SERIALCMD
    printData();
#endif // SERIALCMD

// Only log data if the SD card is active and the machine is in a process state including terminating
#ifdef SDCARD
    if (pData.sdActive && (currMState == PREPARING || currMState == ACTIVE || currMState == TERMINATING || currMState == SUDO))
    {
      logData();
    }
  }
#endif
} // END OF MAIN LOOP

void stateEntrySwitch()
{

  if (currMState != prevMState)
  {

    switch (currMState)
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

    case ERROR_FAILSAFE:
      enterErrorState();
      break;

    case SUDO:
      enterSudoState();
      break;

    default:
      Serial.println(F("Unknown state encountered!"));
      break;
    }
    prevMState = currMState;
  }
}

// State entry functions
void enterStandbyState()
{
  SET_STANDBY();

  for (int i = 0; i < NUM_CTRL; i++)
    pidControllers[i].SetMode(MANUAL);

  timePoint = {0};
  lastUpdate = {0};
  pData.remainingDuration = pData.remainingDurationMinutes = 0;
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

  // eventually make this  to individual control loops
  for (int i = 0; i < NUM_CTRL; i++)
  {
    // pass the fetched value
    pData.setSetpoint(i, 0, true);
  }

  for (int i = 0; i < NUM_CTRL; i++)
    pidControllers[i].SetMode(AUTOMATIC);

  timePoint.prepStart = millis();
  tempProfile.currentStep = 0;
  tempProfile.remainingDuration = tempProfile.steps[0].duration;
  Serial.println(F("Timing set."));
}

void enterActiveState()
{
  SET_ACTIVE();
  Serial.println(F("Entering active process phase..."));
  timePoint.activeStart = millis();
  Serial.println(F("Timing set, duration begun."));
}

void enterTerminatingState()
{
  SET_TERMINATING();

  for (int i = 0; i < NUM_CTRL; i++)
    pidControllers[i].SetMode(MANUAL);

  timePoint.termStart = millis();
  Serial.println(F("Terminating..."));
}

void enterErrorState()
{
  SET_ERROR();
  for (int i = 0; i < NUM_CTRL; i++)
    pidControllers[i].SetMode(MANUAL);
  Serial.println(F("Error state reached."));
}

void enterSudoState()
{
  SET_SUDO();
  for (int i = 0; i < NUM_CTRL; i++)
    pidControllers[i].SetMode(MANUAL);

  pData.output[0] = pData.output[1] = 0;

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

  Serial.println(F("Sudo state reached."));
}

double lastValidTemp[NUM_CTRL] = {0};
uint8_t consecutiveErrors[NUM_CTRL] = {0};
uint8_t consecutiveErrorLimit = 10;

// Temperature functions
void readTemps()
{
  double newTemp[NUM_CTRL] = {0};
  // Read temperatures from the thermocouples
  for (int i = 0; i < NUM_CTRL; i++)
  {
    newTemp[i] = thermocouples[i].readCelsius();
    pData.tempError[i] = thermocouples[i].readError();
  }
  // Process the temperature data
  for (int i = 0; i < NUM_CTRL; i++)
  {
    if (newTemp[i] == 0 || pData.tempError[i])
    {
      consecutiveErrors[i]++;
      if (consecutiveErrors[i] >= consecutiveErrorLimit)
      {
        // Set temperature to 0 if error limit is reached
        pData.temp[i] = 0;
      }
      else
      {
        // Replace with last valid temperature if current reading is zero or an error
        pData.temp[i] = lastValidTemp[i];
      }
    }
    else
    {
      // Update last valid temperature
      lastValidTemp[i] = newTemp[i];
      pData.temp[i] = newTemp[i];
      consecutiveErrors[i] = 0; // Reset error count on valid reading
    }
  }
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
    // ADD BACK ELAPSED AND REMAINING TIME
    file.println();
    file.close();
  }
  else
  {
    Serial.print(F("* "));
  }
}
#endif // SDCARD

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
        for (int i = 0; i < NUM_CTRL; i++)
        {
          pData.setSetpoint(i, newSetpoint);
        }
      }
      else if (strncmp(received, "kp=", 3) == 0)
      {
        double newKp = atof(received + 3);
        for (int i = 0; i < NUM_CTRL; i++)
        {
          pData.setKp(i, newKp);
          pidControllers[i].SetTunings(newKp, pData.getKi(i), pData.getKd(i));
        }
      }
      else if (strncmp(received, "ki=", 3) == 0)
      {
        double newKi = atof(received + 3);
        for (int i = 0; i < NUM_CTRL; i++)
        {
          pData.setKi(i, newKi);
          pidControllers[i].SetTunings(pData.getKp(i), newKi, pData.getKd(i));
        }
      }
      else if (strncmp(received, "kd=", 3) == 0)
      {
        double newKd = atof(received + 3);
        for (int i = 0; i < NUM_CTRL; i++)
        {
          pData.setKd(i, newKd);
          pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), newKd);
        }
      }
      else if ((strncmp(received, "o1=", 3) == 0) && (currMState == SUDO))
      {
        double newOutput = atof(received + 3);
        pData.output[0] = newOutput;
      }
      else if ((strncmp(received, "o2=", 3) == 0) && (currMState == SUDO))
      {
        double newOutput = atof(received + 3);
        pData.output[1] = newOutput;
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
      else if (strcmp(received, "sudo") == 0)
      {
        SET_SUDO();
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
 */
void printData()
{
  Serial.print(getMachineStateName(currMState));
  Serial.print(F(", T1:"));
  Serial.print(pData.temp[0], 2);
  Serial.print(F(", T2:"));
  Serial.print(pData.temp[1], 2);
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
  Serial.print(pData.output[0]);
  Serial.print(F(", O2:"));
  Serial.print(pData.output[1]);
  Serial.print(F(", Remaining:"));
  Serial.print(pData.getRemainingDurationMinutes());
  Serial.print(F("m"));
  Serial.print(F(", Kp:"));
  Serial.print(pData.getKp(0), 3);
  Serial.print(F(", Ki:"));
  Serial.print(pData.getKi(0), 3);
  Serial.print(F(", Kd:"));
  Serial.print(pData.getKd(0), 3);
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
  pData.remainingDuration = DEF_HEATING_DURATION;
  pData.fileCount = 0;

  for (int i = 0; i < NUM_CTRL; i++)
  {
    pData.setSetpoint(i, DEF_SETPOINT, true);
    pData.setKp(i, DEF_KP, true);
    pData.setKi(i, DEF_KI, true);
    pData.setKd(i, DEF_KD, true);
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