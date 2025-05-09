#include "config.h"

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
  if (index >= 0 && index < 2)
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
  double temp[2];       // Temp is the current temperature read by the TCs
  uint8_t tempError[2]; // TempError is the error flag for the TCs
  double output[2];     // Output is the control signal for the PID controllers

  unsigned long setDuration;             // Set duration in milliseconds for active state
  unsigned long remainingDurationMillis; // Remaining duration in milliseconds for active state
  float setDurationMinutes;              // Set duration in minutes for active state
  float remainingDurationMinutes;        // Remaining duration in minutes for active state
  double setpoint[2];                    // Setpoint is the target temperature for the PID controllers
  double Kp[2];                          // Kp is the proportional gain the PID controllers
  double Ki[2];                          // Ki is the integral gain the PID controllers
  double Kd[2];                          // Kd is the derivative gain the PID controllers
  double Cp[2];                          // Cp is the proportional gain for adaptive PID
  double Ci[2];                          // Ci is the integral gain for adaptive PID
  double Cd[2];                          // Cd is the derivative gain for adaptive PID
  double adaptiveTempGap[2];             // Adaptive temperature gap for adaptive PID
  int16_t fileCount;                     // Number of log files created, used for naming
  bool sdActive;                         // Indicates that the SD card is active
  int8_t traStatus;                      // Thermal Runaway status, 0 for normal, -1 for impending, -2 for runaway
  bool isInvalid = false;                // Indicates that the last operation was invalid
  int16_t incr;                          // Increment value for encoder, used for setting values in GUI
                                         // for setpoint, Kp, Ki, Kd and other parameters dependent on 2 this leads
                                         // to them being set to the same value for all control loops in the current implementation
  // Getters
  unsigned long getSetDuration() const
  {
    return setDuration;
  }
  unsigned long getRemainingDuration() const
  {
    return remainingDurationMillis;
  }
  float getSetDurationMinutes() const
  {
    return setDuration / MINUTE;
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
  double getCp(int index) const
  {
    return Cp[index];
  }
  double getCi(int index) const
  {
    return Ci[index];
  }
  double getCd(int index) const
  {
    return Cd[index];
  }
  double getAdaptiveTempGap(int index) const
  {
    return adaptiveTempGap[index];
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

  void setAdaptiveTempGap(int index, double value, bool clampToLimits = false)
  {
    setArrayValue(adaptiveTempGap, index, value, static_cast<double>(0), static_cast<double>(MAX_TEMP), isInvalid, clampToLimits);
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
Adafruit_MAX31855 thermocouples[2] = {
    Adafruit_MAX31855(PIN_TC_CLK, PIN_TC_CS1, PIN_TC_DO),
    Adafruit_MAX31855(PIN_TC_CLK, PIN_TC_CS2, PIN_TC_DO)};

// PID
PID pidControllers[2] = {PID(&pData.temp[0], &pData.output[0], &pData.setpoint[0], DEF_KP, DEF_KI, DEF_KD, P_ON_M, DIRECT),
                         PID(&pData.temp[1], &pData.output[1], &pData.setpoint[1], DEF_KP, DEF_KI, DEF_KD, P_ON_M, DIRECT)};

// Relay pins
const int relayPins[2] = {PIN_SSR1, PIN_SSR2};

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
  CountTimers ct;     // Counters
  PointInTime pit;    // Point in time
  LastUpdateTime lut; // Last update time
};

Timing timing = {0};

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
  for (int i = 0; i < 2; i++)
  {
    double newValue = pData.getSetpoint(i) + pData.incr;
    pData.setSetpoint(i, newValue);
  }
}

void incrementDuration()
{
  unsigned long newValue = pData.getSetDuration() + (pData.incr * MINUTE);
  pData.setSetDuration(newValue);
}

void incrementKp()
{
  for (int i = 0; i < 2; i++)
  {
    double newValue = pData.getKp(i) + pData.incr;
    pData.setKp(i, newValue);
    pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }
}

void incrementKi()
{
  for (int i = 0; i < 2; i++)
  {
    double newValue = pData.getKi(i) + pData.incr;
    pData.setKi(i, newValue);
    pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }
}

void incrementKd()
{
  for (int i = 0; i < 2; i++)
  {
    double newValue = pData.getKd(i) + pData.incr;
    pData.setKd(i, newValue);
    pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }
}

char currStateName[16] = "standby";

LiquidLine tempsLine(1, 0, "T:", pData.temp[0], ",", pData.temp[1]);
LiquidLine setpointLine(10, 0, " S:", pData.setpoint[0]);
LiquidLine outputsLine(1, 1, "O:", pData.output[0], ",", pData.output[1]);
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
#endif

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

  for (int i = 0; i < 2; i++)
  {
    // Initialize thermocouples
    thermocouples[i].begin();

    // Initialize PID controllers
    pidControllers[i].SetMode(AUTOMATIC);
    pidControllers[i].SetOutputLimits(0, MAX_OUTPUT);
    pidControllers[i].SetSampleTime(PID_INTERVAL);
    pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), pData.getKd(i));
  }

  pinMode(relayPins[0], OUTPUT);
  pinMode(relayPins[1], OUTPUT);

  monitor.initialize();

#ifdef GUI
  initializeEncoder(PIN_ENC_DT, PIN_ENC_CLK, PIN_ENC_SW, ENC_STEPS);
  initializeLCD();

  // Attach functions to lines for options
  backLine.attach_function(FUNC_USE, goToMainScreen);

  currStateLine.attach_function(FUNC_USE, startProcess);
  currStateLine.attach_function(FUNC_SKIP, skipProcessPhase);
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
#endif

  Serial.println(F("System initialized"));
}

void updateTiming()
{
  // Save the previous master time for elapsed time calculation
  unsigned long lutMasterPrev = timing.lut.master;

  // Update the master time
  timing.lut.master = millis();

  // Calculate the elapsed time since the last update
  unsigned long elapsedSinceLastUpdate = timing.lut.master - lutMasterPrev;

  // Update the elapsed time (since beginning of the process)
  timing.ct.elapsed = timing.lut.master - timing.pit.preStart;

  // Only update the remaining duration if the process has reached the active state
  if (timing.lut.master > 0 && currMState == ACTIVE)
  {
    timing.ct.durationRemaining = (timing.ct.durationRemaining > elapsedSinceLastUpdate) ? (timing.ct.durationRemaining - elapsedSinceLastUpdate) : 0;
  }

  // Update the program data struct with the remaining duration both in milliseconds and minutes
  pData.remainingDurationMillis = timing.ct.durationRemaining;
  pData.remainingDurationMinutes = pData.remainingDurationMillis / MINUTE;
}

// -------------------------------
// ------ Main program loop ------
// -------------------------------
void loop()
{

  // If the state has changed, execute "on entry" actions for the new state
  if (currMState != prevMState)
  {
    stateEntrySwitch(currMState);
    prevMState = currMState;
  }

  for (int i = 0; i < 2; i++)
  {
    // This is to stop nan from propogating in event of a TC error
    // See 'Safeguard against NaN' in docs for more info
    if (isnan(pData.output[i]))
    {
      pidControllers[i].SetMode(MANUAL);
      pData.output[i] = 0;
      pidControllers[i].SetMode(AUTOMATIC);
    }

    // Apply adaptive tunings
    if (abs(pData.setpoint[i] - pData.temp[i] < DEF_ADAPTIVE_TEMP_GAP))
    {
      pidControllers[i].SetTunings(pData.Cp[i], pData.Ci[i], pData.Cd[i]);
    }
    else
    {
      pidControllers[i].SetTunings(pData.Kp[i], pData.Ki[i], pData.Kd[i]);
    }

    // Compute the PID output
    if (currMState == PREPARING || currMState == ACTIVE)
      pidControllers[i].Compute();
  }

  // Update the EEPROM data every EEPROM_UPDATE_T milliseconds
  updateEeprom();

  // Update the timing if the machine is preparing or active
  if (currMState == PREPARING || currMState == ACTIVE)
  {
    updateTiming();
  }

  // Handle serial commands if enabled
#ifdef SERIALCMD
  handleSerialCommands();
#endif

  // Update control every CONTROL_INTERVAL milliseconds
  if (millis() - timing.lut.control >= CONTROL_INTERVAL)
  {
    // Update the control timing
    timing.lut.control = millis();
    readTemps();
    pData.traStatus = monitor.updateThermalRunaway(pData.setpoint, pData.temp);

    // Crude check if output is too high
    // This is really for SUDO mode and will be removed
    // in the future and replaced with separation of PID owned vars
    // and program data where it must be "accepted" into the program from PID
    if (pData.output[0] > MAX_OUTPUT || pData.output[1] > MAX_OUTPUT)
    {
      // Set to upper limit
      pData.output[0] = MAX_OUTPUT;
      pData.output[1] = MAX_OUTPUT;
    }

    if (currMState == PREPARING || currMState == ACTIVE || currMState == SUDO)
    {
      for (int i = 0; i < 2; i++)
      {
        pidControllers[i].SetMode(AUTOMATIC);
      }
      writeRelays(pData.output, relayPins);
    }
    else
    {
      for (int i = 0; i < 2; i++)
      {
        pidControllers[i].SetMode(MANUAL);
        pData.output[i] = 0;
      }
      writeRelays(pData.output, relayPins);
    }
  }

#ifdef GUI
  // Update GUI every GUI_INTERVAL milliseconds
  if (millis() - timing.lut.gui >= GUI_INTERVAL)
  {
    timing.lut.gui = millis();
    strncpy(currStateName, getMachineStateName(currMState), sizeof(currStateName) - 1);
    currStateName[sizeof(currStateName) - 1] = '\0'; // null-termination

    pData.incr = processEncoderEvents();
    updateLcdGui();
  }
#endif

  // Log data every LOG_INTERVAL milliseconds
  if (millis() - timing.lut.log >= LOG_INTERVAL)
  {
    timing.lut.log = millis();
#ifdef SERIALCMD
    printData();
#endif // SERIALCMD
    if (pData.sdActive && (currMState == PREPARING || currMState == ACTIVE))
    {
#ifdef SDCARD
      logData();
#endif
    }
  }
}
