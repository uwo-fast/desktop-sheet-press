#ifndef DEFAULT_H
#define DEFAULT_H

#include "default/hardware_def.h"     // This file contains hardware-specific configurations such as pin assignments
#include "default/operating_limits.h" // This file contains the limits for the various parameters used in the program
#include "default/timing_intervals.h" // This file contains the timing intervals for the control loop and other processes
#include "default/eeprom_settings.h"  // This file contains the EEPROM settings for the program

// ------------------------------
// Control Settings
// ------------------------------

// Default Heating Process Settings
#define DEF_SETPOINT 120
#define DEF_HEATING_DURATION 3000000 // 50 minutes

// PID Default Settings
#define DEF_KP 5.00
#define DEF_KI 0.01
#define DEF_KD 0.001

// Number of Control Loops
#define NUM_CTRL 2

// Preheat to Heat Settings
#define DEF_PRE_TO_HEAT_TEMP_OFFSET 5
#define DEF_PRE_TO_HEAT_HOLD_TIME 10 * 1000
#define TERM_TEMP 40

#endif // DEFAULT_H
