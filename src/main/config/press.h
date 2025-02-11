#ifndef FURNACE_H
#define FURNACE_H

#include "press/hardware_def.h" // This file contains hardware-specific configurations such as pin assignments
#include "press/operating_limits.h"   // This file contains the limits for the various parameters used in the program
#include "default/timing_intervals.h"   // This file contains the timing intervals for the control loop and other processes
#include "default/eeprom_settings.h"   // This file contains the EEPROM settings for the program

// ------------------------------
// Control Settings
// ------------------------------

// Default Heating Process Settings
#define DEF_SETPOINT 250
#define DEF_HEATING_DURATION 3000000 // 50 minutes

// PID Default Settings
#define DEF_KP 0.50
#define DEF_KI 0.01
#define DEF_KD 0.00

// Number of Control Loops
#define NUM_CTRL 2

// Preheat to Heat Settings
#define DEF_PRE_TO_HEAT_TEMP_OFFSET 5
#define DEF_PRE_TO_HEAT_HOLD_TIME 10 * 1000
#define TERM_TEMP 40

#endif // FURNACE_H
