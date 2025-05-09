#ifndef CONFIG_H
#define CONFIG_H

// ------------------------------
// Choose Configuration
// ------------------------------

#include "config/hardware_def.h"     // This file contains hardware-specific configurations such as pin assignments
#include "config/operating_limits.h"  // This file contains the limits for the various parameters used in the program
#include "config/timing_intervals.h" // This file contains the timing intervals for the control loop and other processes
#include "config/eeprom_settings.h"  // This file contains the EEPROM settings for the program

// ------------------------------
// Feature Enable/Disable
// ------------------------------

// Comment/uncomment the following to enable the corresponding feature

// This is for the serial command line interface
#define SERIALCMD 1

// This is for the SD card logging
#define SDCARD 1

// This is for the LCD + encoder GUI
#define GUI 1

#define REVERSE_ENCODER 1 // uncomment if your encoder is reversed

// ------------------------------
// Control Settings
// ------------------------------

// Default Heating Process Settings
#define DEF_SETPOINT 60
#define DEF_HEATING_DURATION 3000000 // 50 minutes

// PID Default Settings
#define DEF_KP 0.50
#define DEF_KI 0.05
#define DEF_KD 0.00

// PID Adaptive Settings
#define DEF_ADAPTIVE_TEMP_GAP 10
#define DEF_CP 0.10
#define DEF_CI 0.01
#define DEF_CD 0.00

// Preheat to Heat Settings
#define DEF_PRE_TO_HEAT_TEMP_OFFSET 5
#define DEF_PRE_TO_HEAT_HOLD_TIME 10 * 1000

#endif // CONFIG_H
