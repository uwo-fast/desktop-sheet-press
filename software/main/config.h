#ifndef CONFIG_H
#define CONFIG_H

#include "config/hardware_config.h" // This file contains hardware-specific configurations such as pin assignments
#include "config/limits_config.h"   // This file contains the limits for the various parameters used in the program
#include "config/timing_config.h"   // This file contains the timing intervals for the control loop and other processes
#include "config/eeprom_config.h"   // This file contains the EEPROM settings for the program

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
// Default Settings
// ------------------------------

// Default Heating Process Settings
#define DEF_SETPOINT 250
#define DEF_HEATING_DURATION 3000000 // 50 minutes

// PID Default Settings
#define DEF_KP 5
#define DEF_KI 0.01
#define DEF_KD 0.001

// ------------------------------
// Miscalleanous
// ------------------------------

// Number of Control Loops
#define NUM_CTRL 2

// Preheat to Heat Settings
#define DEF_PRE_TO_HEAT_TEMP_OFFSET 5
#define DEF_PRE_TO_HEAT_HOLD_TIME 10 * 1000
#define TERM_TEMP 40

#endif // CONFIG_H
