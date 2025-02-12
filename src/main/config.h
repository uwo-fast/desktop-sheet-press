#ifndef CONFIG_H
#define CONFIG_H

// ------------------------------
// Choose Configuration
// ------------------------------

// Define valid configuration options
#define NONE 0
#define PRESS 1
#define FURNACE 2

// Set the desired configuration HERE:
#define CONFIG_SELECT FURNACE // Change this to switch configurations

// Ensure CONFIG_SELECT is a valid option
#if CONFIG_SELECT == NONE
#include "config/default.h"
#pragma message "Compiling with default configuration"
#elif CONFIG_SELECT == PRESS
#include "config/press.h"
#pragma message "Compiling with press configuration"
#elif CONFIG_SELECT == FURNACE
#include "config/furnace.h"
#pragma message "Compiling with furnace configuration"
#else
#error "Invalid CONFIG_SELECT selection"
#endif

// ------------------------------
// Feature Enable/Disable
// ------------------------------

#define SERIALCMD 1       // Enable serial command line interface
#define SDCARD 1          // Enable SD card logging
#define GUI 1             // Enable LCD + encoder GUI
#define REVERSE_ENCODER 1 // Reverse encoder direction if needed
#define RESET_EEPROM 1    // Reset EEPROM settings (partial) on start up, for testing

// Display feature settings
#if SERIALCMD
#pragma message "Serial command line interface enabled"
#endif
#if SDCARD
#pragma message "SD card logging enabled"
#endif
#if GUI
#pragma message "LCD + encoder GUI enabled"
#endif
#if REVERSE_ENCODER
#pragma message "Encoder direction reversed"
#endif

// ------------------------------
// Temperature Profile Selection
// ------------------------------

#define EXAMPLE_PROFILE 10
#define SINTERING_ALPHA 11

// Select the temperature profile HERE:
#define TEMP_PROFILE EXAMPLE_PROFILE

#if TEMP_PROFILE == EXAMPLE_PROFILE
#include "profiles/example_profile.h"
#pragma message "Compiling with temperature profile example_profile"
#elif TEMP_PROFILE == SINTERING_ALPHA
#include "profiles/sintering_alpha.h"
#pragma message "Compiling with temperature profile sintering_alpha"
#else
#error "Invalid TEMP_PROFILE selection"
#endif

#endif // CONFIG_H
