#ifndef CONFIG_H
#define CONFIG_H

// ------------------------------
// Choose Configuration
// ------------------------------

// #include "config/default.h"
#include "config/press.h"
// #include "config/furnace.h"

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

#endif // CONFIG_H
