#ifndef CONFIG_H
#define CONFIG_H

#define SERIALCMD 1
#define SDCARD 1

// Number of Sensors and Relays, needs more work
// to allow for more cohesive definition of these
// values connected with the hardware
#define NUM_SENSORS 2
#define NUM_RELAYS 2

// Thermocouple Pins
#define PIN_TC_DO 4
#define PIN_TC_CLK 5
#define PIN_TC_CS1 6
#define PIN_TC_CS2 7

// Relay Pins
#define PIN_SSR1 8
#define PIN_SSR2 9

// SD Card Pins
#define SD_CS 10
#define SD_MOSI 11
#define SD_MISO 12
#define SD_SCK 13

// Rotary Encoder Pins
#define PIN_ENC_SW 17
#define PIN_ENC_CLK 2
#define PIN_ENC_DT 3

#define REVERSE_ENCODER // uncomment if your encoder is reversed

// Timing Intervals
#define CONTROL_INTERVAL 100 // 10 Hz (100 ms)
#define GUI_INTERVAL 40      // ~25 Hz (40 ms)
#define LOG_INTERVAL 1000    // 1 Hz (1000 ms)

// EEPROM Update Interval
#define EEPROM_UPDATE_T 10000

// Thermal runaway settings
#define THRM_RUNAWAY_DELTA 30
#define THRM_RUNAWAY_CYCLES 20

#define RELAY_PWM_PERIOD 1 * 1000 // 1 Hz (1000 ms)

// Default Heating Process Settings
#define DEF_SET_TEMP 25
#define DEF_HEATING_DURATION 300000

// Preheat to Heat Settings !!!!!!!!
#define DEF_PRE_TO_HEAT_TEMP_OFFSET 5
#define DEF_PRE_TO_HEAT_HOLD_TIME 10 * 1000

// PID Default Settings
#define DEF_KP 50
#define DEF_KI 1
#define DEF_KD 0.2
#define DEF_CP 2
#define DEF_CI 2
#define DEF_CD 2
#define DEF_GAP_THRESHOLD 20

// Limits !!!!!!
/*
#define MIN_TEMP 0
#define MAX_TEMP 480
#define MIN_CONTROL_PERIOD 500
#define MAX_CONTROL_PERIOD 5000
#define MIN_PROCESS_INTERVAL 5
#define MAX_PROCESS_INTERVAL 100
#define MIN_HEATING_DURATION (1.0 * 60 * 1000)
#define MAX_HEATING_DURATION (120.0 * 60 * 1000)
#define MIN_PRE_TO_HEAT_TEMP_OFFSET 0
#define MAX_PRE_TO_HEAT_TEMP_OFFSET 25
#define MIN_PRE_TO_HEAT_HOLD_TIME 1000
#define MAX_PRE_TO_HEAT_HOLD_TIME 60000
#define MIN_SERIAL_PRINT_INTERVAL 100
#define MAX_SERIAL_PRINT_INTERVAL 5000
#define MIN_KP 0
#define MAX_KP 2000
#define MIN_KI 0
#define MAX_KI 500
#define MIN_KD 0
#define MAX_KD 100
#define MIN_CP 1
#define MAX_CP 5
#define MIN_CI 1
#define MAX_CI 5
#define MIN_CD 1
#define MAX_CD 5
#define MIN_GAP_THRESHOLD 5
#define MAX_GAP_THRESHOLD 50
*/

#endif // CONFIG_H
