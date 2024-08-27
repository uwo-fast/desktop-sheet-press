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
#define ENCSTEPS 4
#define REVERSE_ENCODER 1 // uncomment if your encoder is reversed

// Timing Intervals
#define CONTROL_INTERVAL 100 // 10 Hz (100 ms)
#define GUI_INTERVAL 200     // 5 Hz (200 ms)
#define LOG_INTERVAL 2000    // 0.5 Hz (2000 ms)

// EEPROM Settings
#define EEPROM_UPDATE_T 10000
#define EEA_ID 0
#define EEA_PDATA (EEA_ID + 4)
#define EE_UNIQUEID 0x18fae9c8
#define EE_FULL_RESET true
#define EE_PARTIAL_RESET false

#define RELAY_PWM_PERIOD 1000 // 1 Hz (1000 ms)

#define MAX_DURATION 300 * 60 * 1000 // 300 minutes or 5 hours

// Default Heating Process Settings
#define DEF_SETPOINT 25
#define DEF_HEATING_DURATION 3000000 // 50 minutes

// Preheat to Heat Settings !!!!!!!!
#define DEF_PRE_TO_HEAT_TEMP_OFFSET 5
#define DEF_PRE_TO_HEAT_HOLD_TIME 10 * 1000
#define TERM_TEMP 40

// PID Default Settings
#define DEF_KP 50
#define DEF_KI 1
#define DEF_KD 0.2
#define DEF_CP 1
#define DEF_CI 1
#define DEF_CD 1
#define DEF_GAP_THRESHOLD 10

// Limits for pData
#define MIN_TEMP 0
#define MAX_TEMP 480
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
#define MIN_SET_DURATION 1000
#define MAX_SET_DURATION 10800000 // 3 hours

#endif // CONFIG_H
