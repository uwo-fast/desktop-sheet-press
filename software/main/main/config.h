#ifndef CONFIG_H
#define CONFIG_H

#define NUM_SENSORS 2
#define NUM_RELAYS 2

const unsigned long HIGH_FREQ_INTERVAL = 100;  // 10 Hz (100 ms)
const unsigned long MONITORING_INTERVAL = 1000; // 1 Hz (1000 ms)
const unsigned long LOGGING_INTERVAL = 5000;    // 0.2 Hz (5000 ms)
const unsigned long COMMUNICATION_INTERVAL = 200; // 5 Hz (200 ms)
const unsigned long UI_UPDATE_INTERVAL = 33;     // ~30 Hz (33 ms)

// EEPROM Update Interval
#define EEPROM_UPDATE_T 5000

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

#define REVERSE_ENCODER //uncomment if your encoder is reversed

// Default Values
#define DEF_TEMP_RUNA_DELTA 30
#define DEF_TEMP_RUNA_CYCLES 20
#define DEF_SET_TEMP 50
#define DEF_CONTROL_PERIOD 1000
#define DEF_PROCESS_INTERVAL 10
#define DEF_HEATING_DURATION 300000
#define DEF_PRE_TO_HEAT_TEMP_OFFSET 5
#define DEF_PRE_TO_HEAT_HOLD_TIME 10000
#define DEF_SERIAL_PRINT_INTERVAL 1000
#define DEF_KP 500
#define DEF_KI 100
#define DEF_KD 50
#define DEF_CP 2
#define DEF_CI 2
#define DEF_CD 2
#define DEF_GAP_THRESHOLD 20

// Limits
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

// Macros for default values
#define STANDBY_TIME_OUT 300000L
#define RS_DEBOUNCE 50
#define T_INTERVAL 10000
#define MILLI_UNIT 1000

// EEPROM unique ID
#define EEA_ID 0
#define EEA_PDATA (EEA_ID + 4)
#define EE_UNIQUEID 0x18fae9c8
#define EE_FULL_RESET true

#endif // CONFIG_H
