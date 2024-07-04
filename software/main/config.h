

/***************************************************************************************************
 * User Configuration                                                                               *
 ***************************************************************************************************/
// Uncomment the below line to enable the serial command interface
 #define _SERIALCMD_ 1 /**< Enable serial command interface */
// Uncomment the below line to enable the LCD GUI interface
// #define _LCDGUI_ 1 /**< Enable the LCD GUI interface */
// Uncomment the below line to enable development mode for use without GUI
// #define _DEVELOPMENT_  1              /**< Allows printing of to serial for development*/
// Uncomment the below line to enable boot to system menu for testing, currently only for manual eeprom reset
// #define _BOOTSYS_   1                        /**< Force boot to system menu for testing. NEED TO ENABLE GUI */

#define _LANG_EN_            /**< Language:  _LANG_EN/DE/FR/ES/IT_ */

/***************************************************************************************************
 * Pin definitions                                                                                 *
 ***************************************************************************************************/
#define PIN_ENC_SW 17 /**< Rotary encoder push button switch input */
#define PIN_ENC_CLK 2 /**< Rotary encoder CLK input  */
#define PIN_ENC_DT 3  /**< Rotary encoder DT input */

#define PIN_TC_DO 4  /**< Thermocouple SPI data out */
#define PIN_TC_CLK 5 /**< Thermocouple SPI clock */
#define PIN_TC_CS1 6 /**< Thermocouple 1 SPI chip select */
#define PIN_TC_CS2 7 /**< Thermocouple 2 SPI chip select */

#define SD_CS 10   /**< SD card SPI CS */
#define SD_MOSI 11 /**< SD card SPI MOSI */
#define SD_MISO 12 /**< SD card SPI MISO */
#define SD_SCK 13  /**< SD card SPI SCK */

#define PIN_SSR1 8 /**< Relay 1 output */
#define PIN_SSR2 9 /**< Relay 2 output */

/***************************************************************************************************
 * Macros                                                                                           *
 ***************************************************************************************************/
// Defaults for operational variables

#define DEF_TEMP_RUNA_DELTA 30          /**< Default temperature gap in C */
#define DEF_TEMP_RUNA_CYCLES 20         /**< Default number of allowed active temperature gap cycles */
#define DEF_SET_TEMP 150                /**< Default set temperature in C */
#define DEF_CONTROL_PERIOD 1000         /**< Default control period (ms) */
#define DEF_PROCESS_INTERVAL 10         /**< Default process interval (ms) */
#define DEF_HEATING_DURATION 300000     /** <f e.g. 300000 = 5 minute */
#define DEF_PRE_TO_HEAT_TEMP_OFFSET 5   /**< Default preheat to heating process state temperature offset to transition requirements */
#define DEF_PRE_TO_HEAT_HOLD_TIME 10000 /**< (ms) Default preheat to heating process state required hold time to meet transition requirements */
#define DEF_SERIAL_PRINT_INTERVAL 1000  /**< Default serial print interval (ms) */
#define DEF_KP 5000                     /**< Default Proportional milli gain [5.000] */
#define DEF_KI 0400                     /**< Default Integral milli gain [0.400] */
#define DEF_KD 0050                     /**< Default Derivative milli gain [0.050] */
#define DEF_CP 2                        /**< Default Derivative scale constant for dynamic tuning */
#define DEF_CI 2                        /**< Default Integral scale constant for dynamic tuning */
#define DEF_CD 2                        /**< Default Derivative scale constant for dynamic tuning */
#define DEF_GAP_THRESHOLD 20            /**< Default temperature gap threshold */

// Limits for operational variables
#define MIN_TEMP_RUNA_DELTA 31                   /**< Current hardcoded until further investigation into safe limits are done before users can be given control */
#define MAX_TEMP_RUNA_DELTA 29                   /**< Current hardcoded until further investigation into safe limits are done before users can be given control */
#define MIN_TEMP_RUNA_CYCLES 19                  /**< Current hardcoded until further investigation into safe limits are done before users can be given control */
#define MAX_TEMP_RUNA_CYCLES 21                  /**< Current hardcoded until further investigation into safe limits are done before users can be given control */
#define MIN_TEMP 0                               /**< Min temperature, thermocouple min is -100C; this would require elements to be removed and additional cooling implementation */
#define MAX_TEMP 480                             /**< Max temperature, thermocouple max is 1100C and max of heaters is 480C */
#define MIN_CONTROL_PERIOD 500                   /**< Minimum control period (ms) */
#define MAX_CONTROL_PERIOD 5000                  /**< Maximum control period (ms) */
#define MIN_PROCESS_INTERVAL 5                   /**< Minimum process interval (ms) */
#define MAX_PROCESS_INTERVAL 100                 /**< Maximum process interval (ms) */
#define MIN_HEATING_DURATION (1.0 * 60 * 1000)   /**< Minimum process duration in ms (1 minute) */
#define MAX_HEATING_DURATION (120.0 * 60 * 1000) /**< Maximum process duration in ms (120 minutes) */
#define MIN_PRE_TO_HEAT_TEMP_OFFSET 0            /**< Minimum preheat to heating process state temperature offset to transition requirements */
#define MAX_PRE_TO_HEAT_TEMP_OFFSET 25           /**< Maximum preheat to heating process state temperature offset to transition requirements */
#define MIN_PRE_TO_HEAT_HOLD_TIME 1000           /**< Minimum preheat to heating process state required hold time to  meet transition requirements */
#define MAX_PRE_TO_HEAT_HOLD_TIME 60000          /**< Maximum preheat to heating process state required hold time to  meet transition requirements */
#define MIN_SERIAL_PRINT_INTERVAL 100            /**< Minimum serial print interval (ms) */
#define MAX_SERIAL_PRINT_INTERVAL 5000           /**< Maximum serial print interval (ms) */
#define MIN_KP 0                                 /**< Minimum Proportional milli gain */
#define MAX_KP 2000                              /**< Maximum Proportional milli gain */
#define MIN_KI 0                                 /**< Minimum Integral milli gain */
#define MAX_KI 500                               /**< Maximum Integral milli gain */
#define MIN_KD 0                                 /**< Minimum Derivative milli gain */
#define MAX_KD 100                               /**< Maximum Derivative milli gain */
#define MIN_CP 1                                 /**< Minimum Derivative scale constant for dynamic tuning */
#define MAX_CP 5                                 /**< Maximum Derivative scale constant for dynamic tuning */
#define MIN_CI 1                                 /**< Minimum Integral scale constant for dynamic tuning */
#define MAX_CI 5                                 /**< Maximum Integral scale constant for dynamic tuning */
#define MIN_CD 1                                 /**< Minimum Derivative scale constant for dynamic tuning */
#define MAX_CD 5                                 /**< Maximum Derivative scale constant for dynamic tuning */
#define MIN_GAP_THRESHOLD 5                      /**< Minimum temperature gap threshold */
#define MAX_GAP_THRESHOLD 50                     /**< Maximum temperature gap threshold */

// Timing macros
#define STANDBY_TIME_OUT 300000L /**< Device sleep timeout (ms) */
#define EEPROM_UPDATE_T 5000     /**< EEPROM update time (ms) */
#define RS_DEBOUNCE 50 /*20*/    /**< Rotary encoder & switch debounce time (ms) */
#define T_INTERVAL 10000         /**< temperature measurement interval (ms) */
