#ifndef _PRESS_CONTROL_v1_0_0_H
#define _PRESS_CONTROL_v1_0_0_H

/***********************************************************************************************//**
 *  \par        Press Controller - Header.
 *
 *  \par        Details
 *  \par
 *              Press Controller v1 Firmware.
 *              This is the Arduino Code for the Open Source Cold Hot Scientific Sheet Press. 
 *  \par
 *              You need to have the following libraries installed in your Arduino IDE:
 *               - [leave blank fill later with all the libraries]
 *
 *              This release has been tested with the following library versions:
 *              [lib]  Vxx.yy.zz
 *              
 *  \file       press_controller_v1_0_0.h
 *  \author     Cameron K. Brooks
 *  \authors    Adapted from Arduino_Spot_Welder_V4_0_2.ino <https://github.com/KaeptnBalu/Arduino_Spot_Welder_V4> by Marc Schönfleisch <http://malectrics.eu/> 
 *  \version    1.0.0
 *  \date       Mar 2024
 *  \copyright  Copyright(c)2024 Marc Schönfleisch <info@malectrics.eu>. All right reserved.
 *  \copyright  Copyright(c) 2024 Cameron K. Brooks <cambrooks3393@gmail.com>.
 *
 *  \par        Changelog
 *  \li         2024-MM-DD  -  Initial release
 *
 *  \par        License
 *              This program is free software: you can redistribute it and/or modify it under the
 *              terms of the GNU General Public License as published by the Free Software 
 *              Foundation, either version 3 of the License, or (at your option) any later version.
 *  \par
 *              This program is distributed in the hope that it will be useful, but WITHOUT ANY 
 *              WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
 *              PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *  \par
 *              You should have received a copy of the GNU General Public License along with this 
 *              program.  If not, see <http://www.gnu.org/licenses/> or write to the Free Software 
 *              Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.
 *
 *//***********************************************************************************************/


// Most of these macros define strings that appear on the splash screen during set up 
// (each line of the splash screen is limited to 16 characters)
#define _DEVICENAME_        "OSCHSSP"
#define _PROGNAME_          "Press Controller"
#define _AUTHOR_            "CKB - MCW - JMP"
#define _VERSION_MAJOR_     1
#define _VERSION_MINOR_     0
#define _REVISION_          0
#define _COPYRIGHT_         "2024"
#define _LICENSE_           "GNU V3 "FREE""
#define _ORGANIZATION_      "FAST Research"

/***************************************************************************************************
* User Configuration                                                                               *
***************************************************************************************************/
// Uncomment the below line to enable the serial command interface
#define _SERIALCMD_ 1                  /**< Enable serial command interface */
// Uncomment the below line to enable the LCD GUI interface
#define _LCDGUI_  1                    /**< Enable the LCD GUI interface */
// Uncomment the below line to enable development mode for use without GUI
// #define _DEVELOPMENT_  1              /**< Allows printing of to serial for development*/
// Uncomment the below line to enable boot to system menu for testing, currently only for manual eeprom reset
// #define _BOOTSYS_   1                        /**< Force boot to system menu for testing. NEED TO ENABLE GUI */


#define _LANG_EN_                       /**< Language:  _LANG_EN/DE/FR/ES/IT_ */
#define _SERIAL_BAUD_       115200      /**< Comms rate for serial, maximum before it got dicey was 115200*/

/***************************************************************************************************
* Pin and interrupt definitions                                                                    *
***************************************************************************************************/
#define PIN_ENC_SW              17               /**< Rotary encoder push button switch input */
#define PIN_ENC_DT              6               /**< Rotary encoder DT input */
#define PIN_ENC_CLK             7               /**< Rotary encoder CLK input  */

#define PIN_TC_CS1              4               /**< Thermocouple 1 SPI chip select */
#define PIN_TC_CLK              2               /**< Thermocouple SPI clock */
#define PIN_TC_CS2              5               /**< Thermocouple 2 SPI chip select */
#define PIN_TC_DO               3               /**< Thermocouple SPI data out */

#define SD_MISO                 12               /**< SD card SPI MISO */
#define SD_SCK                  13              /**< SD card SPI SCK */
#define SD_CS                   10              /**< SD card SPI CS */
#define SD_MOSI                 11              /**< SD card SPI MOSI */

#define PIN_SSR1                8               /**< Relay 1 output | 14 - A0 */
#define PIN_SSR2                9               /**< Relay 2 output | 15 - A1 */

/***************************************************************************************************
* Macros                                                                                           *
***************************************************************************************************/
// Defaults for operational variables

#define DEF_TEMP_RUNA_DELTA  30              /**< Default temperature gap in C */
#define DEF_TEMP_RUNA_CYCLES  20              /**< Default number of allowed active temperature gap cycles */
#define DEF_SET_TEMP         150              /**< Default set temperature in C */
#define DEF_CONTROL_PERIOD   1000             /**< Default control period (ms) */
#define DEF_PROCESS_INTERVAL 10             /**< Default process interval (ms) */
#define DEF_HEATING_DURATION 300000          /** <f e.g. 300000 = 5 minute */
#define DEF_PRE_TO_HEAT_TEMP_OFFSET 5        /**< Default preheat to heating process state temperature offset to transition requirements */
#define DEF_PRE_TO_HEAT_HOLD_TIME  10000         /**< (ms) Default preheat to heating process state required hold time to meet transition requirements */
#define DEF_SERIAL_PRINT_INTERVAL 1000       /**< Default serial print interval (ms) */
#define DEF_KP               2000               /**< Default Proportional milli gain [2.000] */
#define DEF_KI               0400                /**< Default Integral milli gain [0.400] */
#define DEF_KD               0050                /**< Default Derivative milli gain [0.050] */
#define DEF_CP               2                /**< Default Derivative scale constant for dynamic tuning */
#define DEF_CI               2                /**< Default Integral scale constant for dynamic tuning */
#define DEF_CD               2                /**< Default Derivative scale constant for dynamic tuning */
#define DEF_GAP_THRESHOLD    20                /**< Default temperature gap threshold */

// Limits for operational variables
#define MIN_TEMP_RUNA_DELTA 31              /**< Current hardcoded until further investigation into safe limits are done before users can be given control */
#define MAX_TEMP_RUNA_DELTA 29             /**< Current hardcoded until further investigation into safe limits are done before users can be given control */
#define MIN_TEMP_RUNA_CYCLES 19              /**< Current hardcoded until further investigation into safe limits are done before users can be given control */
#define MAX_TEMP_RUNA_CYCLES 21             /**< Current hardcoded until further investigation into safe limits are done before users can be given control */
#define MIN_TEMP            0               /**< Min temperature, thermocouple min is -100C; this would require elements to be removed and additional cooling implementation */
#define MAX_TEMP            480              /**< Max temperature, thermocouple max is 1100C and max of heaters is 480C */
#define MIN_CONTROL_PERIOD  500             /**< Minimum control period (ms) */
#define MAX_CONTROL_PERIOD  5000            /**< Maximum control period (ms) */
#define MIN_PROCESS_INTERVAL 5              /**< Minimum process interval (ms) */
#define MAX_PROCESS_INTERVAL 100            /**< Maximum process interval (ms) */
#define MIN_HEATING_DURATION (1.0 * 60 * 1000) /**< Minimum process duration in ms (1 minute) */
#define MAX_HEATING_DURATION (120.0 * 60 * 1000) /**< Maximum process duration in ms (120 minutes) */
#define MIN_PRE_TO_HEAT_TEMP_OFFSET 0       /**< Minimum preheat to heating process state temperature offset to transition requirements */
#define MAX_PRE_TO_HEAT_TEMP_OFFSET 25      /**< Maximum preheat to heating process state temperature offset to transition requirements */
#define MIN_PRE_TO_HEAT_HOLD_TIME  1000     /**< Minimum preheat to heating process state required hold time to  meet transition requirements */
#define MAX_PRE_TO_HEAT_HOLD_TIME  60000    /**< Maximum preheat to heating process state required hold time to  meet transition requirements */
#define MIN_SERIAL_PRINT_INTERVAL 100       /**< Minimum serial print interval (ms) */
#define MAX_SERIAL_PRINT_INTERVAL 5000      /**< Maximum serial print interval (ms) */
#define MIN_KP               0               /**< Minimum Proportional milli gain */
#define MAX_KP               2000            /**< Maximum Proportional milli gain */
#define MIN_KI               0               /**< Minimum Integral milli gain */
#define MAX_KI               500             /**< Maximum Integral milli gain */
#define MIN_KD               0               /**< Minimum Derivative milli gain */
#define MAX_KD               100             /**< Maximum Derivative milli gain */
#define MIN_CP               1               /**< Minimum Derivative scale constant for dynamic tuning */
#define MAX_CP               5               /**< Maximum Derivative scale constant for dynamic tuning */
#define MIN_CI               1               /**< Minimum Integral scale constant for dynamic tuning */
#define MAX_CI               5               /**< Maximum Integral scale constant for dynamic tuning */
#define MIN_CD               1               /**< Minimum Derivative scale constant for dynamic tuning */
#define MAX_CD               5               /**< Maximum Derivative scale constant for dynamic tuning */
#define MIN_GAP_THRESHOLD    5               /**< Minimum temperature gap threshold */
#define MAX_GAP_THRESHOLD    50              /**< Maximum temperature gap threshold */


// Timing macros
#define STANDBY_TIME_OUT    300000L         /**< Device sleep timeout (ms) */ 
#define EEPROM_UPDATE_T     5000            /**< EEPROM update time (ms) */
#define RS_DEBOUNCE         50  /*20*/      /**< Rotary encoder & switch debounce time (ms) */
#define T_INTERVAL          10000           /**< temperature measurement interval (ms) */
#define MILLI_UNIT          1000            /**< Milli unit for gain values */


// Macros to define logical states
#define DD_READ             true            /**< Data transfer direction - read */ // UN USED ???
#define DD_WRITE            false           /**< Data transfer direction - write */ // UN USED ???
#define P_ON                true            /**< General macro for ON state */
#define P_OFF               false           /**< General macro for OFF state */
#define PL_ACTIVE_H         false           /**< Pin logic macro for Active High */
#define PL_ACTIVE_L         true            /**< Pin logic macro for Active Low */
#define TC_FAULT            false            /**< Thermocouple fault state */

// EEPROM macros
#define EEA_ID              0               /**< Address of unique ID */
#define EEA_PDATA           (EEA_ID+4)      /**< Eeprom address of program data */
#define EE_UNIQUEID         0x18fae9c8      /**< Unique Eeprom verification ID, arbitrary */
#define EE_FULL_RESET       true            /**< Reset parameter to reset all Eeprom parameters */

// Macros masquerading as functions - makes the code more readable
/** This macro reads the state of the pushbutton switch on the encoder. */
#define btnState()          (!digitalRead(PIN_SW))


/***************************************************************************************************
* Display Configuration                                                                       *
***************************************************************************************************/

#define SPLASHTIME          2500            /**< Splash screen time (ms) */


/***************************************************************************************************
* Structure, union, and enumerated type definitions                                                *
***************************************************************************************************/
typedef  struct   progData {                /**< Program operating data structure */

        uint8_t  tempRunAwayDelta;
        uint8_t  tempRunAwayCycles;        
        uint16_t setTemp;						 /**< Current set temperature */
        uint16_t controlPeriod;             /**< Control period (ms) */
        uint16_t processInterval;           /**< Process interval (ms) */
        double heatingDuration;            /**< Process duration (ms) */
        uint8_t preToHeatTempOffset;        /**< Preheat to heating process state temperature offset to transition requirements */
        uint8_t preToHeatHoldTime;          /**< Preheat to heating process state required hold time to  meet transition requirements */
        uint16_t serialPrintInterval;       /**< Serial print interval (ms) */
        uint16_t  kp;                        /**< Proportional gain in milli */
        uint16_t  ki;                        /**< Integral gain in milli */
        uint16_t  kd;                        /**< Derivative gain in milli*/
        uint8_t  cp;                        /**< Derivative constant for dynamic tuning */
        uint8_t ci;                        /**< Integral constant for dynamic tuning */
        uint8_t  cd;                        /**< Derivative constant for dynamic tuning */
        uint8_t gapThreshold;               /**< Temperature gap threshold for dynamic tuning */
} progData;

/*
Fixed-width integer types and their usage:

1. Unsigned integers (only non-negative values):
   - uint8_t:  8-bit unsigned integer, range 0 to 255.
   - uint16_t: 16-bit unsigned integer, range 0 to 65,535.
   - uint32_t: 32-bit unsigned integer, range 0 to 4,294,967,295.
   - uint64_t: 64-bit unsigned integer, range 0 to 18,446,744,073,709,551,615.

2. Signed integers (positive and negative values):
   - int8_t:  8-bit signed integer, range -128 to 127.
   - int16_t: 16-bit signed integer, range -32,768 to 32,767.
   - int32_t: 32-bit signed integer, range -2,147,483,648 to 2,147,483,647.
   - int64_t: 64-bit signed integer, range -9,223,372,036,854,775,808 to 9,223,372,036,854,775,807.

Benefits of using fixed-width integers:
- Memory Efficiency: Allows for optimization of memory usage by selecting the smallest adequate type.
- Performance: Can enhance processing speed, especially in arrays or on certain architectures.
- Portability & Predictability: Ensures consistent behavior across different platforms.
- Code Clarity: Indicates the intended use and range of values, improving readability and maintenance.

These types are defined in the <cstdint> header and provide a standardized way to declare integers with specific sizes, ensuring consistent application behavior.
*/


/***************************************************************************************************
* Procedure prototypes                                                                             *
***************************************************************************************************/

// Process control and state handling
void handleSerialCommands();
void printSerialData();
void signalError(); // Placeholder for actual implementation
void systemChecks();
void thermalRunawayCheck(); // Placeholder for actual implementation
void logSD(); // Placeholder for actual implementation
void checkSleep();
void readCheckTemp();
void checkIsnan(double temp1, double temp2);
void processTimeManagement();
void slowPWM(int SSRn, unsigned long &cycleStart, double period, double output);
void dynamicTuning();

#ifdef _LCDGUI_
void encoderEvent();
void lcdUserStateMachine();
#endif /* _LCDGUI_ */

// EEPROM management functions
void resetEeprom(bool full);
void loadEeprom();
void updateEeprom();

#endif // _PRESS_CONTROL_v1_0_0_H

// EOF press_controller_v1_0_0.h
 
