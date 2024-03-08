#ifndef _PRESS_CONTROL_v1_0_0
#define _PRESS_CONTROL_v1_0_0

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

// General macros
#define str(s)              #s
#define xstr(s)             str(s)

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

#define _LANG_EN_                           /**< Language:  _LANG_EN/DE/FR/ES/IT_ */
#define _SERIAL_BAUD_       115200          /**< Comms rate for serial */

/***************************************************************************************************
* Pin and interrupt definitions                                                                    *
***************************************************************************************************/

#define ENC_INT                 0               /**< Rotary interrupt for CLK input (Ph0) */
#define PIN_ENC_CLK             2               /**< Rotary encoder CLK input (Ph0) */
#define PIN_ENC_DT              8               /**< Rotary encoder DT input (Ph1) */
#define PIN_ENC_SW              6               /**< Rotary encoder push button switch input */
#define TC_CS1                  5               /**< Thermocouple 1 SPI chip select */
#define TC_CLK                  6               /**< Thermocouple SPI clock */
#define TC_CS2                  7               /**< Thermocouple 2 SPI chip select */
#define TC_DO                   8               /**< Thermocouple SPI data out */
#define PIN_SSR1                11               /**< Relay 1 output | 14 - A0 */
#define PIN_SSR2                12               /**< Relay 2 output | 15 - A1 */

/***************************************************************************************************
* Macros                                                                                           *
***************************************************************************************************/
// Defaults for operational variables
#define DEF_TEMP_GAP_ALARM  65              /**< Default high temperature */
#define DEF_TEMP_GAP_ALARM  65              /**< Default high temperature */
#define DEF_OLED_INVERT     false           /**< Default OLED orientation */ 

// Limits for operational variables
#define MIN_PULSE_TIME      1               /**< Minimum weld pulse time */
#define MAX_PULSE_TIME      500             /**< Absolute maximum weld pulse time */
#define MAX_APULSE_DELAY    50              /**< Maximum auto pulse delay */
#define MIN_APULSE_DELAY    5               /**< Minimum auto pulse delay */
#define MAX_SPULSE_TIME     100             /**< Maximum short pulse time */
#define MIN_SPULSE_TIME     0               /**< Minimum short pulse time */
#define MAX_BATT_ALARM      120             /**< Maximum low battery alarm voltage */
#define MIN_BATT_BALARM     74              /**< Minimum low battery alarm voltage */
#define MAX_BATT_V          200             /**< Absolute maximum battery voltage */
#define MIN_BATT_V          50              /**< Absolute minimum battery voltage */

// Timing macros
#define STANDBY_TIME_OUT    300000L         /**< Device sleep timeout (ms) */ 
#define EEPROM_UPDATE_T     5000            /**< EEPROM update time (ms) */
#define RS_DEBOUNCE         50  /*20*/      /**< Rotary encoder & switch debounce time (ms) */
#define T_INTERVAL          10000           /**< temperature measurement interval (ms) */


// Macros to define logical states
#define DD_READ             true            /**< Data transfer direction - read */ // UN USED ???
#define DD_WRITE            false           /**< Data transfer direction - write */ // UN USED ???
#define P_ON                true            /**< General macro for ON state */
#define P_OFF               false           /**< General macro for OFF state */
#define B_DN                true            /**< General macro for DOWN state */
#define B_UP                false           /**< General macro for UP state */
#define PL_ACTIVE_H         false           /**< Pin logic macro for Active High */
#define PL_ACTIVE_L         true            /**< Pin logic macro for Active Low */

// EEPROM macros
#define EEA_ID              0               /**< Address of unique ID */
#define EEA_PDATA           (EEA_ID+4)      /**< Eeprom address of program data */
#define EE_UNIQUEID         0x18fae9c8      /**< Unique Eeprom verification ID, arbitrary */
#define EE_FULL_RESET       true            /**< Reset parameter to reset all Eeprom parameters */

// Macros masquerading as functions - makes the code more readable
/** This macro reads the state of the pushbutton switch on the encoder. */
#define btnState()          (!digitalRead(PIN_SW))

/** This macro drives the welding pulse. */
#define weldPulse(state)    digitalWrite(PIN_PULSE,state?HIGH:LOW)

/** Where has this macro gone?? It was in WString.h */
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))               

/***************************************************************************************************
* OLED Display Configuration                                                                       *
***************************************************************************************************/

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET          4               /**< OLED mode */
#define OLED_INVERT         2               /**< OLED defined orientation mode - check OLED doc'n */
#define SPLASHTIME          2500            /**< Splash screen time (ms) */


/***************************************************************************************************
* Structure, union, and enumerated type definitions                                                *
***************************************************************************************************/

typedef  enum {                             /**< Type enumerations for format of variables */
         VF_BATTALM,                        /**< Battery alarm voltage */
         VF_TEMPALM,                        /**< Temperature Alarm value */
         VF_BATTV,                          /**< Battery voltage */
         VF_BATTA,                          /**< Battery Amps */
         VF_TEMP,                           /**< Temperature */
         VF_WELDCNT,                        /**< Weld count */
         VF_PLSDLY,                         /**< Pulse delay */
         VF_SHTPLS,                         /**< Short pulse duration */
         VF_DELAY                           /**< Delay */
} vf_Type;

typedef  struct   progData {                /**< Program operating data structure */
         uint8_t  autoPulseDelay;           /**< Auto-pulse delay (ms/100) */ 
         uint8_t  batteryAlarm;             /**< Low battery voltage (A/D count) */
         uint8_t  batteryhighAlarm;         /**< High battery voltage (A/D count) */
         uint8_t  TCelsius;                 /**< Temperature in Celsius */
         uint8_t  maxTCelsius;              /**< maximum Temperature in Celsius */
         uint16_t weldCount;                /**< Count of welds performed */
         uint16_t pulseTime;                /**< Pulse time (ms) */ 
         uint16_t maxPulseTime;             /**< Maximum allowed pulse time (ms) */ 
         uint8_t  shortPulseTime;           /**< Short pulse time (% of pulse time) */ 
         int8_t   batteryOffset;            /**< Battery voltage calibration offset (signed) x10 */
         uint16_t  PulseBatteryVoltage;     /**< Battery voltage during pulse x10 */
         uint16_t  PulseAmps;               /**< esimated Amps during pulse x10 */  
} progData;

/***************************************************************************************************
* Procedure prototypes                                                                             *
***************************************************************************************************/

void     stateMachine();

void     resetEeprom(boolean = false);
void     loadEeprom();
void     updateEeprom();

void     checkForLowVoltageEvent();
void     checkForSleepEvent();
void     checkForBtnEvent();
void     checkTemp();
void     foot_switch_error();
void     FootSwitch_Alarm();
void     Boot_Sound();
void     LowBattery_Sound();
void     isr();
void     splash();
void     sendWeldPulse(uint8_t, uint16_t, uint16_t, boolean = PL_ACTIVE_H);  
void     message(const __FlashStringHelper*, const __FlashStringHelper*,
                 const __FlashStringHelper*, uint8_t = 0);
void     displayMenuType1(const __FlashStringHelper*, const __FlashStringHelper*,
                          const __FlashStringHelper*, const __FlashStringHelper*, 
                          uint8_t SelectedItem);
void     displayMenuType2(const __FlashStringHelper*, const char*, const __FlashStringHelper*);
void     displayMainScreen();
void     displayPulseData();
void     displayLowBattery();
void     displayHighBattery();
void     displayHighTemperature();
void     drawStatusLine();
void     setTextProp(uint8_t, uint8_t, uint8_t, uint16_t = WHITE, boolean = false);
char*    valStr(char*, uint16_t, vf_Type);

#endif // _ARDUINO_SPOT_WELDER_V3_H

// EOF Arduino_Spot_Welder_V3.h
 
