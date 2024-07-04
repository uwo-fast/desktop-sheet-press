#ifndef _MAIN_H
#define _MAIN_H

/***************************************************************************************************
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
 *  \file       main.h
 *  \author     Cameron K. Brooks
 *  \version    1.1.0
 *  \date       Mar 2024
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
 *
 ***********************************************************************************************/


#define MILLI_UNIT 1000          /**< Milli unit for gain values */

// Macros to define logical states
#define DD_READ true /**< Data transfer direction - read */    // UN USED ???
#define DD_WRITE false /**< Data transfer direction - write */ // UN USED ???
#define P_ON true                                              /**< General macro for ON state */
#define P_OFF false                                            /**< General macro for OFF state */
#define PL_ACTIVE_H false                                      /**< Pin logic macro for Active High */
#define PL_ACTIVE_L true                                       /**< Pin logic macro for Active Low */
#define TC_FAULT false                                         /**< Thermocouple fault state */


typedef struct progData
{ /**< Program operating data structure */

    uint8_t tempRunAwayDelta;
    uint8_t tempRunAwayCycles;
    uint16_t setTemp;             /**< Current set temperature */
    uint16_t controlPeriod;       /**< Control period (ms) */
    uint16_t processInterval;     /**< Process interval (ms) */
    double heatingDuration;       /**< Process duration (ms) */
    uint8_t preToHeatTempOffset;  /**< Preheat to heating process state temperature offset to transition requirements */
    uint8_t preToHeatHoldTime;    /**< Preheat to heating process state required hold time to  meet transition requirements */
    uint16_t serialPrintInterval; /**< Serial print interval (ms) */
    uint16_t kp;                  /**< Proportional gain in milli */
    uint16_t ki;                  /**< Integral gain in milli */
    uint16_t kd;                  /**< Derivative gain in milli*/
    uint8_t cp;                   /**< Derivative constant for dynamic tuning */
    uint8_t ci;                   /**< Integral constant for dynamic tuning */
    uint8_t cd;                   /**< Derivative constant for dynamic tuning */
    uint8_t gapThreshold;         /**< Temperature gap threshold for dynamic tuning */
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

// Function Prototypes
void reset_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void setup();
void loop();



// EOF main.h
