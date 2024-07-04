#define _PRESS_CONTROL_v1_1_0

/**********************************************************************************************
 *  \par        Press Controller - Source File.
 *
 *  \par        Details
 *  \par
 *              Press Controller v1 Firmware.
 *              This is the Arduino Code for the Open Source Cold Hot Scientific Sheet Press.
 *  \par
 *              You need to have the following libraries installed in your Arduino IDE :
 *               - [leave blank fill later with all the libraries]
 *
 *              This release has been tested with the following library versions:
 *              [lib]  Vxx.yy.zz
 *
 *  \file       press_controller_v1_1_0.ino
 *  \author     Cameron K. Brooks <https://www.appropedia.org/User:CameronBrooks11>
 *  \authors	Morgan C. Woods <https://www.appropedia.org/User:M.Woods>
 * 					- First author of the research paper for which this code was prepared.
 * 				Joshua M. Pearce <https://www.appropedia.org/User:J.M.Pearce>
 *  					-  Supervising Professor for the research paper for which this code was prepared.
 *  \authors
 *  		    Marc Schönfleisch <http://malectrics.eu/>
 * 					- The creator of Arduino_Spot_Welder_V4_0_2.ino <https://github.com/KaeptnBalu/Arduino_Spot_Welder_V4>.
 * 					- Served as a key reference for the development of this code.
 *
 *  \version    1.1.0
 *  \date       Mar 2024
 *  \copyright  Copyright(c) 2024 Marc Schönfleisch <info@malectrics.eu>.
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
 *  \par
 *				The full license is available in the file LICENSE.md, distributed with this software.
 *  \par
 *  			This file is part of [Project Name]. It has been modified by Cameron K. Brooks
 *  			on 2024-MM-DD. The modifications are under the same GPL version 3
 *  			or later license as the original work.
 *
 *
 */
/***********************************************************************************************/

/***********************************************************************************************
 *  \par        Menu and display Screen Descriptions.
 *              The menus and dispay screens are arranged as follows;
 *  \verbatim
 *  \todo
 * 	INSERT HERE FRON THE SHEET : https://docs.google.com/spreadsheets/d/1ETsrqTxRlOqclLa2pnuBXwNIZhDpUWXC6jgvGGqIkTA/edit?usp=sharing
 *
 *  \endverbatim
 */
/***********************************************************************************************/

#include "press_controller_v1_2_0.h"

#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>

#include <SD.h>

// The EEPROM library, this is for storing data in the EEPROM memory aka non-volatile memory
#include <EEPROM.h>

// The PROGMEM library, this is for storing data in the program memory aka flash memory
#include <avr/pgmspace.h>

#include <PID_v1.h>
#include <Adafruit_MAX31855.h>

#ifdef _LCDGUI_
// The I2C LCD library
#include <LiquidCrystal_I2C.h>
// The menu wrapper library
#include <LiquidMenu.h>

// For Rotary Click Encoder
#include <ClickEncoder.h>
#include <TimerOne.h>
#endif /* _LCDGUI_ */

/***************************************************************************************************
 * Global program variables and objects                                                             *
 ***************************************************************************************************/

// Static variables
int drivingDeltaCounter1 = 0;
int drivingDeltaCounter2 = 0;
bool tempRunAwayAlarm1 = false;				  /**< Temperature runaway alarm for thermocouple 1 */
bool tempRunAwayAlarm2 = false;				  /**< Temperature runaway alarm for thermocouple 2 */
bool tc1Status = TC_FAULT;					  /**< Thermocouple 1 status */
bool tc2Status = TC_FAULT;					  /**< Thermocouple 2 status */
uint8_t errorFlagsMAX[] = {0, 0, 0, 0, 0, 0}; /**< Error flags for MAX31855 thermocouple sensor */
int16_t temp1;								  /**< Current temperature reading from thermocouple 1 */
int16_t temp2;								  /**< Current temperature reading from thermocouple 2 */
bool sdGood = false;						  /**< SD card status */
String currentFileName = "";				  /**< Current file name for data logging */
File dataFile;								  /**< Data file object for logging */
static unsigned long lastFlushTime = 0;
const unsigned long flushInterval = 5000; // Flush every 5 seconds

// Process Varaibles
unsigned long activeTime = 0;
unsigned long preheatingTime = 0;
unsigned long heatingTime = 0;

unsigned long preheatTransitionStartTime = 0; // Tracks when both temps first meet preheating criteria

unsigned long currentMillis = 0;
unsigned long lastMillis = 0;

unsigned long cycleStart1 = 0;
unsigned long cycleStart2 = 0;

unsigned long lastThermalRunawayCheck = 0; // Tracks the last thermal runaway check time

bool relay1ManualOff = false;
bool relay2ManualOff = false;




// ------------------------------------------------------
// SETUP -------------------------------------------------
// ------------------------------------------------------

/**
 * @brief Prepares MCU for startup by clearing reset flags and disabling the watchdog timer.
 *
 * Executes in the .init3 section during AVR startup, ensuring a clean state by:
 * - Clearing MCU Status Register (MCUSR) to remove any reset flags.
 * - Disabling the watchdog timer to prevent unintended resets at startup.
 *
 * Utilizes `__attribute__((naked))` to avoid standard prologue/epilogue and
 * `__attribute__((section(".init3")))` for early execution before main().
 */
void reset_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void reset_mcusr(void)
{
	MCUSR = 0;
	wdt_disable();
}



// Setup Main
void setup()
{
#ifdef _LCDGUI_
	// This needs to happen before the delay to allow the Timer1 to be set up before the
	// user needs to use it to possibly enter the system menu.
	encoder = new ClickEncoder(PIN_ENC_DT, PIN_ENC_CLK, PIN_ENC_SW, 4);

	Timer1.initialize(1 * MILLI_UNIT); // 1* MILLI_UNIT = 1,000ms = 1s
	Timer1.attachInterrupt(timerIsr);

	// This is the I2C LCD object initialization.
	lcd.init();
	lcd.backlight();

	// Load lines from PROGMEM, add lines, attach functions
	setupLcdLines();

	// Menu initialization.
	menu_system.update();
	menu_system.change_menu(setup_menu);

	unsigned long setupMenuMillis = 0;
	const long setupMenuInterval = 500; // interval at which to change screen (milliseconds)

	// This timing loop also the user the physical time for entering eeprom reset on GUI or to click and open serial monitor on PC
	// Display the setup screens
	for (int i = 0; i < 3; i++)
	{
		setupMenuMillis = millis();
		while (millis() - setupMenuMillis < setupMenuInterval)
		{
		}
		if (i != 0)
		{
			setup_menu.next_screen();
		}
	}
#endif /* _LCDGUI_ */

	// Initialize thermocouple pins
	pinMode(PIN_TC_CS1, OUTPUT);
	tc1Status = thermocoupleSetup(thermocouple1);

	pinMode(PIN_TC_CS2, OUTPUT);
	tc2Status = thermocoupleSetup(thermocouple2);

	// Set SSR pins as output
	pinMode(PIN_SSR1, OUTPUT);
	pinMode(PIN_SSR2, OUTPUT);

#ifdef _SERIALCMD_ || _DEVELOPMENT_
	Serial.begin(_SERIAL_BAUD_);
	while (!Serial)
	{
		; // wait for serial port to connect. Needed for native USB port only
	}

	Serial.println(F("Initializing TC sensor..."));

	if (tc1Status == TC_FAULT)
	{
		Serial.println(F("Thermocouple 1 ERROR, could not initialize."));
	}
	else
	{
		Serial.println(F("Thermocouple 1 initialized."));
	}
	if (tc2Status == TC_FAULT)
	{
		Serial.println(F("Thermocouple 2 ERROR, could not initialize."));
	}
	else
	{
		Serial.println(F("Thermocouple 2 initialized."));
	}
#endif /* _SERIALCMD_ || _DEVELOPMENT_ */

	// Initialze SD card
	Serial.print("Initializing SD card...");

	// see if the card is present and can be initialized:
	if (!SD.begin(SD_CS))
	{
		sdGood = false;
		Serial.println(F("SD Card failed, or not present"));
	}
	else
	{
		sdGood = true;
		Serial.println(F("SD Card initialized."));
	}

	// Applying the default PID tunings / settings
	// Set initial setpoint temperature
	Setpoint = pData.setTemp;

	/***********************
	Set PID controllers to automatic mode
	* In most applications, there is a desire to sometimes turn off the PID
	* controller and adjust the output by hand, without the controller interfering.
	* \todo
	* Add a switch to turn off the PID controller and adjust the output by hand
	* as a future feature to try and reverse thermal runaway proacivtively, rather than
	* just detecting and stopping it reactively.
	******************* */
	myPID1.SetMode(AUTOMATIC);
	myPID1.SetSampleTime(pData.controlPeriod);
	myPID1.SetOutputLimits(0, 255);

	myPID2.SetMode(AUTOMATIC);
	myPID2.SetSampleTime(pData.controlPeriod);
	myPID2.SetOutputLimits(0, 255);

// Test if the pushbutton is pressed at boot time. If so then ensure entry to the system
// menu by the issue of a boot button down event.
#ifdef _BOOTSYS_ &&_LCDGUI_
	uEvent = EV_BOOTDN;
#endif

#ifdef _LCDGUI_
	encoderEvent();
	uEvent == EV_BTN_HELD ? EV_BOOTDN : EV_NONE;
#endif /* _BOOTSYS_ && _LCDGUI_ */
	/** IMPORTANT FOR UNDERSTANDING THE EEPROM AND VARIABLE INITIALIZATION
	 * @brief Loads program data from EEPROM on startup.
	 *
	 * This function is called during the setup phase of the Arduino sketch to initialize
	 * the system with user settings and history stored in EEPROM. It includes a mechanism
	 * to determine whether the EEPROM contains valid data for this program.
	 *
	 * A unique identifier (magic number) is used at the beginning of the EEPROM to verify
	 * the data's validity. If the magic number does not match (indicating either a first-time
	 * program upload, data corruption, or previous use of the EEPROM by another program),
	 * the function resets the EEPROM to default program values.
	 *
	 * The use of a unique ID helps ensure that the EEPROM contains a valid data set belonging
	 * to this specific program. However, it is acknowledged that there are more robust methods
	 * for EEPROM data validation, which are not used here due to code space constraints.
	 *
	 * It's important to note that the reliability of this mechanism is based on the convention
	 * of starting EEPROM writes at the beginning of its address space. Deviating from this
	 * convention in subsequent programs might lead to a false positive validation of EEPROM
	 * data integrity, as the magic number might remain unaltered.
	 *
	 * @note This function is guaranteed to reset data to defaults only on the first upload
	 *       of the program. Subsequent uploads not adhering to the EEPROM writing convention
	 *       may inadvertently preserve the unique ID, leading to incorrect data validation.
	 */
	loadEeprom();
	// resetEeprom(EE_FULL_RESET);	// manual
#ifdef _LCDGUI_
	menu_system.change_menu(machine_status_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
#endif
}

// ------------------------------------------------------
// LOOP -------------------------------------------------
// ------------------------------------------------------


void loop()
{
	readCheckTemp();

#ifdef _LCDGUI_
	encoderEvent();
	updateLcdGui();
	checkSleep();
#endif

	logSD();

#ifdef _SERIALCMD_
	handleSerialCommands();
	printSerialData();
#endif

	updateEeprom();

	// Main process control
	// Update the current time in milliseconds
	currentMillis = millis();

	// Check if enough time has passed for the next process control loop
	if (currentMillis - lastMillis >= pData.processInterval)
	{
		switch (currentProcessState.getState())
		{
		case ACTIVE_PROCESS:
			thermalRunawayCheck();

			openNewFileIfNeeded();

			// Only run PID and slowPWM if activeProcess is true
			Setpoint = (double)pData.setTemp;
			dynamicTuning();

			Input1 = (double)temp1;
			myPID1.Compute();
			Input2 = (double)temp2;
			myPID2.Compute();

			if (relay1ManualOff)
			{
				Output1 = 0; // Set Output1 to zero if relay1ManualOff is true
			}

			if (relay2ManualOff)
			{
				Output2 = 0; // Set Output2 to zero if relay2ManualOff is true
			}

			slowPWM(PIN_SSR1, cycleStart1, pData.controlPeriod, Output1);
			slowPWM(PIN_SSR2, cycleStart2, pData.controlPeriod, Output2);

			switch (currentActiveProcessSubstate.getSubstate())
			{
			case PREHEATING:
				preheatingTime += pData.processInterval;
				// Check if both temperatures are equal to or greater than setTemp - offset
				if (temp1 >= pData.setTemp - pData.preToHeatTempOffset && temp2 >= pData.setTemp - pData.preToHeatTempOffset)
				{
					// If preheatTransitionStartTime is 0, this is the first loop iteration where both temps meet the criteria
					if (preheatTransitionStartTime == 0)
					{
						preheatTransitionStartTime = currentMillis; // Start tracking time
					}
					else if (currentMillis - preheatTransitionStartTime >= pData.preToHeatHoldTime)
					{
						// Conditions have been met for 10 seconds, transition to HEATING
						currentActiveProcessSubstate.setSubstate(HEATING);
						heatingTime = 0;				// Reset or start tracking processing time
						preheatTransitionStartTime = 0; // Reset preheat start time for next use
					}
				}
				else
				{
					// If conditions are not met, reset the start time
					preheatTransitionStartTime = 0;
				}
				break;
			case HEATING:
				// Accumulate heating process duration
				heatingTime += pData.processInterval;

				// Check if process duration has been met
				if (heatingTime >= pData.heatingDuration)
				{
					currentActiveProcessSubstate.setSubstate(COOLING_DOWN);
				}
				break;
			case COOLING_DOWN:
				currentProcessState.setState(INACTIVE_PROCESS); // hardcoded at the moment, present system doesnt have active cooling
				currentActiveProcessSubstate.setSubstate(UNKNOWN);
				break;
			default:
				currentProcessState.setState(INACTIVE_PROCESS);
				break;
			}
			break;

		case INACTIVE_PROCESS:
			// If the process is not active, just read temperatures and write SSRs LOW

			Setpoint = 0;
			Output1 = 0;
			Output2 = 0;
			digitalWrite(PIN_SSR1, LOW);
			digitalWrite(PIN_SSR2, LOW);
			closeFileIfNeeded();

			break;

		case ERROR_PROCESS:

			Setpoint = 0;
			Output1 = 0;
			Output2 = 0;
			// Handle error condition, perhaps by setting SSRs to LOW and signaling error
			digitalWrite(PIN_SSR1, LOW);
			digitalWrite(PIN_SSR2, LOW);

			closeFileIfNeeded();
			// Signal error condition, e.g., by blinking an LED or sending an error message
			signalError(); // TODO

			break;

		default: // STANDBY_PROCESS & ALL ELSE

			Setpoint = 0;
			Output1 = 0;
			Output2 = 0;
			// Default action, perhaps similar to INACTIVE_PROCESS or specific safe state
			digitalWrite(PIN_SSR1, LOW);
			digitalWrite(PIN_SSR2, LOW);
			closeFileIfNeeded();

			break;
		}
		lastMillis = currentMillis;
	}
}


