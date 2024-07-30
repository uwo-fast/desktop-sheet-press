#define _PRESS_CONTROL_v1_0_0

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
 *  \file       press_controller_v1_0_0.ino
 *  \author     Cameron K. Brooks <https://www.appropedia.org/User:CameronBrooks11>
 *  \authors	Morgan C. Woods <https://www.appropedia.org/User:M.Woods>
 * 					- First author of the research paper for which this code was prepared.
 * 				Joshua M. Pearce <https://www.appropedia.org/User:J.M.Pearce>
 *  					-  Supervising Professor for the research paper for which this code was prepared.
 *  		    Marc Schönfleisch <http://malectrics.eu/>
 * 					- The creator of Arduino_Spot_Welder_V4_0_2.ino <https://github.com/KaeptnBalu/Arduino_Spot_Welder_V4>.
 * 					- Served as a key reference for the development of this code.
 *
 *  \version    1.0.0
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

#include "press_controller_v1_0_0.h"

// Libraries for PID and Thermocouples
#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>

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
int8_t temp1;								  /**< Current temperature reading from thermocouple 1 */
int8_t temp2;								  /**< Current temperature reading from thermocouple 2 */
int8_t lastValidTemp1;						  /**< Last valid temperature reading from thermocouple 1 */
int8_t lastValidTemp2;						  /**< Last valid temperature reading from thermocouple 2 */
bool validTemp1 = false;					  /**< Flag indicating if the temperature reading from thermocouple 1 is valid */
bool validTemp2 = false;					  /**< Flag indicating if the temperature reading from thermocouple 2 is valid */

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

// PID Process Parameters
double Setpoint, Input1, Output1, Input2, Output2;

// Timing variable for the serial print interval, for serial command mode
#ifdef _SERIALCMD_
unsigned long lastSerialPrint = 0;
#endif /* _SERIALCMD_ */

// LCD State and Event variables
#ifdef _LCDGUI_
uint8_t uEvent;				   /**< Current pending user event */
uint8_t uToggle;			   /**< Current user toggle */
int16_t encLastPos, encNewPos; /**< Encoder position variables */
unsigned long lastActiveTime;
#endif /* _LCDGUI_ */

// Structures and objects
progData pData; /**< Program operating data */

ProcessStateWrapper currentProcessState(INACTIVE_PROCESS);

ActiveProcessSubstateWrapper currentActiveProcessSubstate(UNKNOWN);

const char *getCurrentProcessState()
{
	return currentProcessState.toChar();
}

const char *getCurrentActiveProcessSubstate()
{
	return currentActiveProcessSubstate.toChar();
}

const char *getStateInfo()
{
	static char buffer[16];
	unsigned long output = static_cast<unsigned long>(Output1) * 0.5 + static_cast<unsigned long>(Output2) * 0.5;

	if (strcmp(getCurrentProcessState(), "ACTIVE") == 0)
	{
		// Concatenate first two letters, setTemp, "C", and output without spaces
		sprintf(buffer, "%dC O:%lu", pData.setTemp, output);
	}
	else
	{
		// For other states, just show the first two letters of the state
		sprintf(buffer, "%s", getCurrentProcessState());
	}
	return buffer;
}

const char *getStatusInfo()
{
	static char buffer[10]; // Buffer to hold the formatted string

	if (strcmp(getCurrentProcessState(), "ACTIVE") == 0)
	{
		// Ensure heatingDuration is treated accurately as milliseconds in integer math
		unsigned long heatingDurationMillis = static_cast<unsigned long>(pData.heatingDuration);
		unsigned long remainingTime = heatingDurationMillis - heatingTime; // Remaining time in milliseconds
		unsigned long hours = remainingTime / (3600000UL);				   // 1000 * 60 * 60
		unsigned long minutes = (remainingTime % (3600000UL)) / (60000UL); // (1000 * 60)
		unsigned long percentCompletion = (heatingTime * 100UL) / heatingDurationMillis;

		sprintf(buffer, "%lu%% %02lu:%02lu", percentCompletion, hours, minutes); // Format as "XX% HH:MM"
	}
	else
	{
		strcpy(buffer, "---% --:--"); // Set the string for non-ACTIVE states
	}

	return buffer;
}

const char *getDurationInfo()
{
	static char buffer[5];

	// Ensure heatingDuration is treated accurately as milliseconds in integer math
	unsigned long heatingDurationMillis = static_cast<unsigned long>(pData.heatingDuration);
	unsigned long hours = heatingDurationMillis / (3600000UL);				   // 1000 * 60 * 60
	unsigned long minutes = (heatingDurationMillis % (3600000UL)) / (60000UL); // (1000 * 60)

	sprintf(buffer, "%02lu:%02lu", hours, minutes); // Format as "XX% HH:MM"
	return buffer;
}

const char *getSetTempInfo()
{
	static char buffer[16];

	// Get the current set temperature from pData structure
	unsigned int setTemp = pData.setTemp;

	sprintf(buffer, "Set Temp: %u C", setTemp); // Format as "Set Temp: ### C"
	return buffer;
}

Adafruit_MAX31855 thermocouple1(PIN_TC_CLK, PIN_TC_CS1, PIN_TC_DO); /**< Thermocouple 1 object */
Adafruit_MAX31855 thermocouple2(PIN_TC_CLK, PIN_TC_CS2, PIN_TC_DO); /**< Thermocouple 2 object */

#ifdef _LCDGUI_
ClickEncoder *encoder;				/**< Encoder object */
LiquidCrystal_I2C lcd(0x27, 16, 2); /**< LCD display object */

// Declare strings to store in flash memory
const char fastResearchText[] PROGMEM = "FAST Research";
const char oschssPressText[] PROGMEM = "OSCHSS Press";
const char versionText[] PROGMEM = "Version:";
const char versionNumberText[] PROGMEM = "1.0.0";
const char licenseText[] PROGMEM = "License:";
const char gplv3FreeText[] PROGMEM = "GPLv3 \"FREE\"";

// SETUP DISPLAYS
LiquidLine setup_line1(0, 0, fastResearchText);
LiquidLine setup_line2(0, 1, oschssPressText);
LiquidLine setup_line3(0, 0, versionText);
LiquidLine setup_line4(0, 1, versionNumberText);
LiquidLine setup_line5(0, 0, licenseText);
LiquidLine setup_line6(0, 1, gplv3FreeText);
LiquidScreen setup_screenA(setup_line1, setup_line2);
LiquidScreen setup_screenB(setup_line3, setup_line4);
LiquidScreen setup_screenC(setup_line5, setup_line6);
LiquidMenu setup_menu(lcd, setup_screenA, setup_screenB, setup_screenC);

const char t1StatusScreenText[] PROGMEM = "1:";
// MAIN DISPLAY MACHINE STATUS
LiquidLine machine_status_line1(0, 0, t1StatusScreenText, temp1, "C ", getStateInfo);
LiquidLine machine_status_line2(0, 1, "2:", temp2, "C ", getStatusInfo);
LiquidScreen machine_status_screenA(machine_status_line1, machine_status_line2);
LiquidMenu machine_status_menu(lcd, machine_status_screenA);

const char statusScreenText[] PROGMEM = "/Status Screen";
const char durationText[] PROGMEM = "Duration: ";
const char activateText[] PROGMEM = "Activate";
const char stopText[] PROGMEM = "Stop";

// MAIN DISPLAY MAIN MENU
LiquidLine main_line0(0, 0, statusScreenText);
LiquidLine main_line1(0, 1, getSetTempInfo);
LiquidLine main_line2(0, 1, durationText, getDurationInfo);
LiquidLine main_line3(0, 1, activateText);
LiquidLine main_line4(0, 1, stopText);
LiquidScreen main_screen;

LiquidMenu main_menu(lcd, main_screen);

// Menu system
LiquidSystem menu_system(setup_menu, machine_status_menu, main_menu);

void setupLcdLines()
{
	setup_line1.set_asProgmem(1);
	setup_line2.set_asProgmem(1);
	setup_line3.set_asProgmem(1);
	setup_line4.set_asProgmem(1);
	setup_line5.set_asProgmem(1);
	setup_line6.set_asProgmem(1);

	machine_status_line1.set_asProgmem(1);
	machine_status_line1.attach_function(FUNC_ENTER_MENU, goto_main_menu);
	machine_status_line1.set_focusPosition(Position::CUSTOM, 16, 0);

	machine_status_line2.attach_function(FUNC_ENTER_MENU, goto_main_menu);
	machine_status_line2.set_focusPosition(Position::CUSTOM, 16, 0);

	// Add lines and attach functions to the main menu
	main_line0.set_asProgmem(1);
	main_line1.set_asProgmem(1);
	main_line2.set_asProgmem(1);
	main_line3.set_asProgmem(1);
	main_line4.set_asProgmem(1);
	main_screen.add_line(main_line0);
	main_screen.add_line(main_line1);
	main_screen.add_line(main_line2);
	main_screen.add_line(main_line3);
	main_screen.add_line(main_line4);
	main_screen.set_displayLineCount(2);
	main_line0.attach_function(FUNC_ENTER_MENU, goto_machine_status_menu);
	main_line1.attach_function(FUNC_ENTER_MENU, toggle_function);
	main_line1.attach_function(FUNC_DECRT_PDATA, decrt_pdata_setTemp);
	main_line1.attach_function(FUNC_INCRT_PDATA, incrt_pdata_setTemp);
	main_line2.attach_function(FUNC_ENTER_MENU, toggle_function);
	main_line2.attach_function(FUNC_DECRT_PDATA, decrt_pdata_heatingDuration);
	main_line2.attach_function(FUNC_INCRT_PDATA, incrt_pdata_heatingDuration);
	main_line3.attach_function(FUNC_ENTER_MENU, activate_proc);
	main_line4.attach_function(FUNC_ENTER_MENU, deactivate_proc);
}

void timerIsr()
{
	encoder->service();
}

#endif /* _LCDGUI_ */

// PID object setup for both thermocouples
// PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint, DEF_KP / MILLI_UNIT, DEF_KI / MILLI_UNIT, DEF_KD / MILLI_UNIT, DIRECT); /**< PID1 object gets input from thermocouple 1 and ouputs to relay 1*/
PID myPID2(&Input2, &Output2, &Setpoint, DEF_KP / MILLI_UNIT, DEF_KI / MILLI_UNIT, DEF_KD / MILLI_UNIT, DIRECT); /**< PID2 object gets input from thermocouple 2 and ouputs to relay 2*/

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

// Setup helpers
bool thermocoupleSetup(Adafruit_MAX31855 &thermocouple)
{
	// Initializing the setup error flag
	bool tcInitFlag = false;

	// Check if thermocouple initialization fails
	if (!thermocouple.begin())
	{
		tcInitFlag = false;
	}
	else
	{
		tcInitFlag = true; // replace with macro
	}
	return tcInitFlag;
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

	// Initialize thermocouples
	pinMode(PIN_TC_CS1, OUTPUT);
	pinMode(PIN_TC_CS2, OUTPUT);

	// Set SSR pins as output
	pinMode(PIN_SSR1, OUTPUT);
	pinMode(PIN_SSR2, OUTPUT);

	tc1Status = thermocoupleSetup(thermocouple1);
	tc2Status = thermocoupleSetup(thermocouple2);

#ifdef _SERIALCMD_ || _DEVELOPMENT_ || defined _BOOTSYS_
	Serial.begin(_SERIAL_BAUD_);

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
#endif /* _SERIALCMD_ || _DEVELOPMENT_ || _BOOTSYS_*/

	// Applying the default PID tunings / settings
	// Set initial pData.setpoint temperature
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
	myPID2.SetMode(AUTOMATIC);
	// Set PID sample time
	myPID1.SetSampleTime(pData.controlPeriod);
	myPID2.SetSampleTime(pData.controlPeriod);
	myPID1.SetOutputLimits(0, 255);
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

	menu_system.change_menu(machine_status_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
}

// ------------------------------------------------------
// LOOP -------------------------------------------------
// ------------------------------------------------------

unsigned int lcdRefreshPeriod = 300;
unsigned long lcdLastRefresh = 0;

void updateLcdGui()
{
#ifdef _LCDGUI_
	if (millis() - lcdLastRefresh > lcdRefreshPeriod)
	{
		lcdEventHandler();
		lcdLastRefresh = millis();
		menu_system.update();
	}
#endif /* _LCDGUI_ */
}

void loop()
{
	readCheckTemp();

	encoderEvent();
	updateLcdGui();
	checkSleep();

	handleSerialCommands();
	printSerialData();

	logSD(); // TODO

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

			// Only run PID and slowPWM if activeProcess is true
			Input1 = (double)temp1;
			Input2 = (double)temp2;
			Setpoint = (double)pData.setTemp;

			dynamicTuning();
			myPID1.Compute();
			myPID2.Compute();
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

			break;

		case ERROR_PROCESS:

			Setpoint = 0;
			Output1 = 0;
			Output2 = 0;
			// Handle error condition, perhaps by setting SSRs to LOW and signaling error
			digitalWrite(PIN_SSR1, LOW);
			digitalWrite(PIN_SSR2, LOW);
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

			break;
		}
		lastMillis = currentMillis;
	}
}

/* -------------------------------Signal Error----------------------------------*/
/**
 *  \brief    Signals an error condition.
 *  \remarks
 *
 */
void signalError() // TODO
{
	// Implement continuous error signaling in LCD corner and serial output
	// on first pass log dumb to SD card and serial error message. record in eeprom
}

/* -------------------------------Thermal Runaway Check----------------------------------*/
/**
 *  \brief    Checks for thermal runaway.
 *  \remarks This function checks for thermal runaway by comparing the temperature readings
 *  to the pData.setpoint plus a delta value. If the temperature readings exceed the pData.setpoint plus
 *  the delta value for a certain number of cycles and if the output for that temperature
 *  is not issuing a zero output, a thermal runaway event is issued.
 * 	The drivingDeltaCounter holds how many cycles the temperature has been above the pData.setpoint plus the
 *  delta value while still driving the output. If the temperature is below the pData.setpoint plus the delta
 *  value, the counter is reset to zero. If the counter reaches the number of cycles set in the program
 *  data, a thermal runaway event is issued. drivingDeltaCounter only increases once per control period.
 */
void thermalRunawayCheck() // TODO
{

	unsigned long currentMillis = millis();
	// Check if enough time has passed for the next thermal runaway check
	if (currentMillis - lastThermalRunawayCheck >= pData.controlPeriod)
	{
		// The actual thermal runaway check logic remains the same
		if (temp1 > pData.setTemp + pData.tempRunAwayDelta || temp2 > pData.setTemp + pData.tempRunAwayDelta)
		{
			// Check if the output is not zero
			if ((int)Output1 > 0)
			{
				tempRunAwayAlarm1 = true;
				drivingDeltaCounter1++;
			}

			if ((int)Output2 > 0)
			{
				tempRunAwayAlarm2 = true;
				drivingDeltaCounter2++;
			}
		}
		else
		{
			tempRunAwayAlarm1 = false;
			tempRunAwayAlarm2 = false;
			drivingDeltaCounter1 = 0;
			drivingDeltaCounter2 = 0;
		}

		if (drivingDeltaCounter1 >= pData.tempRunAwayCycles || drivingDeltaCounter2 >= pData.tempRunAwayCycles)
		{
			currentProcessState.setState(ERROR_PROCESS);
			// Signal error condition
			signalError();
		}

		// Update the last check time
		lastThermalRunawayCheck = currentMillis;
	}
}

/* -------------------------------Log Data to SD----------------------------------*/
/**
 *  \brief    Logs data to the SD card.
 *  \remarks  Logs data to the SD card.
 */
void logSD() // TODO
{
}

/* -------------------------------Read and Check Temperature----------------------------------*/
/**
 *  \brief    Reads the temperature from the thermocouples and checks for errors.
 *  \remarks
 *  \todo implement a function to compute and report on temperature readings error ratio
 */
void readCheckTemp()
{
	temp1 = thermocouple1.readCelsius();
	temp2 = thermocouple2.readCelsius();

	// Read error flags from thermocouple 1 and generate error code
	uint8_t e1 = thermocouple1.readError();
	errorFlagsMAX[0] = (e1 & MAX31855_FAULT_OPEN) ? 1 : 0;
	errorFlagsMAX[1] = (e1 & MAX31855_FAULT_SHORT_GND) ? 1 : 0;
	errorFlagsMAX[2] = (e1 & MAX31855_FAULT_SHORT_VCC) ? 1 : 0;
	// Read error flags from thermocouple 2 and generate error code
	uint8_t e2 = thermocouple2.readError();
	errorFlagsMAX[3] = (e2 & MAX31855_FAULT_OPEN) ? 1 : 0;
	errorFlagsMAX[4] = (e2 & MAX31855_FAULT_SHORT_GND) ? 1 : 0;
	errorFlagsMAX[5] = (e2 & MAX31855_FAULT_SHORT_VCC) ? 1 : 0;

	// If temperature reading is NaN, replace with last valid reading
	temp1 = isnan(temp1) ? lastValidTemp1 : temp1;
	temp2 = isnan(temp2) ? lastValidTemp2 : temp2;
	// Update last valid temperature readings
	lastValidTemp1 = temp1;
	lastValidTemp2 = temp2;
}

// Function to implement slow PWM for SSR control
void slowPWM(int SSRn, unsigned long &cycleStart, double period, double output)
{
	// Get current time in milliseconds
	unsigned long currentMillis = millis();

	double dutyCycle = output / 255;

	// If within the ON part of the cycle, turn the SSR on
	if (currentMillis - cycleStart < period * dutyCycle)
	{
		digitalWrite(SSRn, HIGH);
	}
	// If within the OFF part of the cycle, turn the SSR off
	else if (currentMillis - cycleStart >= period * dutyCycle && currentMillis - cycleStart < period)
	{
		digitalWrite(SSRn, LOW);
	}
	// If the cycle is complete, reset the cycle start time
	else
	{
		cycleStart = currentMillis;
	}
}

// Optional function to dynamically adjust PID tuning parameters based on gap to pData.setpoint
void dynamicTuning()
{
	double gap1 = abs(Setpoint - Input1);
	if (gap1 < pData.gapThreshold)
	{ // Less aggressive tuning parameters for small gap
		myPID1.SetTunings(pData.kp / pData.cp / MILLI_UNIT, pData.ki / pData.ci / MILLI_UNIT, pData.kd / pData.cd / MILLI_UNIT);
	}
	else
	{ // More aggressive tuning parameters for large gap
		myPID1.SetTunings(pData.kp / MILLI_UNIT, pData.ki / MILLI_UNIT, pData.kd / MILLI_UNIT);
	}

	double gap2 = abs(Setpoint - Input2);
	if (gap2 < pData.gapThreshold)
	{ // Less aggressive tuning parameters for small gap
		myPID2.SetTunings(pData.kp / pData.cp / MILLI_UNIT, pData.ki / pData.ci / MILLI_UNIT, pData.kd / pData.cd / MILLI_UNIT);
	}
	else
	{ // More aggressive tuning parameters for large gap
		myPID2.SetTunings(pData.kp / MILLI_UNIT, pData.ki / MILLI_UNIT, pData.kd / MILLI_UNIT);
	}
}

/* --------------------------------------------------------------------------------------*/
/*------------------------------------- _SERIALCMD_--------------------------------------*/
/* --------------------------------------------------------------------------------------*/

// Function to allow serial navigation and control
void handleSerialCommands()
{
#ifdef _SERIALCMD_ || _DEVELOPMENT_ || defined _BOOTSYS_
	static String received = "";
	while (Serial.available() > 0)
	{
		char inChar = (char)Serial.read();
		received += inChar;
		if (inChar == '\n')
		{					 // when a complete command is received
			received.trim(); // remove potential leading/trailing white space
			if (received.startsWith("t1="))
			{
				temp1 = (uint8_t)received.substring(3).toInt();
			}
			else if (received.startsWith("t2="))
			{
				temp2 = (uint8_t)received.substring(3).toInt();
			}
			else if (received.startsWith("st="))
			{
				pData.setTemp = (uint8_t)received.substring(3).toInt();
			}
			else if (received.startsWith("o1="))
			{
				Output1 = (uint8_t)received.substring(3).toInt();
			}
			else if (received.startsWith("o2="))
			{
				Output2 = (uint8_t)received.substring(3).toInt();
			}
			else if (received.startsWith("dt="))
			{
				pData.heatingDuration = (received.substring(3).toDouble() * MILLI_UNIT * 60);
			}
			else if (received.startsWith("kp="))
			{
				pData.kp = (uint16_t)(received.substring(3).toFloat() * MILLI_UNIT);
			}
			else if (received.startsWith("ki="))
			{
				pData.ki = (uint16_t)(received.substring(3).toFloat() * MILLI_UNIT);
			}
			else if (received.startsWith("kd="))
			{
				pData.kd = (uint16_t)(received.substring(3).toFloat() * MILLI_UNIT);
			}
			else if (received == "ON")
			{
				currentProcessState.setState(ACTIVE_PROCESS);
				currentActiveProcessSubstate.setSubstate(PREHEATING);
			}
			else if (received == "OFF")
			{
				currentProcessState.setState(INACTIVE_PROCESS);
				currentActiveProcessSubstate.setSubstate(UNKNOWN);
			}

			received = ""; // clear received data
		}
	}
#endif /* _SERIALCMD_ || _DEVELOPMENT_ || _BOOTSYS_ */
}

void printSerialData()
{
#ifdef _SERIALCMD_ || _DEVELOPMENT_ || defined _BOOTSYS_

	if (millis() - lastSerialPrint > pData.serialPrintInterval)
	{
		Serial.print("MAX flags: ");

		int errorFlagsMAXSize = sizeof(errorFlagsMAX) / sizeof(errorFlagsMAX[0]);
		// Print error flags
		for (int i = 0; i < errorFlagsMAXSize; i++)
		{
			Serial.print(errorFlagsMAX[i]);
		}
		Serial.print(F(", TRA1 Alarm: "));
		Serial.print(tempRunAwayAlarm1 ? "WARNING" : "SAFE");
		Serial.print(", TRA2 Alarm: ");
		Serial.print(tempRunAwayAlarm2 ? "WARNING" : "SAFE");
		Serial.print(F(", t1: "));
		Serial.print(temp1);
		Serial.print(F("C, t2: "));
		Serial.print(temp2);
		Serial.print("C, st: ");
		Serial.print(pData.setTemp);
		Serial.print(F("C, o1: "));
		Serial.print((int)Output1 < 10 ? "00" : (int)Output1 < 100 ? "0"
																   : "");
		Serial.print((int)Output1);
		Serial.print(", o2: ");
		Serial.print((int)Output2 < 10 ? "00" : (int)Output2 < 100 ? "0"
																   : "");
		Serial.print((int)Output2);
		Serial.print(F(", Preheat t: "));
		// \todo instead of floats, use a function to convert to string and shift float point
		Serial.print((float)preheatingTime / MILLI_UNIT / 60, 2);
		Serial.print(F("m, Heat t: "));
		Serial.print((float)heatingTime / MILLI_UNIT / 60, 2);
		Serial.print(F("m, Total t: "));
		Serial.print(((float)(preheatingTime + heatingTime) / MILLI_UNIT / 60), 2); // Total time in minutes
		Serial.print(F("m, dt: "));
		Serial.print((float)pData.heatingDuration / MILLI_UNIT / 60);
		Serial.print(F("m, Kp: "));
		Serial.print((float)pData.kp / MILLI_UNIT);
		Serial.print(F(", Ki: "));
		Serial.print((float)pData.ki / MILLI_UNIT);
		Serial.print(F(", Kd: "));
		Serial.print((float)pData.kd / MILLI_UNIT); // Use println to add newline at the end
		Serial.print(F(", State: "));
		Serial.print(currentProcessState.toString());
		Serial.print(F(", Substate: "));
		Serial.print(currentActiveProcessSubstate.toString());
		Serial.println();

		lastSerialPrint = millis();
	}

#endif /* _SERIALCMD_ || _DEVELOPMENT_ || _BOOTSYS_*/
}

/* --------------------------------------------------------------------------------------*/
/*--------------------------------------- LCD_GUI ---------------------------------------*/
/* --------------------------------------------------------------------------------------*/

/* -------------------------------LCD Event Handler----------------------------------*/
void lcdEventHandler()
{

	switch (uEvent)
	{
	case (EV_NONE):
		break;
	case (EV_BTN_CLICKED):
		menu_system.call_function(FUNC_ENTER_MENU);
		uEvent = EV_NONE;
		break;
	case (EV_ENCUP):
		if (uToggle == TOGGLE_OFF)
		{
			menu_system.switch_focus(false);
			uEvent = EV_NONE;
		}
		else
		{
			menu_system.call_function(FUNC_DECRT_PDATA);
			uEvent = EV_NONE;
		}
		break;
	case (EV_ENCDN):
		if (uToggle == TOGGLE_OFF)
		{
			menu_system.switch_focus(true);
			uEvent = EV_NONE;
		}
		else
		{
			menu_system.call_function(FUNC_INCRT_PDATA);
			uEvent = EV_NONE;
		}
		break;
	case (EV_BOOTDN):

		break;
	case (EV_STBY_TIMEOUT):

		break;

	// UNUSED AT THE MOMENT//
	case (EV_BTN_2CLICKED):
		break;
	case (EV_BTN_HELD):
		break;
	case (EV_BTN_RELEASED):
		break;
		// UNUSED AT THE MOMENT//

	default:
		break;
	}
}
/* -------------------------------------FUNCTIONS----------------------------------------*/
void blank_function()
{
	// Do nothing
}

void toggle_function()
{
	uToggle = uToggle == TOGGLE_OFF ? TOGGLE_ON : TOGGLE_OFF;
}

// ------ FUNC_ENTER_MENU ------
void goto_main_menu()
{
	menu_system.change_menu(main_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
}

void goto_machine_status_menu()
{
	menu_system.change_menu(machine_status_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
}

void activate_proc()
{
	currentProcessState.setState(ACTIVE_PROCESS);
	currentActiveProcessSubstate.setSubstate(PREHEATING);
	menu_system.change_menu(machine_status_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
}

void deactivate_proc()
{
	currentProcessState.setState(INACTIVE_PROCESS);
	currentActiveProcessSubstate.setSubstate(COOLING_DOWN);
	menu_system.change_menu(machine_status_menu);
	menu_system.update();
	menu_system.set_focusedLine(0);
}

// ------ FUNC_INCRT_PDATA and FUNC_DECRT_PDATA ------

// Increment set temperature
void incrt_pdata_setTemp()
{
	if (pData.setTemp + 5 <= MAX_TEMP)
		pData.setTemp += 5;
	menu_system.update();
}

// Decrement set temperature
void decrt_pdata_setTemp()
{
	if (pData.setTemp - 5 >= MIN_TEMP)
		pData.setTemp -= 5;
	menu_system.update();
}

// Increment control period
void incrt_pdata_controlPeriod()
{
	if (pData.controlPeriod + 60000 <= MAX_CONTROL_PERIOD)
		pData.controlPeriod += 60000;
	menu_system.update();
}

// Decrement control period
void decrt_pdata_controlPeriod()
{
	if (pData.controlPeriod - 60000 >= MIN_CONTROL_PERIOD)
		pData.controlPeriod -= 60000;
	menu_system.update();
}

// Increment heating duration
void incrt_pdata_heatingDuration()
{
	if (pData.heatingDuration + 60000 <= MAX_HEATING_DURATION)
		pData.heatingDuration += 60000;
	menu_system.update();
}

// Decrement heating duration
void decrt_pdata_heatingDuration()
{
	if (pData.heatingDuration - 60000 >= MIN_HEATING_DURATION)
		pData.heatingDuration -= 60000;
	menu_system.update();
}

/*-------------------------------Check Sleep for Standby----------------------------------*/
/**
 *  \brief    Issues a standby timeout event if the standby time has expired without any activity.
 *  \remarks  The standby timeout period is defined in ms in the file header.
 */
void checkSleep()
{
#ifdef _LCDGUI_

	// make sure it doesnt go to sleep if the process is active
	if (currentProcessState.getState() == ACTIVE_PROCESS)
	{
		lastActiveTime = millis();
	}
	// The last active time is updated every time some activity occurs. If the standby timeout
	// period has expired without any activity then a timeout event is issued.
	if (lastActiveTime + STANDBY_TIME_OUT < millis())
		uEvent = EV_STBY_TIMEOUT;

#endif /* _LCDGUI_ */
}

/***************************************************************************************************
 * Click Button Encoder Event Processor                                                            *
 ***************************************************************************************************/
/**
 *  \brief    Processes the button and encoder events.
 *  \remarks  This function processes the button and encoder events and sets the user event
 *            variable accordingly.
 */

void encoderEvent()
{
#ifdef _LCDGUI_

	static unsigned long lastEncTime = 0;

	// Check rotary action first since button takes precendence and we want to avoid
	// overwriting the button event and missing a button event.
	encNewPos += encoder->getValue();

	if (encNewPos != encLastPos)
	{
		lastActiveTime = lastEncTime = millis();

		if (encNewPos < encLastPos)
		{
			uEvent = EV_ENCDN;
		}
		else if (encNewPos > encLastPos)
		{
			uEvent = EV_ENCUP;
		}

		encLastPos = encNewPos;
	}

	ClickEncoder::Button b = encoder->getButton();
	if (b != ClickEncoder::Open)
	{
		switch (b)
		{
		case ClickEncoder::Clicked:
			lastActiveTime = lastEncTime = millis();
			uEvent = EV_BTN_CLICKED;
			break;
		case ClickEncoder::DoubleClicked:
			lastActiveTime = lastEncTime = millis();
			uEvent = EV_BTN_2CLICKED;
			break;
		case ClickEncoder::Held:
			lastActiveTime = lastEncTime = millis();
			uEvent = EV_BTN_HELD;
			break;
		case ClickEncoder::Released:
			lastActiveTime = lastEncTime = millis();
			uEvent = EV_BTN_RELEASED;
			break;
		default:
			uEvent = EV_NONE;
			break;
		}
	}

#endif /* _LCDGUI_ */
}

/***************************************************************************************************
 * Utility EEPROM Functions                                                                         *
 ***************************************************************************************************/
/**
 *  \brief                    Reset the EEPROM and program data to factory default settings.
 *  \remarks                  EEPROM data is only written if the new data is different to the
 *                            existing data to limit EEPROM wearout.
 *  \param [in] boolean full  True to reset the weld count, battery offset, and screen inversion.
 */
void resetEeprom(boolean full)
{

	// Write the factory default data to the eeprom. In the case of
	// \todo
	// [vals], these are zeroed, otherwise they are left unchanged.

	pData.tempRunAwayDelta = DEF_TEMP_RUNA_DELTA;
	pData.tempRunAwayCycles = DEF_TEMP_RUNA_CYCLES;
	pData.setTemp = DEF_SET_TEMP;
	pData.controlPeriod = DEF_CONTROL_PERIOD;
	pData.processInterval = DEF_PROCESS_INTERVAL;
	pData.heatingDuration = DEF_HEATING_DURATION;
	pData.preToHeatTempOffset = DEF_PRE_TO_HEAT_TEMP_OFFSET;
	pData.preToHeatHoldTime = DEF_PRE_TO_HEAT_HOLD_TIME;
	pData.serialPrintInterval = DEF_SERIAL_PRINT_INTERVAL;
	pData.kp = DEF_KP;
	pData.ki = DEF_KI;
	pData.kd = DEF_KD;
	pData.cp = DEF_CP;
	pData.ci = DEF_CI;
	pData.cd = DEF_CD;
	pData.gapThreshold = DEF_GAP_THRESHOLD;

	// The put function does not write new data if the existing data is the same thereby
	// limiting eeprom wearout.
	EEPROM.put(EEA_PDATA, pData);

	// The unique id is a simple method to ensure that a valid data set exists in the eeprom
	// (there are much better methods but we don't have the code space to spare).
	EEPROM.put(EEA_ID, EE_UNIQUEID);

#ifdef _SERIALCMD_ || _DEVELOPMENT_ || defined _BOOTSYS_

	if (full)
		Serial.print(F("EEPROM Full Reset"));
	else
		Serial.println(F("EEPROM Reset"));
#endif /* _SERIALCMD_ || _DEVELOPMENT_ || _BOOTSYS_*/
}

void loadEeprom()
{
	// Check the eeprom integrity by reading a magic number. If it is corrupt then the eeprom
	// is given a full factory reset, otherwise program data is loaded from the eeprom.
	uint32_t uniqueID;

	EEPROM.get(EEA_ID, uniqueID);

	// If the unique id is not present (first upload or change of program) or is
	// incorrect (corrupted, falsely overwritten) then the eeprom is reset to factory
	if (uniqueID != EE_UNIQUEID)
		resetEeprom(EE_FULL_RESET);
	else
		EEPROM.get(EEA_PDATA, pData);
}
/**
 *  \brief    Udates the EEPROM data with local program data structure.
 *  \remarks  EEPROM data is only written if the new data is different to the
 *            existing data to limit EEPROM wearout.
 */
void updateEeprom()
{
	static unsigned long lastEEUpdatetime = 0;

	// Do not do this too often to prevent premature eeprom wearout.
	if (millis() - lastEEUpdatetime > EEPROM_UPDATE_T)
	{
		lastEEUpdatetime = millis();

		// Write the current program data to the eeprom.
		EEPROM.put(EEA_PDATA, pData);

#ifdef _DEVELOPMENT_
		Serial.println(F("Updated EEPROM"));
#endif /* _DEVELOPMENT_ */
	}
}
