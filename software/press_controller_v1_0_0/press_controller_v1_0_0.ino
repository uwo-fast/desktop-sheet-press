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

// Libraries for PID and Thermocouples
#include <PID_v1.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31855.h>
#include <EEPROM.h>

#include <press_controller_v1_0_0.h>

// Machine states
enum states
{
	ST_STANDBY,			/**< Machine state: standby */
	ST_MAIN_SCREEN,		/**< Machine state: display main screen */
	ST_MAIN_SCREEN_CNT, /**< Machine state: display statistics */
	ST_MENU_SCREEN,		/**< Machine state: display menu screen */
	ST_SUB_MENU_1,		/**< Machine state: display sub-menu1 */
	ST_SUB_MENU_2,		/**< Machine state: display sub-menu2 */
	ST_BATTERY_LOW,		/**< Machine state: low battery voltage */
	ST_BATTERY_HIGH,	/**< Machine state: high battery voltage */
	ST_TEMP_HIGH,		/**< Machine state: high temperature */

	ST_SYSTEM_HOME,	  /**< Machine state: display system screen */
	ST_SYSTEM_MENU,	  /**< Machine state: display system menu */
	ST_SETTINGS_MENU, /**< Machine state: display settings menu */
	ST_REBOOT_MENU,	  /**< Machine state: display reboot menu */
};

enum event
{
	// Private machine events
	EV_NONE,  /**< Machine event: no pending event */
	EV_BTNDN, /**< Machine event: button pressed */
	EV_BTNUP, /**< Machine event: button released */
	EV_ENCUP, /**< Machine event: encoder rotate right */
	EV_ENCDN, /**< Machine event: encoder rotate left */

	// Public machine events
	EV_BOOTDN,		 /**< Machine event: button pressed on boot */
	EV_STBY_TIMEOUT, /**< Machine event: standby timer has timed out */
	EV_EEUPD,		 /**< Machine event: EEPROM needs updating */
};

/***************************************************************************************************
 * Global program variables and objects                                                             *
 ***************************************************************************************************/

// Structures and objects
progData pData;								   /**< Program operating data */
Adafruit_MAX31855 thermocouple1(CLK, CS1, DO); /**< Thermocouple 1 object */
Adafruit_MAX31855 thermocouple2(CLK, CS2, DO); /**< Thermocouple 2 object */

// PID object setup for both thermocouples
// PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, DIRECT); /**< PID1 object gets input from thermocouple 1 and ouputs to relay 1*/
PID myPID2(&Input2, &Output2, &Setpoint, consKp, consKi, consKd, DIRECT); /**< PID2 object gets input from thermocouple 2 and ouputs to relay 2*/

// Adafruit_SSD1306 display( 128, 64, &Wire, OLED_RESET, 800000L ); /**< OLED display object */ REPLACE WITH I2C LCD

// Static variables
uint8_t mState = ST_MAIN_SCREEN;		 /**< Current machine state */
uint8_t TCelsius;						 /**< System temperature in celsius */
uint8_t batteryVoltage = DEF_NOM_BATT_V; /**< Current battery voltage x10 */
int8_t batt_gauge;						 /**< Battery gauge segments to display */
boolean sysMenu = false;				 /**< In the system menu structure */

// Volatile variables - will be changed by the ISR
volatile unsigned long lastActiveTime;
volatile uint8_t mEvent; /**< Current pending machine event */

void reset_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void reset_mcusr(void)
{
	MCUSR = 0;
	wdt_disable();
}

//--------------------------------------------------------------------------------------------

// Initialization and Setup
// Define Chip Select pins and I2C pins for MAX31855s

// Initializing thermocouples 1 and 2

// Initializing the setup error flags
bool tcInitFlag1 = false;
bool tcInitFlag2 = false;

// Initializing error flags
uint8_t errorFlagsMAX[] = {0, 0, 0, 0, 0, 0};

// Define period and duty cycle for SSR control
const unsigned long period = 1 * 1000;
unsigned long cycleStart1 = 0;
unsigned long cycleStart2 = 0;

// Initializing temperatures and set points for PID
double setTemp = 0.0;
double temp1 = 0.0;
double temp2 = 0.0;
double lastValidTemp1 = 0.0; // * Varaible to preserve the last valid temp 1 reading
double lastValidTemp2 = 0.0; // * Varaible to preserve the last valid temp 2 reading
bool validTemp1 = false;	 // * Variable to indicate if the temp 1 reading is valid
bool validTemp2 = false;	 // * Variable to indicate if the temp 2 reading is valid

// Process varaibles
unsigned long processDuration = 5.0 * 1000 * 60; // format is for illustrative effect, e.g. 1.0*1000*60*60 = 1 minute
unsigned long processTime = 0;
unsigned long currentMillis = 0;
unsigned long lastMillis = 0;	 // Variable to hold the last update time
const long processInterval = 10; // Interval time for the process loop
unsigned long lastActiveTime;

// Process Parameters
double Setpoint, Input1, Output1, Input2, Output2;
double Cp = 1, Ci = 1, Cd = 1;										  // Constants to divide aggressive params to get the conservative params
double aggKp = 1, aggKi = 0.1, aggKd = 0.02;						  // Aggressive PID Tuning Params
double consKp = aggKp / Cp, consKi = aggKi / Ci, consKd = aggKd / Cd; // Conservative PID Tuning Params

double gapThres = 5; // Gap threshold for the dynamic tunings

// Serial print interval settings
unsigned long serialPrintStart = 0;
const unsigned long serialPrintInterval = 1000;

// ------------------------------------------------------
// SETUP -------------------------------------------------
// ------------------------------------------------------

// Setup helpers

void thermocoupleSetup(Adafruit_MAX31855 &thermocouple, int tcNum)
{
	Serial.println("Initializing TC sensor...");
	// Check if thermocouple initialization fails
	if (!thermocouple.begin())
	{
		if (tcNum == 1)
		{
			tcInitFlag1 = true;
		}
		else if (tcNum == 2)
		{
			tcInitFlag2 = true;
		}
		Serial.println("Thermocouple " + String(tcNum) + " ERROR.");
	}
	else
	{
		if (tcNum == 1)
		{
			tcInitFlag1 = false;
		}
		else if (tcNum == 2)
		{
			tcInitFlag2 = false;
		}
		Serial.println("Thermocouple " + String(tcNum) + " OK.");
	}
}

void eepromSetup()
{
	Serial.println("Initializing EEPROM...");
	byte tmpArraEe[10], tmp2ArrEe[10];
	eeprom_read_bytes(0, tmpArraEe, 8);

	if (memcmp(tmpArraEe, &gAllData, 8) != 0)
	{
		if (tmpArraEe[6] == 0xFF)
		{
			memcpy(tmp2ArrEe, &gAllData, 8);
			eeprom_write_bytes(0, tmp2ArrEe, 8);
		}
		else
		{
			memcpy(&gAllData, tmpArraEe, 8);
		}
	}
	eepromReset();

	Delay = gAllData.delay;
	BatteryAlarm = gAllData.failureAlarms;
	WeldCount = gAllData.processCount;

	Serial.println(gAllData.delay);
	Serial.println(gAllData.failureAlarms);
	Serial.println(gAllData.processCount);
}

void updateEeprom()
{
	byte tmpArraEe[10], tmp2ArrEe[10];
	if (gAllData.delay != Delay ||
		gAllData.failureAlarms != FailureAlarms ||
		gAllData.processCount != ProcessCount ||)
	{

		gAllData.delay = Delay;
		gAllData.failureAlarms = FailureAlarms;
		gAllData.processCount = ProcessCount;

		memcpy(tmp2ArrEe, &gAllData, 8);
		eeprom_write_bytes(0, tmp2ArrEe, 8);
		Serial.println("Updated eeprom");
	}
}

// Setup Main
void setup()
{
	// The interrupt is used to sense the encoder rotation. It could just as well be polled
	// without loss of responsiveness. This was actually tried and no noticeable performance
	// degradation was observed. Interrupts are usefull for high speed encoders such as
	// used on servo systems. Manually adjusted encoders are very slow.
	attachInterrupt(ENC_INT, isr, FALLING);

	lastActiveTime = millis();

	// Initialize serial communication at 9600 baud rate
	Serial.begin(9600); // max is prob 115200
	delay(500);

	// Initialize thermocouples
	pinMode(CS1, OUTPUT);
	pinMode(CS2, OUTPUT);
	thermocoupleSetup(thermocouple1, 1);
	thermocoupleSetup(thermocouple2, 2);

	// Set SSR pins as output
	pinMode(PIN_SSR1, OUTPUT);
	pinMode(PIN_SSR2, OUTPUT);

	// Applying the default PID tunings / settings
	// Set initial setpoint temperature
	Setpoint = setTemp;

	// Set PID controllers to automatic mode
	myPID1.SetMode(AUTOMATIC);
	myPID2.SetMode(AUTOMATIC);
	// Set PID sample time
	myPID1.SetSampleTime(period);
	myPID2.SetSampleTime(period);
	// Function to initialize thermocouples
	myPID1.SetOutputLimits(10 * 2.55, 255);
	myPID2.SetOutputLimits(10 * 2.55, 255);

	eepromSetup();

	int suDelay = 100;
	delay(suDelay);
}

// ------------------------------------------------------
// LOOP -------------------------------------------------
//

enum ProcessState
{
	INACTIVE_PROCESS, // Indicates an inactive process
	ACTIVE_PROCESS,	  // Indicates an active process
	ERROR_PROCESS,	  // Indicates an error in the process
	STANDBY_PROCESS	  // Indicates the system is in standby, operationally analogous to INACTIVE_PROCESS
};

ProcessState currentProcessState = INACTIVE_PROCESS;

void loop()
{
	SystemChecks();
	UserStateMachine();
	handleSerialCommands();
	printSerialData();
	updateEeprom();
	logSD();

	currentMillis = millis();

	if (currentMillis - lastMillis >= processInterval)
	{
		switch (currentProcessState)
		{
		case ACTIVE_PROCESS:
			// Only run PID and slowPWM if activeProcess is true

			Input1 = temp1;
			Input2 = temp2;
			Setpoint = setTemp;

			// dynamicTuning();
			myPID1.Compute();
			myPID2.Compute();
			slowPWM(PIN_SSR1, cycleStart1, period, Output1);
			slowPWM(PIN_SSR2, cycleStart2, period, Output2);
			// Further operations for the active process
			processTimeManagement();
			break;

		case INACTIVE_PROCESS:
			// If the process is not active, just read temperatures and write SSRs LOW
			digitalWrite(PIN_SSR1, LOW);
			digitalWrite(PIN_SSR2, LOW);
			break;

		case ERROR_PROCESS:
			// Handle error condition, perhaps by setting SSRs to LOW and signaling error
			digitalWrite(PIN_SSR1, LOW);
			digitalWrite(PIN_SSR2, LOW);
			// Signal error condition, e.g., by blinking an LED or sending an error message
			signalError();
			break;

		default:
			// Default action, perhaps similar to INACTIVE_PROCESS or specific safe state
			digitalWrite(PIN_SSR1, LOW);
			digitalWrite(PIN_SSR2, LOW);
			break;
		}

		lastMillis = currentMillis;
	}
}

void signalError()
{
	// Implement continuous error signaling in LCD corner and serial output
	// on first pass log dumb to SD card and serial error message. record in eeprom
}

void SystemChecks()
{
	thermalRunawayCheck();
	readCheckTemp();
	checkSleep();
}


thermalRunawayCheck()
{

}


/*-------------------------------Check Sleep for Standby----------------------------------*/
/**
 *  \brief    Issues a standby timeout event if the standby time has expired without any activity.
 *  \remarks  The standby timeout period is defined in ms in the file header.
 */
void checkSleep()
{
	// The last active time is updated every time some activity occurs. If the standby timeout
	// period has expired without any activity then a timeout event is issued.
	if (lastActiveTime + STANDBY_TIME_OUT < millis())
		if (mState != ST_BATTERY_LOW)
			mEvent = EV_STBY_TIMEOUT;
}

/* -------------------------------Read and Check Temperature----------------------------------*/
/**
 *  \brief    Reads the temperature from the thermocouples and checks for errors.
 *  \remarks  
 */
void readCheckTemp()
{
	temp1 = thermocouple1.readCelsius();
	temp2 = thermocouple2.readCelsius();
	checkIsnan(temp1, temp2);

	// error ratio checking
}

// Function to check if temperature readings are NaN and perform error handling
void checkIsnan(double &temp1, double &temp2)
{
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


void processTimeManagement()
{

	// TODO DURATION CONTROL
	// Check if processTime has reached processDuration

	// Check if temperature readings meet condition
	if (temp1 >= setTemp && temp2 >= setTemp)
	{
		tempConditionMet = min(tempConditionMet + 1, 5);
	}
	else
	{
		tempConditionMet = 0; // Reset counter if condition not met
	}

	if (lastSetTemp != setTemp)
	{
		tempConditionMet = 0;
	}

	// Update the process time
	if (tempConditionMet == 5)
	{
		processTime += processInterval;
	}

	// Check if processTime has reached processDuration
	if (processTime >= processDuration)
	{
		processDone = true;
	}
	lastSetTemp = setTemp;
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

// Optional function to dynamically adjust PID tuning parameters based on gap to setpoint
void dynamicTuning()
{
	double gap1 = abs(Setpoint - Input1);
	if (gap1 < gapThres)
	{ // Less aggressive tuning parameters for small gap
		myPID1.SetTunings(consKp, consKi, consKd);
	}
	else
	{ // More aggressive tuning parameters for large gap
		myPID1.SetTunings(aggKp, aggKi, aggKd);
	}

	double gap2 = abs(Setpoint - Input2);
	if (gap2 < gapThres)
	{ // Less aggressive tuning parameters for small gap
		myPID2.SetTunings(consKp, consKi, consKd);
	}
	else
	{ // More aggressive tuning parameters for large gap
		myPID2.SetTunings(aggKp, aggKi, aggKd);
	}
}

// Functon to allow serial navigation and control
void handleSerialCommands()
{
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
				temp1 = received.substring(3).toDouble();
			}
			else if (received.startsWith("t2="))
			{
				temp2 = received.substring(3).toDouble();
			}
			else if (received.startsWith("st="))
			{
				setTemp = received.substring(3).toDouble();
			}
			else if (received.startsWith("o1="))
			{
				Output1 = received.substring(3).toDouble();
			}
			else if (received.startsWith("o2="))
			{
				Output2 = received.substring(3).toDouble();
			}
			else if (received.startsWith("dt="))
			{
				processDuration = (long)(received.substring(3).toDouble() * 1000 * 60); // converting minutes to milliseconds
			}
			else if (received.startsWith("t="))
			{
				processTime = (long)(received.substring(2).toDouble() * 1000 * 60); // converting minutes to milliseconds
			}
			else if (received.startsWith("kp="))
			{
				aggKp = received.substring(3).toDouble();
			}
			else if (received.startsWith("ki="))
			{
				aggKi = received.substring(3).toDouble();
			}
			else if (received.startsWith("kd="))
			{
				aggKd = received.substring(3).toDouble();
			}

			received = ""; // clear received data
		}
	}
}

void printSerialData()
{
	if (millis() - serialPrintStart > serialPrintInterval)
	{

		Serial.print("Error flags: ");

		int errorFlagsMAXSize = sizeof(errorFlagsMAX) / sizeof(errorFlagsMAX[0]);
		// Print error flags
		for (int i = 0; i < errorFlagsMAXSize; i++)
		{
			Serial.print(errorFlagsMAX[i]);
		}

		Serial.print(", t1: ");
		Serial.print(temp1);
		Serial.print(", t2: ");
		Serial.print(temp2);
		Serial.print(", st: ");
		Serial.print(setTemp);
		Serial.print(", o1: ");
		Serial.print(Output1);
		Serial.print(", o2: ");
		Serial.print(Output2);
		Serial.print(", t: ");
		Serial.print((float)processTime / 1000 / 60);
		Serial.print(", dt: ");
		Serial.print((float)processDuration / 1000 / 60);
		Serial.print(", nrg: ");
		Serial.print(accumulatedEnergy / 1000);
		Serial.print(", aggKp: ");
		Serial.print(aggKp);
		Serial.print(", aggKi: ");
		Serial.print(aggKi);
		Serial.print(", aggKd: ");
		Serial.println(aggKd); // Use println to add newline at the end

		serialPrintStart = millis();
	}
}



/***************************************************************************************************
 * State Machine                                                                                    *
 ***************************************************************************************************/
/**
 *  \brief  Implementation of state machine.
 */

void stateMachine()
{
	char str[5];
	static uint8_t selectedMenu = 0;
	static uint8_t selectedMainMenu = 0;
	static uint8_t selectedSubMenu = 0;
	static uint8_t measuredVoltage;

	// Scan for events - the event queue length is one.

	// Process any public boot events. These events are the highest priority
	// and must be processed before any of the private events.
	if (mEvent == EV_BOOTDN)
	{
		mState = ST_EEPROM_SCREEN;
		mEvent = EV_NONE;
	}
	else
	{
		// Search for and process any private events.
		checkForBtnEvent();

		switch (mEvent)
		{
		case EV_STBY_TIMEOUT:
			mState = ST_STANDBY;
			mEvent = EV_NONE;
			break;
		default:
			break;
		}

		// Machine states
		switch (mState)
		{

		case ST_PULSE_VOLTAGE:

			break;

		default:
			break;
		}
	}
}

/***************************************************************************************************
 * LCD DISPLAYS SCREENS		                                                                       *
 ***************************************************************************************************/
/**
 *  \brief    Display the SPLASH screen.
 *  \remarks  This is shown once only at start-up and is for general information and advertising.
 */
void splash()
{
	display.clearDisplay();
	display.display();

	display.print(F(_DEVICENAME_)); // 16 chars per line

	display.print(F("Ver " xstr(_VERSION_MAJOR_) "." xstr(_VERSION_MINOR_) "." xstr(_REVISION_) " " __DATE__));

	display.print(F("by " _AUTHOR_));

	display.print(F(_COMPANY_));

	display.print(F(_RIGHTS_));

	display.display();

	display.clearDisplay();
	display.display(); // set to home screen
}

/***************************************************************************************************
 * Interrupt service routine                                                                        *
 ***************************************************************************************************/
/**
 *  \brief    Interrupt service routine.
 *  \remarks  Responds to interrupts from the rotary encoder. Manages and tracks rotary encoder
 *            position changes. Issues encoder position transition events according to whether
 *            the encoder is rotated clockwise (up) or anticlockwise (down).
 */
void isr()
{
	static volatile unsigned long lastInterruptTime = 0;

	// Ignore changes that occur within the debounce period.
	if ((millis() - lastInterruptTime) > RS_DEBOUNCE)
	{

		// There are two pins to the encoder. One pin issues the interrupt. The other pin
		// therefore indicates the phase, which is effectively the direction of rotation.
		// Read the phase pin and issue an encoder direction event based on its state.
		mEvent = digitalRead(PIN_DT) ? EV_ENCDN : EV_ENCUP;
		lastActiveTime = lastInterruptTime = millis();
	}
}

/***************************************************************************************************
 * Utility Conversion Functions                                                                     *
 ***************************************************************************************************/
/**
 *  \brief                    Returns a character string representing a formatted numeric value.
 *  \param [in] char *str     Pointer to the string to receive the formatted result.
 *  \param [in] uint16_t val  The integer value to be formatted.
 *  \param [in] vf_Type fType The type of variable to be formatted.
 *  \return     char*         Pointer to the formatted string.
 */
char *valStr(char *str, uint16_t val, vf_Type fType)
{

	// We must resort to this awkward kludge to format the variables because variable width and
	// precision (* specifier) are not implemented in avr gcc - bugger!!!

	switch (fType)
	{
	case VF_BATTALM:
	case VF_BATTV:
		sprintf_P(str, PSTR("%2.1u.%01u"), val / 10, val % 10);
		break;
	case VF_BATTA:
		sprintf_P(str, PSTR("%2.1u.%01u"), val / 10, val % 10);
		break;
	case VF_WELDCNT:
		sprintf_P(str, PSTR("%5.1u"), val);
		break;
	case VF_TEMP:
		sprintf_P(str, PSTR("%5.1u"), val);
		break;
	case VF_PLSDLY:
		sprintf_P(str, PSTR("%4.1u"), val);
		break;
	case VF_SHTPLS:
		sprintf_P(str, PSTR("%3.1u"), val);
		break;
	case VF_DELAY:
		sprintf_P(str, PSTR("%1.1u.%01u"), val / 10, val % 10);
		break;
	}

	return str;
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

	// Write the factory default data to the eeprom. In the case of a full reset, the weld count,
	// battery offset, and screen orientation are zeroed, otherwise they are left unchanged.
	pData.autoPulseDelay = DEF_AUTO_PLSDELAY;
	pData.PulseBatteryVoltage = DEF_PULSE_VOLTAGE;
	pData.PulseAmps = DEF_PULSE_AMPS;
	pData.batteryAlarm = DEF_BATT_ALARM;
	pData.batteryhighAlarm = DEF_HIGH_BATT_ALARM;
	pData.weldCount = full == EE_FULL_RESET ? 0 : pData.weldCount;
	pData.pulseTime = DEF_PULSE_TIME;
	pData.maxPulseTime = DEF_MAX_PULSE_TIME;
	pData.shortPulseTime = DEF_SPULSE_TIME;
	pData.batteryOffset = full == DEF_BATTERY_OFFSET ? 0 : pData.batteryOffset;
	pData.pFlags.en_autoPulse = DEF_AUTO_PULSE;
	pData.pFlags.en_oledInvert = full ? DEF_OLED_INVERT : pData.pFlags.en_oledInvert;

	// The put function does not write new data if the existing data is the same thereby
	// limiting eeprom wearout.
	EEPROM.put(EEA_PDATA, pData);

	// The unique id is a simple method to ensure that a valid data set exists in the eeprom
	// (there are much better methods but we don't have the code space to spare).
	EEPROM.put(EEA_ID, EE_UNIQUEID);

#if defined _DEVELOPMENT_ || defined _BOOTSYS_

	if (full)
		Serial.print(F("EEPROM Full Reset"));
	else
		Serial.println(F("EEPROM Reset"));

#endif /* _DEVELOPMENT_ || _BOOTSYS_*/
}

void loadEeprom()
{
	// Check the eeprom integrity by reading a magic number. If it is corrupt then the eeprom
	// is given a full factory reset, otherwise program data is loaded from the eeprom.
	uint32_t uniqueID;

	EEPROM.get(EEA_ID, uniqueID);

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
