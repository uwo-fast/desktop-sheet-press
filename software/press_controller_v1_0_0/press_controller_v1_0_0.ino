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

// The I2C LCD library
#include <LiquidCrystal_I2C.h>
// The menu wrapper library
#include <LiquidMenu.h>


#include <press_controller_v1_0_0.h>

// Machine states
enum states
{
	ST_STANDBY,		/**< Machine state: standby */
	ST_HOME_SCREEN, /**< Machine state: home screen */

	ST_SYSTEM_SCREEN, /**< Machine state: display system screen, currently only manual eeprom reset */
};

// Machine events
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
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// PID object setup for both thermocouples
// PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, DIRECT); /**< PID1 object gets input from thermocouple 1 and ouputs to relay 1*/
PID myPID2(&Input2, &Output2, &Setpoint, consKp, consKi, consKd, DIRECT); /**< PID2 object gets input from thermocouple 2 and ouputs to relay 2*/

// Adafruit_SSD1306 display( 128, 64, &Wire, OLED_RESET, 800000L ); /**< OLED display object */ REPLACE WITH I2C LCD

// Static variables
uint8_t mState = ST_MAIN_SCREEN;				/**< Current machine state */
bool tempRunAwayAlarm1 = DEF_TEMP_RUNA_ALARM_1; /**< Temperature runaway alarm for thermocouple 1 */
bool tempRunAwayAlarm2 = DEF_TEMP_RUNA_ALARM_2; /**< Temperature runaway alarm for thermocouple 2 */
bool tc1Status = TC_FAULT;						/**< Thermocouple 1 status */
bool tc2Status = TC_FAULT;						/**< Thermocouple 2 status */
uint8_t errorFlagsMAX[] = {0, 0, 0, 0, 0, 0};	/**< Error flags for MAX31855 thermocouple sensor */
int8_t temp1;									/**< Current temperature reading from thermocouple 1 */
int8_t temp2;									/**< Current temperature reading from thermocouple 2 */
int8_t lastValidTemp1;							/**< Last valid temperature reading from thermocouple 1 */
int8_t lastValidTemp2;							/**< Last valid temperature reading from thermocouple 2 */
bool validTemp1 = false;						/**< Flag indicating if the temperature reading from thermocouple 1 is valid */
bool validTemp2 = false;						/**< Flag indicating if the temperature reading from thermocouple 2 is valid */

boolean sysMenu = false; //***********//	 /**< In the system menu structure */

// Volatile variables - will be changed by the ISR
volatile unsigned long lastActiveTime;
volatile uint8_t mEvent; /**< Current pending machine event */

// Process Varaibles
unsigned long processTimeComplete = 0;
unsigned long currentMillis = 0;
unsigned long lastMillis = 0; // Variable to hold the last update time

unsigned long cycleStart1 = 0;
unsigned long cycleStart2 = 0;

unsigned long lastSerialPrint = 0;

// Process Parameters
uint8_t Setpoint, Input1, Output1, Input2, Output2;

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
#if defined _DEVELOPMENT_ || defined _BOOTSYS_
	Serial.begin(_SERIAL_BAUD_);
#endif /* _DEVELOPMENT_ || _BOOTSYS_*/

	// The interrupt is used to sense the encoder rotation. It could just as well be polled
	// without loss of responsiveness. This was actually tried and no noticeable performance
	// degradation was observed. Interrupts are usefull for high speed encoders such as
	// used on servo systems. Manually adjusted encoders are very slow.
	// This needs to happen before the delay to allow the interrupt to be set up before the
	// user needs to use it to possibly enter the system menu.
	attachInterrupt(ENC_INT, isr, FALLING);

	// Delay to allow entering eeprom reset on GUI or to click and open serial monitor on PC
	delay(500);

	// Initialize thermocouples
	pinMode(CS1, OUTPUT);
	pinMode(CS2, OUTPUT);

	// Set SSR pins as output
	pinMode(PIN_SSR1, OUTPUT);
	pinMode(PIN_SSR2, OUTPUT);

	tc1Status = thermocoupleSetup(thermocouple1);
	tc2Status = thermocoupleSetup(thermocouple2);

#if defined _DEVELOPMENT_ || defined _BOOTSYS_
	Serial.println("Initializing TC sensor...");

	if (tc1Status == TC_FAULT)
	{
		Serial.println("Thermocouple 1 ERROR, could not initialize.");
	}
	else
	{
		Serial.println("Thermocouple 1 initialized.");
	}

	if (tc2Status == TC_FAULT)
	{
		Serial.println("Thermocouple 2 ERROR, could not initialize.");
	}
	else
	{
		Serial.println("Thermocouple 2 initialized.");
	}
#endif /* _DEVELOPMENT_ || _BOOTSYS_*/

	// Applying the default PID tunings / settings
	// Set initial setpoint temperature
	Setpoint = setTemp;

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
	myPID1.SetSampleTime(pData.period);
	myPID2.SetSampleTime(pData.period);
	myPID1.SetOutputLimits(0, 255);
	myPID2.SetOutputLimits(0, 255);

	/**
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
	void loadEeprom();

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
	userStateMachine();
	systemChecks();
	handleSerialCommands();
	printSerialData();
	updateEeprom();
	logSD(); //TODO

	currentMillis = millis();

	if (currentMillis - lastMillis >= pData.processInterval)
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
			slowPWM(PIN_SSR1, cycleStart1, pData.period, Output1);
			slowPWM(PIN_SSR2, cycleStart2, pData.period, Output2);
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
			signalError(); // TODO
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

/* -------------------------------System Checks----------------------------------*/
/**
 *  \brief    System checks.
 *  \remarks  Checks for system errors and issues a standby timeout event if the standby time has
 *            expired without any activity.
 */
void systemChecks()
{
	thermalRunawayCheck(); // TODO
	readCheckTemp();
	checkSleep();
}

/* -------------------------------Thermal Runaway Check----------------------------------*/
/**
 *  \brief    Checks for thermal runaway.
 *  \remarks
 */
void thermalRunawayCheck() // TODO
{

}

/* -------------------------------Log Data to SD----------------------------------*/
/**
 *  \brief    Logs data to the SD card.
 *  \remarks  Logs data to the SD card.
 */
void logSD() // TODO
{
	// Log data to SD card
}

/* -------------------------------Read and Check Button Event----------------------------------*/
/**
 *  \brief    Reads the rotary encoder push button switch transition event.
 *  \remarks  Issues a switch transition event according to whether the switch is has been pressed
 *            and the debounce interval has expired or whether the switch has been released.
 */
void checkForBtnEvent()
{
	static unsigned long lastBtnTime = 0;
	static boolean lastBtnState = B_UP;
	boolean thisBtnState;

	thisBtnState = btnState();

	// Ignore changes that occur within the debounce period.
	if (millis() - lastBtnTime > RS_DEBOUNCE)
	{

		// Only respond to a change of the biutton state.
		if (thisBtnState != lastBtnState)
		{
			// Issue an event based on the current state of the button.
			mEvent = thisBtnState == B_UP ? EV_BTNUP : EV_BTNDN;

			lastActiveTime = lastBtnTime = millis();
			lastBtnState = thisBtnState;
		}
	}
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
	// Check if processTimeComplete has reached processDuration

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
		processTimeComplete += pData.processInterval;
	}

	// Check if processTimeComplete has reached processDuration
	if (processTimeComplete >= processDuration)
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
	if (gap1 < pData.gapThreshold)
	{ // Less aggressive tuning parameters for small gap
		myPID1.SetTunings(pData.kp / pData.cp, pData.ki / pData.ci, pData.kd / pData.cd);
	}
	else
	{ // More aggressive tuning parameters for large gap
		myPID1.SetTunings(pData.kp, pData.ki, pData.kd);
	}

	double gap2 = abs(Setpoint - Input2);
	if (gap2 < pData.gapThreshold)
	{ // Less aggressive tuning parameters for small gap
		myPID2.SetTunings(pData.kp / pData.cp, pData.ki / pData.ci, pData.kd / pData.cd);
	}
	else
	{ // More aggressive tuning parameters for large gap
		myPID2.SetTunings(pData.kp, pData.ki, pData.kd);
	}
}

// Function to allow serial navigation and control
void handleSerialCommands()
{
#if defined _DEVELOPMENT_ || defined _BOOTSYS_
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
				// Assuming dt is for processDuration in minutes, converted to milliseconds
				pData.processDuration = (uint16_t)(received.substring(3).toInt() * 1000 * 60);
			}
			else if (received.startsWith("kp="))
			{
				pData.kp = (uint8_t)received.substring(3).toInt();
			}
			else if (received.startsWith("ki="))
			{
				pData.ki = (uint8_t)received.substring(3).toInt();
			}
			else if (received.startsWith("kd="))
			{
				pData.kd = (uint8_t)received.substring(3).toInt();
			}
			received = ""; // clear received data
		}
	}
#endif /* _DEVELOPMENT_ || _BOOTSYS_ */
}

void printSerialData()
{
#if defined _DEVELOPMENT_ || defined _BOOTSYS_
	if (millis() - lastSerialPrint > pData.serialPrintInterval)
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
		Serial.print((float)processTimeComplete / 1000 / 60);
		Serial.print(", dt: ");
		Serial.print((float)pData.processDuration / 1000 / 60);
		Serial.print(", aggKp: ");
		Serial.print(pData.kp);
		Serial.print(", aggKi: ");
		Serial.print(pData.ki);
		Serial.print(", aggKd: ");
		Serial.println(pData.kd); // Use println to add newline at the end

		lastSerialPrint = millis();
	}
#endif /* _DEVELOPMENT_ || _BOOTSYS_*/
}

/***************************************************************************************************
 * State Machine                                                                                    *
 ***************************************************************************************************/
/**
 *  \brief  Implementation of state machine.
 */

void userStateMachine()
{
	static uint8_t currentMenuItemIndex = 0;
	static uint8_t mainMenuSelectionIndex = 0;
	static uint8_t subMenuSelectionIndex = 0;

	// Scan for events - the event queue length is one.
	// Process any public boot events. These events are the highest priority
	// and must be processed before any of the private events.
	if (mEvent == EV_BOOTDN)
	{
		mState = ST_SYSTEM_SCREEN;
		mEvent = EV_NONE;
	}
	else
	{
		// Search for and process any private events.
		// Check for button events
		checkForBtnEvent();

		// MACHINE EVENTS
		switch (mEvent)
		{
		case EV_STBY_TIMEOUT:
			mState = ST_STANDBY;
			mEvent = EV_NONE;
			break;
		default:
			break;
		}

		// MACHINE STATES
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

	display.print(F("Ver " toString(_VERSION_MAJOR_) "." toString(_VERSION_MINOR_) "." toString(_REVISION_) " " __DATE__));

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

	pData.tempRunAwayDelta = DEF_TEMP_RUNA_DELTA;
	pData.tempRunAwayCycles = DEF_TEMP_RUNA_CYCLES;
	pData.setTemp = DEF_SET_TEMP;
	pData.controlPeriod = DEF_CONTROL_PERIOD;
	pData.processInterval = DEF_PROCESS_INTERVAL;
	pData.processDuration = DEF_PROCESS_DURATION;
	pData.serialPrintInterval = DEF_SERIAL_PRINT_INTERVAL;
	pData.kp = DEF_KP;
	pData.ki = DEF_KI;
	pData.kd = DEF_KD;
	pData.cp = DEF_CP;
	pData.ci = DEF_CI;
	pData.cd = DEF_CD;
	pData.gapThreshold = DEF_GAP_THRESHOLD;

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
