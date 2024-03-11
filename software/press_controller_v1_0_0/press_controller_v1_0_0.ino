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
#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>

#include <EEPROM.h>

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

#include "press_controller_v1_0_0.h"

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
unsigned long processTimeComplete = 0;
unsigned long currentMillis = 0;
unsigned long lastMillis = 0; // Variable to hold the last update time

unsigned long cycleStart1 = 0;
unsigned long cycleStart2 = 0;

unsigned long lastThermalRunawayCheck = 0;  // Tracks the last thermal runaway check time

// PID Process Parameters
double Setpoint, Input1, Output1, Input2, Output2;

// Timing variable for the serial print interval, for serial command mode
#ifdef _SERIALCMD_
unsigned long lastSerialPrint = 0;
#endif /* _SERIALCMD_ */

// LCD State and Event variables
#ifdef _LCDGUI_
uint8_t mState = ST_MAIN_SCREEN; /**< Current machine state */
uint8_t mEvent;					 /**< Current pending machine event */
int16_t encLastPos, encNewPos;	 /**< Encoder position variables */
unsigned long lastActiveTime;
#endif /* _LCDGUI_ */

// Structures and objects
progData pData; /**< Program operating data */

Adafruit_MAX31855 thermocouple1(PIN_TC_CLK, PIN_TC_CS1, PIN_TC_DO); /**< Thermocouple 1 object */
Adafruit_MAX31855 thermocouple2(PIN_TC_CLK, PIN_TC_CS2, PIN_TC_DO); /**< Thermocouple 2 object */

#ifdef _LCDGUI_
ClickEncoder *encoder;				/**< Encoder object */
LiquidCrystal_I2C lcd(0x3F, 16, 2); /**< LCD display object */
#endif								/* _LCDGUI_ */

// PID object setup for both thermocouples
// PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint, DEF_KP / MILLI_UNIT, DEF_KI / MILLI_UNIT, DEF_KD / MILLI_UNIT, DIRECT); /**< PID1 object gets input from thermocouple 1 and ouputs to relay 1*/
PID myPID2(&Input2, &Output2, &Setpoint, DEF_KP / MILLI_UNIT, DEF_KI / MILLI_UNIT, DEF_KD / MILLI_UNIT, DIRECT); /**< PID2 object gets input from thermocouple 2 and ouputs to relay 2*/

enum ProcessState
{
	INACTIVE_PROCESS, // Indicates an inactive process
	ACTIVE_PROCESS,	  // Indicates an active process
	ERROR_PROCESS,	  // Indicates an error in the process
	STANDBY_PROCESS	  // Indicates the system is in standby
};

ProcessState currentProcessState = INACTIVE_PROCESS;

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

#ifdef _LCDGUI_
void timerIsr()
{
	encoder->service();
}
#endif /* _LCDGUI_ */

// Setup Main
void setup()
{
#ifdef _SERIALCMD_ || _DEVELOPMENT_ || defined _BOOTSYS_
	Serial.begin(_SERIAL_BAUD_);
#endif /* _SERIALCMD_ || _DEVELOPMENT_ || _BOOTSYS_*/

#ifdef _LCDGUI_
	// This needs to happen before the delay to allow the Timer1 to be set up before the
	// user needs to use it to possibly enter the system menu.
	encoder = new ClickEncoder(PIN_ENC_DT, PIN_ENC_CLK, PIN_ENC_SW, 4);

	Timer1.initialize(1 * MILLI_UNIT); // 1* MILLI_UNIT = 1,000ms = 1s
	Timer1.attachInterrupt(timerIsr);
#endif /* _LCDGUI_ */

	// Delay to allow entering eeprom reset on GUI or to click and open serial monitor on PC
	delay(500);

	// Initialize thermocouples
	pinMode(PIN_TC_CS1, OUTPUT);
	pinMode(PIN_TC_CS2, OUTPUT);

	// Set SSR pins as output
	pinMode(PIN_SSR1, OUTPUT);
	pinMode(PIN_SSR2, OUTPUT);

	tc1Status = thermocoupleSetup(thermocouple1);
	tc2Status = thermocoupleSetup(thermocouple2);

#ifdef _SERIALCMD_ || _DEVELOPMENT_ || defined _BOOTSYS_
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
#endif /* _SERIALCMD_ || _DEVELOPMENT_ || _BOOTSYS_*/

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
	myPID2.SetMode(AUTOMATIC);
	// Set PID sample time
	myPID1.SetSampleTime(pData.controlPeriod);
	myPID2.SetSampleTime(pData.controlPeriod);
	myPID1.SetOutputLimits(0, 255);
	myPID2.SetOutputLimits(0, 255);

	// Test if the pushbutton is pressed at boot time. If so then ensure entry to the system
	// menu by the issue of a boot button down event.
#ifdef _BOOTSYS_ &&_LCDGUI_
	mEvent = EV_BOOTDN;
#endif

#ifdef _LCDGUI_
	encoderEvent();
	mEvent == EV_BTN_HELD ? EV_BOOTDN : EV_NONE;
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

	int suDelay = 100;
	delay(suDelay);
}

// ------------------------------------------------------
// LOOP -------------------------------------------------
//


void loop()
{
	lcdUserStateMachine();
	readCheckTemp();
	checkSleep();	
	handleSerialCommands();
	printSerialData();
	updateEeprom();
	logSD(); // TODO

	currentMillis = millis();

	if (currentMillis - lastMillis >= pData.processInterval)
	{
		switch (currentProcessState)
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
			// Further operations for the active process
			processTimeManagement();
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
 *  to the setpoint plus a delta value. If the temperature readings exceed the setpoint plus
 *  the delta value for a certain number of cycles and if the output for that temperature
 *  is not issuing a zero output, a thermal runaway event is issued.
 */
void thermalRunawayCheck() // TODO
{

	// The drivingDeltaCounter holds how many cycles the temperature has been above the setpoint plus the 
	// delta value while still driving the output. If the temperature is below the setpoint plus the delta
	// value, the counter is reset to zero. If the counter reaches the number of cycles set in the program
	// data, a thermal runaway event is issued. drivingDeltaCounter only increases once per control period.
	

    unsigned long currentMillis = millis();
    // Check if enough time has passed for the next thermal runaway check
    if (currentMillis - lastThermalRunawayCheck >= pData.controlPeriod) 
    {
        // The actual thermal runaway check logic remains the same
        if(temp1 > pData.setTemp + pData.tempRunAwayDelta || temp2 > pData.setTemp + pData.tempRunAwayDelta)
        {
            // Check if the output is not zero
            if((int)Output1 > 0)
            {
                tempRunAwayAlarm1 = true;
                drivingDeltaCounter1++;
            }

            if((int)Output2 > 0)
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

        if(drivingDeltaCounter1 >= pData.tempRunAwayCycles || drivingDeltaCounter2 >= pData.tempRunAwayCycles)
        {
            currentProcessState = ERROR_PROCESS;
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

/*-------------------------------Check Sleep for Standby----------------------------------*/
/**
 *  \brief    Issues a standby timeout event if the standby time has expired without any activity.
 *  \remarks  The standby timeout period is defined in ms in the file header.
 */
void checkSleep()
{
#ifdef _LCDGUI_
	// \todo
	// make sure it doesnt go to sleep if the process is active

	// The last active time is updated every time some activity occurs. If the standby timeout
	// period has expired without any activity then a timeout event is issued.
	if (lastActiveTime + STANDBY_TIME_OUT < millis())
		mEvent = EV_STBY_TIMEOUT;
#endif /* _LCDGUI_ */
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

	// \todo
	// error ratio checking
}

// Function to check if temperature readings are NaN and perform error handling
void checkIsnan(double temp1, double temp2)
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

	// Check if processTimeComplete has reached processDuration

		// Check if temperature readings meet condition
		if (temp1 >= pData.setTemp && temp2 >= pData.setTemp)
		{
			tempConditionMet = min(tempConditionMet + 1, 5);
		}
		else
		{
			tempConditionMet = 0; // Reset counter if condition not met
		}

		if (lastSetTemp != pData.setTemp)
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
		lastSetTemp = pData.setTemp;
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
				// Assuming dt is for processDuration in minutes, converted to milliseconds
				pData.processDuration = (uint16_t)(received.substring(3).toInt() * MILLI_UNIT * 60);
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
				currentProcessState = ACTIVE_PROCESS;
			}
			else if (received == "OFF")
			{
				currentProcessState = INACTIVE_PROCESS;
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
		Serial.print(", TRA1 Alarm: ");
		Serial.print(tempRunAwayAlarm1 ? "WARNING" : "SAFE");
		Serial.print(", TRA2 Alarm: ");
		Serial.print(tempRunAwayAlarm2 ? "WARNING" : "SAFE");
		Serial.print(", t1: ");
		Serial.print(temp1);
		Serial.print(", t2: ");
		Serial.print(temp2);
		Serial.print(", st: ");
		Serial.print(pData.setTemp);
		Serial.print(", o1: ");
		Serial.print((int)Output1);
		Serial.print(", o2: ");
		Serial.print((int)Output2);
		Serial.print(", t: ");
		Serial.print((float)processTimeComplete / MILLI_UNIT / 60);
		Serial.print(", dt: ");
		Serial.print((float)pData.processDuration / MILLI_UNIT / 60);
		Serial.print(", Kp: ");
		Serial.print((float)pData.kp / MILLI_UNIT); 
		// \todo instead of floats, use a function to convert to string and shift float point
		Serial.print(", Ki: ");
		Serial.print((float)pData.ki / MILLI_UNIT);
		Serial.print(", Kd: ");
		Serial.print((float)pData.kd/ MILLI_UNIT); // Use println to add newline at the end
		Serial.print(", State: ");
		Serial.print(processStateToString(currentProcessState));
		Serial.println();

		lastSerialPrint = millis();
	}
#endif /* _SERIALCMD_ || _DEVELOPMENT_ || _BOOTSYS_*/
}

String processStateToString(ProcessState state)
{
	switch(state)
	{
	case INACTIVE_PROCESS:
		return "INACTIVE";
	case ACTIVE_PROCESS:
		return "ACTIVE";
	case ERROR_PROCESS:
		return "ERROR";
	case STANDBY_PROCESS:
		return "STANDBY";
	default:
		return "UNKNOWN";
	}
}


// Machine events
#ifdef _LCDGUI_
enum event
{
	// Private machine events
	EV_NONE,		 /**< Machine event: no pending event */
	EV_BTN_CLICKED,	 /**< Machine event: button pressed */
	EV_BTN_2CLICKED, /**< Machine event: button double pressed */
	EV_BTN_HELD,	 /**< Machine event: button held */
	EV_BTN_RELEASED	 /**< Machine event: button released */
		EV_ENCUP,	 /**< Machine event: encoder rotate right */
	EV_ENCDN,		 /**< Machine event: encoder rotate left */

	// Public machine events
	EV_BOOTDN,		 /**< Machine event: button pressed on boot */
	EV_STBY_TIMEOUT, /**< Machine event: standby timer has timed out */
};
#endif /* _LCDGUI_ */

/***************************************************************************************************
 * Click Button Encoder Event Processor                                                            *
 ***************************************************************************************************/
/**
 *  \brief    Processes the button and encoder events.
 *  \remarks  This function processes the button and encoder events and sets the machine event
 *            variable accordingly.
 */

void encoderEvent()
{
#ifdef _LCDGUI_
	static unsigned long lastEncTime = 0;

	// Check rotary action first since button takes precendence and we want to avoid
	// overwriting the button event and missing a button event.
	encNewPos += encoder->getValue();

	if (value != last)
	{
		lastActiveTime = lastEncTime = millis();

		if (encNewPos < encLastPos)
		{
			mEvent = EV_ENCDN;
		}
		else if (encNewPos > encLastPos)
		{
			mEvent = EV_ENCUP;
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
			mEvent = EV_BTN_CLICKED;
			break;
		case ClickEncoder::DoubleClicked:
			lastActiveTime = lastEncTime = millis();
			mEvent = EV_BTN_2CLICKED;
			break;
		case ClickEncoder::Held:
			lastActiveTime = lastEncTime = millis();
			mEvent = EV_BTN_HELD;
			break;
		case ClickEncoder::Released:
			lastActiveTime = lastEncTime = millis();
			mEvent = EV_BTN_RELEASED;
			break;
		default:
			mEvent = EV_NONE;
			break;
		}
	}
#endif /* _LCDGUI_ */
}

/***************************************************************************************************
 * State Machine                                                                                    *
 ***************************************************************************************************/
/**
 *  \brief  Implementation of state machine.
 */

// Machine states
#ifdef _LCDGUI_
enum states
{
	ST_STANDBY,		/**< Machine state: standby */
	ST_HOME_SCREEN, /**< Machine state: home screen */

	ST_STANDBY_SCREEN, /**< Machine state: standby screen */
	ST_SYSTEM_SCREEN,  /**< Machine state: display system screen, currently only manual eeprom reset */
};
#endif /* _LCDGUI_ */

void lcdUserStateMachine()
{
#ifdef _LCDGUI_
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
		// MACHINE EVENTS -----------------------
		switch (mEvent)
		{
		case EV_STBY_TIMEOUT:
			mState = ST_STANDBY_SCREEN;
			mEvent = EV_NONE;
			break;
		default:
			break;
		}

		// MACHINE STATES -----------------------
		switch (mState)
		{

		case ST_PULSE_VOLTAGE:

			break;

		default:
			break;
		}
	}
#endif /* _LCDGUI_ */
}

#ifdef _LCDGUI_

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

#endif /* _LCDGUI_ */

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
	pData.processDuration = DEF_PROCESS_DURATION;
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
