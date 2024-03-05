// Libraries for PID and Thermocouples
#include <PID_v1.h>
#include <Wire.h>
#include "Adafruit_MAX31855.h"
#include <EEPROM.h>

/*=======================================================================================
                                User Configurarations
========================================================================================*/
#define	_STANDBY_TIME_OUT	(300000)	//in mSeconds

const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 511;

/*=======================================================================================
                                    Definitions
========================================================================================*/
#define _ENCODER_EN

#define STANDBY 				0
#define MAIN_SCREEN 		1
#define MAIN_SCREEN_CNT 2
#define MENU_SCREEN 		3
#define SUB_MENU_1 			4
#define SUB_MENU_2 			5
#define BATTERY_LOW			6

/*=======================================================================================
                                  Globle Variables
========================================================================================*/
long lastVirtualPosition = 0;
uint8_t selectedMenu = 0;
uint8_t State = MAIN_SCREEN;
uint8_t setTemp_1 = 0;
uint8_t setTemp_2 = 0;
uint8_t setTemp_2 = 0;
uint16_t lastProcessCount = 0, lastSetTemp = 0;


typedef struct AllData{
	uint8_t delay;
	uint8_t failureAlarms;
	uint16_t processCount;

}allData;

allData gAllData;

uint8_t Delay 				= 20;
uint8_t FailureAlarms 	= 0;
uint16_t ProcessCount 		= 0;


long VirtualPosition = 0;
static unsigned long lastInterruptTime = 0;

//-------------------------------------------------------------------------------------------- 

// Initialization and Setup
// Define Chip Select pins and I2C pins for MAX31855s
#define CS1 5
#define CLK 6
#define CS2 7
#define DO 8

// Initializing thermocouples 1 and 2
Adafruit_MAX31855 thermocouple1(CLK, CS1, DO);
Adafruit_MAX31855 thermocouple2(CLK, CS2, DO);

// Initializing the setup error flags
bool tcInitFlag1 = false;
bool tcInitFlag2 = false;

// Initializing error flags
uint8_t errorFlagsMAX[] = {0, 0, 0, 0, 0, 0};

// Define pins for SSRs
#define SSR1 11 // 14 //A0
#define SSR2 12 // 15 //A1

// Flags for light error check
bool ssrInitFlag1 = false;
bool ssrInitFlag2 = false;

// Define period and duty cycle for SSR control
const unsigned long period = 1*1000; 
unsigned long cycleStart1 = 0;
unsigned long cycleStart2 = 0;

// Initializing temperatures and set points for PID
double setTemp = 0.0; 
double temp1 = 0.0;
double temp2 = 0.0;
double lastValidTemp1 = 0.0;  // * Varaible to preserve the last valid temp 1 reading
double lastValidTemp2 = 0.0;  // * Varaible to preserve the last valid temp 2 reading
bool validTemp1 = false;  // * Variable to indicate if the temp 1 reading is valid
bool validTemp2 = false;  // * Variable to indicate if the temp 2 reading is valid

// Process varaibles
unsigned long processDuration = 5.0*1000*60; // format is for illustrative effect, e.g. 1.0*1000*60*60 = 1 minute
unsigned long processTime = 0;
unsigned long currentMillis = 0;
unsigned long lastMillis = 0;  // Variable to hold the last update time
const long processLoopInterval = 10;  // Interval time for the process loop
unsigned long lastActiveTime;

// Process Parameters
double Setpoint, Input1, Output1, Input2, Output2;
double Cp=1, Ci=1, Cd=1;  // Constants to divide aggressive params to get the conservative params
double aggKp=1, aggKi=0.1, aggKd=0.02; // Aggressive PID Tuning Params
double consKp=aggKp/Cp, consKi=aggKi/Ci, consKd=aggKd/Cd; // Conservative PID Tuning Params

double gapThres = 5;  // Gap threshold for the dynamic tunings

// PID object setup for both thermocouples

// PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);

PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint, consKp, consKi, consKd, DIRECT);

// Serial print interval settings 
unsigned long serialPrintStart = 0;
const unsigned long serialPrintInterval = 1000;

// ------------------------------------------------------
// SETUP -------------------------------------------------
// ------------------------------------------------------

// Setup helpers

void thermocoupleSetup(Adafruit_MAX31855& thermocouple, int tcNum) {
  Serial.println("Initializing TC sensor...");
  // Check if thermocouple initialization fails
  if (!thermocouple.begin()) {
    if(tcNum == 1) {
      tcInitFlag1 = true;
    } else if (tcNum == 2) {
      tcInitFlag2 = true;
    }
    Serial.println("Thermocouple " + String(tcNum) + " ERROR.");
  } else {
    if(tcNum == 1) {
      tcInitFlag1 = false;
    } else if (tcNum == 2) {
      tcInitFlag2 = false;
    }
    Serial.println("Thermocouple " + String(tcNum) + " OK.");
  }
}

void checkSSR(int ssrNum) {
  Serial.println("Checking SSR...");
  // Check if thermocouple initialization fails
  if (false) { //!!!!!!This should be the light check function for photores!!!!!!!! TODO
    if(ssrNum == 1) {
      tcInitFlag1 = false;
    } else if (ssrNum == 2) {
      tcInitFlag2 = false;
    }
    Serial.println("SSR " + String(ssrNum) + " ERROR.");
  } else {
    if(ssrNum == 1) {
      tcInitFlag1 = false;
    } else if (ssrNum == 2) {
      tcInitFlag2 = false;
    }
    Serial.println("SSR " + String(ssrNum) + " OK.");
  }
}

void eepromSetup() {
  Serial.println("Initializing EEPROM...");
	byte tmpArraEe[10],tmp2ArrEe[10];
	eeprom_read_bytes(0,tmpArraEe,8);
	
	if( memcmp(tmpArraEe,&gAllData,8) != 0){
		if( tmpArraEe[6] == 0xFF ){
			memcpy(tmp2ArrEe, &gAllData,8);
			eeprom_write_bytes(0,tmp2ArrEe,8);
		}
		else{	
			memcpy(&gAllData,tmpArraEe,8);
		}
	}
	eepromReset();
	
	Delay					=		gAllData.delay 				;
	BatteryAlarm	=		gAllData.failureAlarms ;
	WeldCount			=		gAllData.processCount 		;

	Serial.println(gAllData.delay);
	Serial.println(gAllData.failureAlarms);
	Serial.println(gAllData.processCount);

}

void updateEeprom(){
	byte tmpArraEe[10],tmp2ArrEe[10];
	if( gAllData.delay != Delay || \
			gAllData.failureAlarms != FailureAlarms || \
			gAllData.processCount != ProcessCount || ){
		
		gAllData.delay 					= Delay;
		gAllData.failureAlarms 	= FailureAlarms;
		gAllData.processCount 			= ProcessCount;

		
		memcpy(tmp2ArrEe, &gAllData,8);
		eeprom_write_bytes(0,tmp2ArrEe,8);
		Serial.println("Updated eeprom");	
	}
}


// Setup Main
void setup() {

  int suDelay = 100;
  lastActiveTime = millis();

  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);  // max is prob 115200
  delay(500);
    
  // Timer1.initialize(1000);
  // Timer1.attachInterrupt(timerIsr); 

  // Initialize thermocouples
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  thermocoupleSetup(thermocouple1, 1);
  thermocoupleSetup(thermocouple2, 2);

  // Set SSR pins as output
  pinMode(SSR1, OUTPUT);
  pinMode(SSR2, OUTPUT);
  // Check SSR lights
  checkSSR(1);
  checkSSR(2);

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
  myPID1.SetOutputLimits(10*2.55, 255);
  myPID2.SetOutputLimits(10*2.55, 255);

  eepromSetup();

  delay(suDelay);
}

//void timerIsr() {
  //encoder->service();
//}


// ------------------------------------------------------
// LOOP -------------------------------------------------
//
void loop() {
  
  int tempConditionMet = 0;
  bool processDone = false;
  double lastSetTemp = 0;

  StateMachine();
  CheckForSleep();
  updateEeprom();

  // Check if it has been 20 milliseconds since the last update
  currentMillis = millis();
  if (currentMillis - lastMillis >= processLoopInterval) {

    // Read temperature from both thermocouples
    temp1 = thermocouple1.readCelsius();
    temp2 = thermocouple2.readCelsius();
    // temp2 = temp1;

    // Check if temperature readings are NaN and perform error handling
    checkIsnan(temp1, temp2);

    // Update Input values for PID controllers
    Input1 = temp1;
    Input2 = temp2;
    Setpoint = setTemp;


    // Dynamically adjust PID tuning parameters based on gap to setpoint
    dynamicTuning();

    // Compute new Output values with PID controllers
    myPID1.Compute();
    myPID2.Compute();

    // Implement slow PWM for SSR control
    slowPWM(SSR1, cycleStart1, period, Output1);
    slowPWM(SSR2, cycleStart2, period, Output2);

    // Print thermocouple data and errors
    printSerialData();

    handleSerialCommands();
    
    // Update the last update time
    lastMillis = currentMillis;

    // Check if temperature readings meet condition
    if (temp1 >= setTemp && temp2 >= setTemp) {
      if (tempConditionMet < 5) {
        tempConditionMet++;
      }
    } else {
      tempConditionMet = 0; // Reset counter if condition not met
    }

    if(lastSetTemp != setTemp) tempConditionMet = 0;

    // Update the process time
    if (tempConditionMet == 5) {
      processTime += processLoopInterval;
    }

    // Check if processTime has reached processDuration
    if (processTime >= processDuration) {
      processDone = true;
    }
  }
  lastSetTemp = setTemp;
}


// Function to implement slow PWM for SSR control
void slowPWM(int SSRn, unsigned long& cycleStart, double period, double output) {
  // Get current time in milliseconds
  unsigned long currentMillis = millis();

  double dutyCycle = output / 255;

  // If within the ON part of the cycle, turn the SSR on  
  if (currentMillis - cycleStart < period * dutyCycle) {
    digitalWrite(SSRn, HIGH);
  } 
  // If within the OFF part of the cycle, turn the SSR off
  else if (currentMillis - cycleStart >= period * dutyCycle && currentMillis - cycleStart < period) {
    digitalWrite(SSRn, LOW);
  } 
  // If the cycle is complete, reset the cycle start time
  else {
    cycleStart = currentMillis;
  }
}

// Function to check if temperature readings are NaN and perform error handling
void checkIsnan(double& temp1, double& temp2) {
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

// Function to dynamically adjust PID tuning parameters based on gap to setpoint
void dynamicTuning() {
  double gap1 = abs(Setpoint-Input1); 
  if (gap1 < gapThres) {  // Less aggressive tuning parameters for small gap
    myPID1.SetTunings(consKp, consKi, consKd);
  }
  else {  // More aggressive tuning parameters for large gap
     myPID1.SetTunings(aggKp, aggKi, aggKd);
  }

  double gap2 = abs(Setpoint-Input2); 
  if (gap2 < gapThres) {  // Less aggressive tuning parameters for small gap
    myPID2.SetTunings(consKp, consKi, consKd);
  }
  else {  // More aggressive tuning parameters for large gap
     myPID2.SetTunings(aggKp, aggKi, aggKd);
  }
}


// Functon to allow serial navigation and control
void handleSerialCommands() {
  static String received = "";
  while(Serial.available() > 0) {
    char inChar = (char)Serial.read();
    received += inChar;
    
    if(inChar == '\n') { // when a complete command is received
      received.trim(); // remove potential leading/trailing white space
      
      if(received.startsWith("t1=")) {
        temp1 = received.substring(3).toDouble();
      }
      else if(received.startsWith("t2=")) {
        temp2 = received.substring(3).toDouble();
      }
      else if(received.startsWith("st=")) {
        setTemp = received.substring(3).toDouble();
      }
      else if(received.startsWith("o1=")) {
        Output1 = received.substring(3).toDouble();
      }
      else if(received.startsWith("o2=")) {
        Output2 = received.substring(3).toDouble();
      }
      else if(received.startsWith("dt=")) {
        processDuration = (long)(received.substring(3).toDouble() * 1000 * 60); // converting minutes to milliseconds
      }
      else if(received.startsWith("t=")) {
        processTime = (long)(received.substring(2).toDouble() * 1000 * 60); // converting minutes to milliseconds
      }
      else if(received.startsWith("kp=")) {
        aggKp = received.substring(3).toDouble();
      }
      else if(received.startsWith("ki=")) {
        aggKi = received.substring(3).toDouble();
      }
      else if(received.startsWith("kd=")) {
        aggKd = received.substring(3).toDouble();
      }
      
      received = ""; // clear received data
    }
  }
}

void printSerialData() {
  if (millis() - serialPrintStart > serialPrintInterval) {

    Serial.print("Error flags: ");

    int errorFlagsMAXSize = sizeof(errorFlagsMAX)/sizeof(errorFlagsMAX[0]);
    // Print error flags    
    for (int i = 0; i < errorFlagsMAXSize; i++) {
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



void eepromReset(){
	if( encBtnState()==LOW ){
		byte tmpArraEe[10],tmp2ArrEe[10];		
			
		gAllData.delay 					= 20;
		gAllData.batteryAlarm 	= 110;
		gAllData.weldCount 			= gAllData.weldCount;
		gAllData.pulseDelay 		= 5;
		gAllData.shortPulse			= 12;
		gAllData.autoPulse			= 1;
			
		memcpy(tmp2ArrEe, &gAllData,8);
		eeprom_write_bytes(0,tmp2ArrEe,8);
		Serial.println("EEPROM reset");
		
		while(!encBtnState());
	}
}

/*-------------------------------Check Sleep for Standby----------------------------------*/
void CheckForSleep(){
	/*
	if(lastActiveTime + 1000 < millis() ){
		inactiveTime += millis() - lastActiveTime;
		lastActiveTime = millis();
		//Serial.println(inactiveTime);
	
		if(inactiveTime > _STANDBY_TIME_OUT){//(1000*60*_STANDBY_TIME_OUT)){			
			//Serial.println("Enter Standby Mode");
			inactiveTime = 0;
		}
	}		
	*/
	if(lastActiveTime + _STANDBY_TIME_OUT < millis() ){
		if(State!=BATTERY_LOW){
			State = STANDBY;
			standByMsg();
		}
	}	
}

/*-----------------------------------State Machine--------------------------------------*/
void StateMachine(){
	char tmpAr1[5];
	char tmpAr2[5];
	char tmpAr3[5];
	static uint8_t selectedMenu = 0;
	static uint8_t selectedMainMenu = 0;
	static uint8_t selectedSubMenu = 0;
	
	switch(State){
		case STANDBY:
				if( encBtnState()==LOW ){
					State = MAIN_SCREEN;
					delay(30);
					//set all the functionalities anable;
					while(!encBtnState());
				}
			break;
			
		case BATTERY_LOW:
			
		break;
		
		case MAIN_SCREEN:
			State = MAIN_SCREEN_CNT;
			selectedMenu = 0;
			memset(tmpAr1,0,5);
			memset(tmpAr2,0,5);
			memset(tmpAr3,0,5);
			
			tmpAr1[0] = BatteryVoltage/100+ '0';	tmpAr1[1] = BatteryVoltage%100/10+ '0';	tmpAr1[2] = '.';												tmpAr1[3] = BatteryVoltage%100%10+ '0';
			tmpAr2[0] = WeldCount/1000+ '0';			tmpAr2[1] = WeldCount%1000/100+ '0';		tmpAr2[2] = WeldCount%1000%100/10+ '0';	tmpAr2[3] = WeldCount%1000%100%10+ '0';
			tmpAr3[0] = PulseDelay/1000+ '0';			tmpAr3[1] = PulseDelay%1000/100+ '0';		tmpAr3[2] = PulseDelay%1000%100/10+ '0';	tmpAr3[3] = PulseDelay%1000%100%10+ '0';
			
			mainScreen(tmpAr3,tmpAr1,tmpAr2);
			
			break;
		
		case MAIN_SCREEN_CNT:
			if( encBtnState()==LOW ){
				State = MENU_SCREEN;	
				selectedMenu = 0;			
				displayMainMenu(0);
				delay(30);
				//set all the functionalities anable;
				while(!encBtnState());
				break;
			}
			
			if( lastVirtualPosition != VirtualPosition ){			
				if( lastVirtualPosition > VirtualPosition ){
					PulseDelay--;					
				}
				else{		
					PulseDelay++;
				}
				if( PulseDelay <= 1 ) PulseDelay = 1;	
				else if( PulseDelay >= 999)	PulseDelay = 999;	
				
				//PulseDelay = selectedMenu;
				lastVirtualPosition = VirtualPosition;					
			}								
				
			if(lastBatterVoltage!=BatteryVoltage || lastWeldCount != WeldCount || lastPulseDelay != PulseDelay){
				memset(tmpAr1,0,5);
				memset(tmpAr2,0,5);
				memset(tmpAr3,0,5);
				
				tmpAr1[0] = BatteryVoltage/100+ '0';	tmpAr1[1] = BatteryVoltage%100/10+ '0';	tmpAr1[2] = '.';												tmpAr1[3] = BatteryVoltage%100%10+ '0';
				tmpAr2[0] = WeldCount/1000+ '0';			tmpAr2[1] = WeldCount%1000/100+ '0';		tmpAr2[2] = WeldCount%1000%100/10+ '0';	tmpAr2[3] = WeldCount%1000%100%10+ '0';
				tmpAr3[0] = PulseDelay/1000+ '0';			tmpAr3[1] = PulseDelay%1000/100+ '0';		tmpAr3[2] = PulseDelay%1000%100/10+ '0';	tmpAr3[3] = PulseDelay%1000%100%10+ '0';
				
				mainScreen(tmpAr3,tmpAr1,tmpAr2);
				
				lastBatterVoltage=BatteryVoltage;
				lastWeldCount = WeldCount;
				lastPulseDelay = PulseDelay;
				
			}
			
			//other functions
			break;
		
		case MENU_SCREEN:		
			if( encBtnState()==LOW ){
				delay(30);
				State = SUB_MENU_1;
				selectedMainMenu = selectedMenu;
				if(selectedMainMenu == 0){
					displaySubMenuType1(selectedMenu);
				}
				else if(selectedMainMenu == 1){
					uint8_t value = BatteryAlarm;		
					memset(tmpAr1,0,5);				
					tmpAr1[0] = value/100+'0';
					tmpAr1[1] = value%100/10+'0';
					tmpAr1[2] = '.';
					tmpAr1[3] = value%100%10+'0';
					displaySubMenuType2("Battery Alarm: ", tmpAr1 , "voltage");
				}
				else if(selectedMainMenu == 2){
					uint8_t value = ShortPulse;
					memset(tmpAr1,0,5);	
					tmpAr1[0] = value/100+'0';
					tmpAr1[1] = value%100/10+'0';
					tmpAr1[2] = value%100%10+'0';
					displaySubMenuType2("Short Pulse: ", tmpAr1 , "% of pulse time");		
				}
				selectedMenu = 0;
				//set all the functionalities anable;
				while(!encBtnState());	
				break;
			}
			
			if( lastVirtualPosition != VirtualPosition ){		
				if( lastVirtualPosition > VirtualPosition ){
					selectedMenu--;
				}
				else{		
					selectedMenu++;
				}
				if( selectedMenu >= 250){
					selectedMenu = 0;
				}
				else if( selectedMenu >= 2){
					selectedMenu = 2;
				}
				
				lastVirtualPosition = VirtualPosition;
				displayMainMenu(selectedMenu);
			}		
			break;
		
		case SUB_MENU_1:
			if( encBtnState()==LOW ){
				delay(30);
				//set all the functionalities anable;
				if(selectedMainMenu==0){
					State = SUB_MENU_2;
					selectedSubMenu = selectedMenu;
					
					if(selectedSubMenu==0){
						if(AutoPulse == 0){
							displaySubMenuType2("Auto Pulse: On/Off", "ON" , " ");
							AutoPulse = 1;
						}
						else if(AutoPulse == 1){
							displaySubMenuType2("Auto Pulse: On/Off", "OFF" , " ");
							AutoPulse = 0;
						}
					}
					else if(selectedSubMenu==1){
						uint8_t value = Delay;
						memset(tmpAr1,0,5);				
						tmpAr1[0] = value/10+'0';
						tmpAr1[1] = '.';
						tmpAr1[2] = value%10+'0';
						displaySubMenuType2("Auto Pulse: Delay", tmpAr1 , "seconds");
					}
					else{
					
					}
					selectedMenu = 0;
				}
				else if(selectedMainMenu == 1){					
					State = MAIN_SCREEN;
					selectedMenu = 0; 
				}
				else if(selectedMainMenu == 2){					
					State = MAIN_SCREEN;
					selectedMenu = 0;
				}
				else{			
					State = MAIN_SCREEN;
					selectedMenu = 0;
				}
				while(!encBtnState());	
				selectedMenu = 0;
				break;
			}
			
			if(selectedMainMenu==0){
				if( lastVirtualPosition != VirtualPosition ){		
					if( lastVirtualPosition > VirtualPosition ){
						selectedMenu--;
					}
					else{		
						selectedMenu++;
					}
					
					if( selectedMenu >= 3){
						selectedMenu = 2;
					}
					
					lastVirtualPosition = VirtualPosition;
					displaySubMenuType1(selectedMenu);
				}					
			}
			else if(selectedMainMenu == 1){
				
				if( lastVirtualPosition != VirtualPosition ){						
					
					if( lastVirtualPosition > VirtualPosition ){
						BatteryAlarm--;
						
					}
					else{		
						BatteryAlarm++;
					}
					if( BatteryAlarm <= 100 ) BatteryAlarm = 100;	
					else if( BatteryAlarm >= 200)	BatteryAlarm = 200;					
					
					lastVirtualPosition = VirtualPosition;	
					
					uint8_t value = BatteryAlarm;			
					memset(tmpAr1,0,5);				
					tmpAr1[0] = value/100+'0';
					tmpAr1[1] = value%100/10+'0';
					tmpAr1[2] = '.';
					tmpAr1[3] = value%100%10+'0';
					displaySubMenuType2("Battery Alarm: ", tmpAr1 , "voltage");				
				}								
			}
			else if(selectedMainMenu == 2){
				if( lastVirtualPosition != VirtualPosition ){					
					
					if( lastVirtualPosition > VirtualPosition ){
						ShortPulse--;
					}
					else{		
						ShortPulse++;
					}
					if( ShortPulse <= 1 ) ShortPulse = 1;
					else if( ShortPulse >= 100)	ShortPulse = 100;									
					
					lastVirtualPosition = VirtualPosition;	
					uint8_t value = ShortPulse;
					memset(tmpAr1,0,5);	
					tmpAr1[0] = value/100+'0';
					tmpAr1[1] = value%100/10+'0';
					tmpAr1[2] = value%100%10+'0';
					displaySubMenuType2("Short Pulse: ", tmpAr1 , "% of pulse time");					
				}								
			}
			else{
			}
			
			break;
			
		case SUB_MENU_2:			
			if( encBtnState()==LOW ){
				delay(30);
				//set all the functionalities anable;
				State = MAIN_SCREEN;
				while(!encBtnState());	
				selectedMenu = 0;
				break;
			}
			
			if(selectedSubMenu==0){
				if( lastVirtualPosition != VirtualPosition ){
					
					
					if( lastVirtualPosition > VirtualPosition ){
						AutoPulse--;
					}
					else{		
						AutoPulse++;
					}
					if( AutoPulse <= 0 ) AutoPulse = 0;
					else if( AutoPulse >= 1)	AutoPulse = 1;

					
					lastVirtualPosition = VirtualPosition;
					if(AutoPulse == 0){
						displaySubMenuType2("Auto Pulse: On/Off", "ON" , " ");
					}
					else if(AutoPulse == 1){
						displaySubMenuType2("Auto Pulse: On/Off", "OFF" , " ");
					}	
				}
			}
			else if(selectedSubMenu==1){
				if( lastVirtualPosition != VirtualPosition ){					
					
					if( lastVirtualPosition > VirtualPosition ){
						Delay--;
					}
					else{		
						Delay++;
					}
					if( Delay >= 50)	Delay = 50;
					else if( Delay <= 5 ) Delay = 5;

					
					lastVirtualPosition = VirtualPosition;
					
					uint8_t value = Delay;
					memset(tmpAr1,0,5);				
					tmpAr1[0] = value/10+'0';
					tmpAr1[1] = '.';
					tmpAr1[2] = value%10+'0';
					displaySubMenuType2("Auto Pulse: Delay", tmpAr1 , "seconds");
				}
			}
			else{
			
			}	
			
			break;
			
		default:
			break;
	}
}

bool encBtnState(){	
	if(!digitalRead(PinSW)){
		lastActiveTime = millis();
	}
	return (digitalRead(PinSW));
}


/*--------------------------------------- isr ------------------------------------------*/
void isr(){                    
	unsigned long interruptTime = millis();
	if (interruptTime - lastInterruptTime > 20){
	/*
		if (digitalRead(PinCLK)){
			if(digitalRead(PinDT)){			
			}
			else{
				VirtualPosition++;		
			}
		}
		else{
			if(digitalRead(PinDT)){
				VirtualPosition--;
			}
		} 
		*/
		if(digitalRead(PinDT)){
			VirtualPosition--;
		}
		else{
			VirtualPosition++;		
		}
		lastInterruptTime = interruptTime;
		lastActiveTime = interruptTime;
	}
}


/*--------------------------------- Utility Functions ----------------------------------*/

//
// Writes a sequence of bytes to eeprom starting at the specified address.
// Returns true if the whole array is successfully written.
// Returns false if the start or end addresses aren't between
// the minimum and maximum allowed values.
// When returning false, nothing gets written to eeprom.
//
boolean eeprom_write_bytes(int startAddr, const byte* array, int numBytes) {
  // counter
  int i;

  // both first byte and last byte addresses must fall within
  // the allowed range  
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }

  for (i = 0; i < numBytes; i++) {
    EEPROM.write(startAddr + i, array[i]);
  }

  return true;
}

//
// Reads the specified number of bytes from the specified address into the provided buffer.
// Returns true if all the bytes are successfully read.
// Returns false if the star or end addresses aren't between
// the minimum and maximum allowed values.
// When returning false, the provided array is untouched.
//
// Note: the caller must ensure that array[] has enough space
// to store at most numBytes bytes.
//
boolean eeprom_read_bytes(int startAddr, byte array[], int numBytes) {
  int i;

  // both first byte and last byte addresses must fall within
  // the allowed range  
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }

  for (i = 0; i < numBytes; i++) {
    array[i] = EEPROM.read(startAddr + i);
  }

  return true;
}

//
// Returns true if the address is between the
// minimum and maximum allowed values,
// false otherwise.
//
// This function is used by the other, higher-level functions
// to prevent bugs and runtime errors due to invalid addresses.
//
boolean eeprom_is_addr_ok(int addr) {
  return ((addr >= EEPROM_MIN_ADDR) && (addr <= EEPROM_MAX_ADDR));
}