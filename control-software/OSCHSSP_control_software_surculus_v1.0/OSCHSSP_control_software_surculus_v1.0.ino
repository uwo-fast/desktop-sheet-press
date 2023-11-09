// Libraries for PID and Thermocouples
#include <PID_v1.h>
#include <Wire.h>
#include "Adafruit_MAX31855.h"

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
double dutyCycle1 = 0*0.01; // Purely illustrative, 
double dutyCycle2 = 0*0.01; // e.g. 50*0.01 is a 50% duty cycle
unsigned long cycleStart1 = 0;
unsigned long cycleStart2 = 0;

// Initializing temperatures and set points for PID
double setTemp = 0.0; 
double temp1 = 0.0;
double temp2 = 0.0;
double lastValidTemp1 = 0.0;  // * Varaible to preserve the last valid temp
double lastValidTemp2 = 0.0;  // * For display reasons
bool validTemp1 = false;  // * Varaible to track if the current displayed
bool validTemp2 = false;  // * temp is a the true valid one or last valid

// Process varaibles
unsigned long processDuration = 5.0*1000*60; // Mainly illustrative, e.g. 1.0*1000*60*60 = 1 minute
unsigned long processTime = 0;
unsigned long processTimeTotal = 0;
unsigned long currentMillis = 0;
unsigned long lastMillis = 0;  // Variable to hold the last update time
const long processLoopInterval = 10;  // Interval for delay (10 milliseconds)

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

// Serial print interval settings ???!!?!???!????!
unsigned long serialPrintStart = 0;
const unsigned long serialPrintInterval = 1000;

// For Rotary Encoder
// #include <ClickEncoder.h>
// #include <TimerOne.h>

// Initialize rotary encoder + button
// #define encSW 2
// #define encDT 3
// #define encCLK 4
// ClickEncoder *encoder;



Interface _interface;

// Use OpenAssistant or ChatGPT to make these to save a lot of time and keep it organized
// Alternatively you can write it as a string variable and cast it using (unsigned char*)

const int setupScreenRows = 4;
const int setupScreenCols = 21; // increased by 1 to accommodate null terminating character
char setupScreen[setupScreenRows][setupScreenCols] = {
  // Note the use of the '\0' null-terminating character, a standard convention in C and C++ to mark the end of a string
  // The LCD driver functions that handle text printing start at the beginning of the string and continue until they encounter the '\0' character.

  // Row 1
  { 'F', 'A', 'S', 'T', ' ', 'R', 'e', 's', 'e', 'a', 'r', 'c', 'h', ' ', 'G', 'r', 'o', 'u', 'p', ':', '\0' },
      
  // Row 2
  { 'O', 'p', 'e', 'n', ' ', 'S', 'o', 'u', 'r', 'c', 'e', ' ', 'C', 'o', 'l', 'd', ' ', 'a', 'n', 'd', '\0' },
  
  // Row 3
  { 'H', 'o', 't', ' ', 'S', 'c', 'i', 'e', 'n', 't', 'i', 'f', 'i', 'c', ' ', 'S', 'h', 'e', 'e', 't', '\0' },
  
  // Row 4
  { 'P', 'r', 'e', 's', 's', ' ', '-', '>', 'L', 'o', 'a', 'd', 'i', 'n', 'g', ' ', ' ', ' ', ' ', ' ', '\0' }
};

// Loading dots for printing
char dddSetupScreen[3][6] = {
  { '.', ' ', ' ', ' ', ' ', '\0' },
  { '.', ' ', '.', ' ', ' ', '\0' },
  { '.', ' ', '.', ' ', '.', '\0' }
};

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

// Setup Main
void setup() {

  int suDelay = 100;

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
  // Update the screen based on TC results
  setupScreens(5);
  delay(suDelay);
  setupScreens(6);
  delay(suDelay);

  // Set SSR pins as output
  pinMode(SSR1, OUTPUT);
  pinMode(SSR2, OUTPUT);
  // Check SSR lights
  checkSSR(1);
  checkSSR(2);
  // Update the screen based on SSR results
  setupScreens(7);
  delay(suDelay);
  setupScreens(8);
  delay(suDelay);
  clearScreen();
  clearScreen();
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

  delay(suDelay);
}

void timerIsr() {
  encoder->service();
}


// ------------------------------------------------------
// LOOP -------------------------------------------------
// ------------------------------------------------------


// MAIN LOOP @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void loop() {
  handleSerialCommands();
}
// MAIN LOOP @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


// PROCESS PREP-----------------------------------------
// DEG SYMBOL: (char)223

// Process prep loop
void processPrep(bool processPrepFlag) {

  while(processPrepFlag){

    handleSerialCommands();

    // GUI Updates based on flags
    // GUI menu update
    processEncRotary(encRotationFlag);
    processEncSwitch(buttonPressFlag); //SW vs button change throughout
    menuUpdateGUI(processPrepGUI, dispGUIIndex, cursGUIIndex, encRotationFlag);
    }
}


// PROCESS -----------------------------------------------

// Function that contains the main process loop
void processLoop() {

  unsigned long lastMillis = millis(); 
  unsigned long currentMillis = lastMillis;
  
  int tempConditionMet = 0;
  bool processDone = false;
  double lastSetTemp = 0;

  while (!processDone) {  // Loop continues until process is done

    // Update total process run time
   processTimeTotal += processLoopInterval;

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
      slowPWM(SSR1, cycleStart1, period, dutyCycle1);
      slowPWM(SSR2, cycleStart2, period, dutyCycle2);

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
  
  digitalWrite(SSR1, LOW);
  digitalWrite(SSR2, LOW);
}

// Function to implement slow PWM for SSR control
void slowPWM(int SSRn, unsigned long& cycleStart, double period, double dutyCycle) {
  // Get current time in milliseconds
  unsigned long currentMillis = millis();
  // Calculate the duty cycle as a percent based on PID output
  double calculatedDutyCycle = Output1 / 255.0;

  // If within the ON part of the cycle, turn the SSR on  
  if (currentMillis - cycleStart < period * calculatedDutyCycle) {
    digitalWrite(SSRn, HIGH);
  } 
  // If within the OFF part of the cycle, turn the SSR off
  else if (currentMillis - cycleStart < period) {
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

  if (millis() - serialPrintStart > period) {
    if(isnan(temp1) || isnan(temp2)) {
      Serial.println("Error: thermocouple failure.");
      serialPrintStart = millis();
    }
  }
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
      else if(received.startsWith("tt=")) {
        processTimeTotal = (long)(received.substring(3).toDouble() * 1000 * 60); // converting minutes to milliseconds
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
      else if(received.equals("loop")) {
        loop();
      }
      else if(received.equals("process")) {
        processLoop();
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
    Serial.print(", tt: ");
    Serial.print((float)processTimeTotal / 1000 / 60);
    Serial.print(", t: ");
    Serial.print((float)processTime / 1000 / 60);
    Serial.print(", dt: ");
    Serial.print((float)processDuration / 1000 / 60);
    Serial.print(", aggKp: ");
    Serial.print(aggKp);
    Serial.print(", aggKi: ");
    Serial.print(aggKi);
    Serial.print(", aggKd: ");
    Serial.println(aggKd); // Use println to add newline at the end

    serialPrintStart = millis();
  }
}