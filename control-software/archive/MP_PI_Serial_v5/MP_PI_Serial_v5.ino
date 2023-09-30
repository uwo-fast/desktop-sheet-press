// Libraries for PID and Thermocouples
#include <PID_v1.h>
#include <avr/io.h> // remove?
#include <Wire.h>
#include "Adafruit_MAX31855.h"

// For LCD
#include <stdint.h>
#include <stdlib.h>

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
#define SSR1 14 //A0
#define SSR2 15 //A1
// Flags for light error check
bool ssrInitFlag1 = false;
bool ssrInitFlag2 = false;

// Define period and duty cycle for SSR control
const unsigned long period = 10*1000; 
double dutyCycle1 = 0*0.01; // Purely illustrative, 
double dutyCycle2 = 0*0.01; // e.g. 50*0.01 is a 50% duty cycle
unsigned long cycleStart1 = 0;
unsigned long cycleStart2 = 0;

// Initializing temperatures and set points for PID
double desiredTemp = 0.0; 
double temp1 = 0.0;
double temp2 = 0.0;
double lastValidTemp1 = 0.0;  // * Varaible to preserve the last valid temp
double lastValidTemp2 = 0.0;  // * For display reasons
bool validTemp1 = false;  // * Varaible to track if the current displayed
bool validTemp2 = false;  // * temp is a the true valid one or last valid

// Process varaibles
unsigned long processDuration = 1.0*1000*60*60; // Mainly illustrative, e.g. 1.0*1000*60*60 = 1 minute
unsigned long processTime = 0;
unsigned long processTimeTotal = 0;
unsigned long lastProcessTime = 0;
unsigned long lastTotalTime = 0;
bool processDone = false;
unsigned long lastUpdateTime = 0;  // Variable to hold the last update time
const long processLoopInterval = 10;  // Interval for delay (10 milliseconds)

// Process Parameters
double Setpoint, Input1, Output1, Input2, Output2;
double Cp=3, Ci=3, Cd=3;  // Constants to divide aggressive params to get the conservative params
double aggKp=5, aggKi=1, aggKd=0; // Aggressive PID Tuning Params
double consKp=aggKp/Cp, consKi=aggKi/Ci, consKd=aggKd/Cd; // Conservative PID Tuning Params

double gapThres = 5;  // Gap threshold for the dynamic tunings

// PID object setup for both thermocouples
PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);

// Serial print interval settings ???!!?!???!????!
unsigned long serialPrintStart = 0;
const unsigned long serialPrintInterval = 1000;

// For Rotary Encoder
#include <ClickEncoder.h>
#include <TimerOne.h>

// Initialize rotary encoder + button
#define encSW 2
#define encDT 3
#define encCLK 4
ClickEncoder *encoder;
int16_t last, value;

// LCD initialization
#define STARTUP_DELAY 500
#define RS232_DELAY 100
#define I2C_DELAY 100
#define SLAVE_ADDRESS 0x28

// SPI Interface (TRY TO REMOVE)
uint8_t _SCL; // 5
uint8_t _SDI; // 4
uint8_t _CS; // 3

// RS232 Interface (TRY TO REMOVE)
uint8_t _TX; // 2

//I2C Interface
uint8_t _SDA; // 4

enum Interface{
  I2C,
  //SPI,
  //RS232
};

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
void setupScreens(int i) {
  switch (i) {
    case 1:
      // Loading screen LCD screen text
      setCursor(0x00);
      writeString(setupScreen[0]);
      setCursor(0x40);
      writeString(setupScreen[1]);
      setCursor(0x14);
      writeString(setupScreen[2]);
      setCursor(0x54);
      writeString(setupScreen[3]);     
      break;

    case 2:
      setCursor(0x63);
      writeString(dddSetupScreen[0]);
      break;

    case 3:
      setCursor(0x63);
      writeString(dddSetupScreen[1]);
      break;

    case 4:
      setCursor(0x63);
      writeString(dddSetupScreen[2]);
      break;

    case 5:
      setCursor(0x5C);
      if(tcInitFlag1){
        writeString((unsigned char*) "TC-1 ERROR.");
        while(1) delay(10);
      } else {
        writeString((unsigned char*) "TC-1 OK.    ");
      }
      break;

    case 6:
      setCursor(0x5C);
      if(tcInitFlag2){
        writeString((unsigned char*) "TC-2 ERROR.");
        while(1) delay(10);
      } else {
        writeString((unsigned char*) "TC-2 OK.    ");
      }
      break;

    case 7:
      setCursor(0x5C);
      if(ssrInitFlag1){
        writeString((unsigned char*) "SSR-1 ERROR.");
        while(1) delay(10);
      } else {
        writeString((unsigned char*) "SSR-1 OK.   ");
      }
      break;

    case 8:
      setCursor(0x5C);
      if(ssrInitFlag2){
        writeString((unsigned char*) "SSR-2 ERROR.");
        while(1) delay(10);
      } else {
        writeString((unsigned char*) "SSR-2 OK.  ");
      }
    case 9:
      setCursor(0x5A);
      writeString((unsigned char*) "              ");
      break;

    default:
      break;
  }
}

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

  // Initialize LCD
  initLCD_I2C(A5, A4);

  underlineCursorOFF();
  setupScreens(1);
  delay(suDelay);
  setupScreens(2);
  delay(suDelay);
  setupScreens(3);
  delay(suDelay);
  setupScreens(4);
  delay(suDelay);

  // Set encoder's DT & CLK pins to input
  pinMode (encDT, INPUT);
  pinMode (encCLK, INPUT);

  encoder = new ClickEncoder(encDT, encCLK, encSW);
    
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 

  last = -1;


  // Attach an interrupt to the encoder's DT  pin
  //attachInterrupt(digitalPinToInterrupt(encDT), handleEncRotary, CHANGE);

  // Set the encoder's switch pin to 
  //pinMode(encSW, INPUT_PULLUP);

  // Attach an interrupt to the encoder's switch pin
  //attachInterrupt(digitalPinToInterrupt(encSW), handleEncSwitch, FALLING);

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
  Setpoint = desiredTemp;
  // Set PID controllers to automatic mode
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  // Set PID sample time
  myPID1.SetSampleTime(period);
  myPID2.SetSampleTime(period);
  // Function to initialize thermocouples
  myPID1.SetOutputLimits(0, 255);
  myPID2.SetOutputLimits(0, 255);

  delay(suDelay);
}

// ------------------------------------------------------
// RUNTIME GUI ------------------------------------------
// ------------------------------------------------------

// Position variable for the encoder rotation
long oldPosition  = -999;

// Boolean flags for debounced interrupt tracking
//volatile bool encRotationRupt = false;
bool encRotationFlag = false;
//volatile bool buttonPressRupt = false;
bool buttonPressFlag = false;

// Does the current menu need to be updated
volatile bool dispGUIupdate = true; 

// Current cursor index
int cursGUIIndex = 1;
int maxGUIIndex = 1;

// Current display index; this indicates the array populating the top row
// Not used here (ATM???) in effect, but can allow menu scrolling
volatile int dispGUIIndex = 1;

// LOOP GUI Helpers
void menuUpdateGUI(char (*displayArray)[18], int dispGUIIndex) {
  underlineCursorOFF();
  blinkingCursorOFF();
  setCursor(0x00);
  writeString((unsigned char*) displayArray[dispGUIIndex-1]);
  setCursor(0x40);
  writeString((unsigned char*) displayArray[dispGUIIndex]);
  setCursor(0x14);
  writeString((unsigned char*) displayArray[dispGUIIndex+1]);
  setCursor(0x54);
  writeString((unsigned char*) displayArray[dispGUIIndex+2]);
  dispGUIupdate = false;
}

void cursorUpdateGUI(char (*displayArray)[18], int dispGUIIndex, int &cursGUIIndex, bool &encRotationFlag) {

  // Length should match number of cases which is the max number of indices possible
  // Max number of indices possible is the # of rows in the in the GUI with the max # of rows
  int maxIndices = 4;
  int indexMap[maxIndices]; // Here max indices is 4
  int j = 0;

  for (int i = dispGUIIndex-1; i < maxIndices; i++) {
  char typeFlag = displayArray[i][16];

    if(typeFlag == 's' || typeFlag == 'v') {
      switch(i){
        case 0:
          indexMap[j] = (typeFlag == 's') ? 0x00 : 0x0F;
          j++;
          break;
        case 1:
          indexMap[j] = (typeFlag == 's') ? 0x40 : 0x4F;
          j++;
          break;
        case 2:
          indexMap[j] = (typeFlag == 's') ? 0x14 : 0x23;
          j++;
          break;
        case 3:
          indexMap[j] = (typeFlag == 's') ? 0x54 : 0x63;
          j++;
          break;
        default:
          break;
      }
    }
  }
  maxGUIIndex = j;  
  underlineCursorON();  
  setCursor(indexMap[cursGUIIndex-1]);
  Serial.println("Cursor set to: " + String(cursGUIIndex));  // DEBUGGGGGGGGGGG
  encRotationFlag = false;
}

void buttonUpdateGUI(char (*displayArray)[18], int dispGUIIndex, int &cursGUIIndex, bool &buttonPressFlag) {

  // Length should match number of cases which is the max number of indices possible
  // Max number of indices possible is the # of rows in the in the GUI with the max # of rows
  int maxIndices = 4;
  int actionMap[maxIndices]; // Here max indices is 4

  int j = 0;

  for (int i = 0; i < maxIndices; i++) {
  char typeFlag = displayArray[i][16];

    if(typeFlag == 's' || typeFlag == 'v'){
      switch(i){
        case 0:
          actionMap[j] = displayArray[i][17];
          j++;
          break;
        case 1:
          actionMap[j] = displayArray[i][17];
          j++;
          break;
        case 2:
          actionMap[j] = displayArray[i][17];
          j++;
          break;
        case 3:
          actionMap[j] = displayArray[i][17];
          j++;
          break;
        default:
          break;
      }
    }
  }

  maxGUIIndex = j;
  cursGUIIndex = min(max(cursGUIIndex, 1), maxGUIIndex);
  buttonPressFlag = false;
  actionRouter(actionMap[cursGUIIndex-1]);
}

void actionRouter(char action){
  switch(action) {
    case 'p':
      processPrep();
      break;
    
    default:
      break;
  }
}


volatile unsigned long last_interrupt_time = 0;
const unsigned long rot_debounce_time = 50; // 5 ms debounce time
const unsigned long but_debounce_time = 50; // 50 ms debounce time

// Encoder interupt helper functions; rotary movement & switch press
void handleEncRotary() {
  encRotationRupt = true;
}

void processEncRotary() {
  unsigned long current_time = millis();

if (encRotationRupt && (current_time - last_interrupt_time >= rot_debounce_time)) {
    long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
      if (digitalRead(encDT)) {
        cursGUIIndex++;
        if (cursGUIIndex > maxGUIIndex) {
          cursGUIIndex = maxGUIIndex;  // Top out at max index
        }
      } else {
        cursGUIIndex--;
        if (cursGUIIndex < 1) {
          cursGUIIndex = 1;  // Bottom out at 1
        }
      }
      oldPosition = newPosition;
    }
    encRotationFlag = true;
  }

  encRotationRupt = false; // Clear the interrupt flag
  last_interrupt_time = current_time;
}


void handleEncSwitch() {
  buttonPressRupt = true;
}

void processEncSwitch() {
  unsigned long current_time = millis();

  // if enough time has passed since last interrupt
  if (buttonPressRupt && (current_time - last_interrupt_time >= but_debounce_time)) {
    buttonPressFlag = true;
  }

  buttonPressRupt = false; // Clear the interrupt flag  
  last_interrupt_time = current_time; // update the time of the last interrupt
}



void timerIsr() {
  encoder->service();
}




const int loopGUIRows = 4;
const int loopGUICols = 18; // all GUIs are limited to 15 to reserve 5 spots for vars 
                            // + 1 for terminator + 1 for type flag ('d' display, 's' select, 'v' vary) + 1 for action flag
char loopGUI[loopGUIRows][loopGUICols] = {
  // Row 1
  { 'O', 'S', 'C', 'H', 'S', 'S', 'P', ' ', 'v', '0', '.', '0', '.', '1', ' ', '\0', 'd', ' ' },
  
  // Row 2
  { '>', ' ', 'B', 'e', 'g', 'i', 'n', ' ', 'P', 'r', 'o', 'c', 'e', 's', 's', '\0', 's', 'p' },
  
  // Row 3
  { '>', ' ', 'T', 'u', 'n', 'i', 'n', 'g', ' ', 'P', 'a', 'r', 'a', 'm', ' ', '\0', 's', 't' },
  
  // Row 4
  { '>', ' ', 'S', 'e', 't', 't', 'i', 'n', 'g', 's', ' ', ' ', ' ', ' ', ' ', '\0', 's', 's' }
};

// ------------------------------------------------------
// LOOP -------------------------------------------------
// ------------------------------------------------------

// MAIN LOOP @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void loop() {
  // GUI Updates based on flags
  // GUI menu update
  if(encRotationRupt) processEncRotary();
  if(buttonPressRupt) processEncSwitch(); //SW vs button change throughout
  if(dispGUIupdate) menuUpdateGUI(loopGUI, dispGUIIndex);
  if(encRotationFlag) cursorUpdateGUI(loopGUI, dispGUIIndex, cursGUIIndex, encRotationFlag);
  if(buttonPressFlag) buttonUpdateGUI(loopGUI, dispGUIIndex, cursGUIIndex, buttonPressFlag);
}
// MAIN LOOP @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

// PROCESS PREP-----------------------------------------

const int processPrepGUIRows = 4;
const int processPrepGUICols = 21; // increased by 1
char processPrepGUI[processPrepGUIRows][processPrepGUICols] = {
  // Row 1
  { 'O', 'S', 'C', 'H', 'S', 'S', 'P', ' ', 'v', '0', '.', '0', '.', '1', ' ', ' ', ' ', ' ', ' ', ' ', '\0'},
  
  // Row 2
  { 'S', 'e', 't', 'p', 'o', 'i', 'n', 't', ' ', 't', 'e', 'm', 'p', ':', ' ', ' ', ' ', ' ', (char)223, 'C', '\0'},  // (char)223 --> deg symbol
  
  // Row 3
  { 'S', 'e', 't', 'p', 'o', 'i', 'n', 't', ' ', 't', 'i', 'm', 'e', ':', ' ', ' ', ' ', ' ', ' ', 'm', '\0'},
  
  // Row 4
  { '>', ' ', 'I', 'n', 'i', 't', 'i', 'a', 't', 'e', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'}
};

// Process prep menu handler
void processPrepGUIs(int i) {
  switch (i) {
    case 1:
      // Loading screen LCD screen text
      clearScreen();
      clearScreen();
      setCursor(0x00);
      writeString(processPrepGUI[0]);
      setCursor(0x40);
      writeString(processPrepGUI[1]);
      setCursor(0x14);
      writeString(processPrepGUI[2]);
      setCursor(0x54);
      writeString(processPrepGUI[3]);
      //loopGUICursor();
      underlineCursorON();
      blinkingCursorOFF();
      dispGUIupdate = 0;
      break;

    case 2:
      setCursor(0x4F);
      writeString((unsigned char*) String((int)desiredTemp).c_str());
      break;

    case 3:
      setCursor(0x23);
      writeString((unsigned char*) String(processDuration/1000/60/60).c_str());
      break;

    default:
      break;
  }
}

// Process prep loop
void processPrep() {
  processLoop();
}


// PROCESS -----------------------------------------------

// Function that contains the main process loop
void processLoop() {
  
  // Update lastProcessTime and lastTotalTime with the current time
  lastProcessTime = millis();
  lastTotalTime = millis();
  
  while (!processDone) {  // Loop continues until process is done

    // Get the current time
    unsigned long currentMillis = millis();

    // Check if it has been 20 milliseconds since the last update
    if (currentMillis - lastUpdateTime >= processLoopInterval) {

      // Read temperature from both thermocouples
      temp1 = thermocouple1.readCelsius();
      temp2 = thermocouple2.readCelsius();

      // Check if temperature readings are NaN and perform error handling
      checkIsnan(temp1, temp2);

      // Update Input values for PID controllers
      Input1 = temp1;
      Input2 = temp2;

      // Dynamically adjust PID tuning parameters based on gap to setpoint
      dynamicTuning();

      // Compute new Output values with PID controllers
      myPID1.Compute();
      myPID2.Compute();

      // Implement slow PWM for SSR control
      slowPWM(SSR1, cycleStart1, period, dutyCycle1);
      slowPWM(SSR2, cycleStart2, period, dutyCycle2);

      // Print thermocouple data and errors
      printThermocoupleDataPrint(temp1, temp2, desiredTemp, Output1, Output2, errorFlagsMAX, sizeof(errorFlagsMAX)/sizeof(errorFlagsMAX[0]));

      // Update the last update time
      lastUpdateTime = currentMillis;

      // Update processTime and processTimeTotal
      if (temp1 >= desiredTemp && temp2 >= desiredTemp) {
        processTime += currentMillis - lastProcessTime;
      }
      processTimeTotal += currentMillis - lastTotalTime;

      // Update lastProcessTime and lastTotalTime for the next iteration
      lastProcessTime = currentMillis;
      lastTotalTime = currentMillis;
      
      // Check if processTime has reached processDuration
      if (processTime >= processDuration) {
        processDone = true;
      }
    }
  }
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

// Function to print thermocouple data and errors
void printThermocoupleDataPrint(double temp1, double temp2, double desiredTemp, int Output1, int Output2, uint8_t* errorFlagsMAX, int errorFlagsMAXSize) {
  if (millis() - serialPrintStart > serialPrintInterval) {
    Serial.print(temp1); 
    Serial.print(", ");
    Serial.print(temp2); 
    Serial.print(", ");
    Serial.print(desiredTemp); 
    Serial.print(", ");
    Serial.print(Output1); 
    Serial.print(", ");
    Serial.print(Output2); 
    Serial.print(", ");

  // Print error flags    
    for (int i = 0; i < errorFlagsMAXSize; i++) {
      Serial.print(errorFlagsMAX[i]);
    }
    
    Serial.println();  

    serialPrintStart = millis();
  }
}

// ------------------------------------------------------------------------------------------------------------|
// From: https://support.newhavendisplay.com/hc/en-us/articles/5902187111191-Serial-Interface-LCD-with-Arduino |
// ------------------------------------------------------------------------------------------------------------|

/***********************************************************
 * Serial_LCD.ino
 * This code was written to interface and Arduino UNO with NHD serial LCDs.
 * 
 * Program Loop:
 * 1. Write "Newhaven Display--" on line 1
 * 2. Write " - 4x20  Characters" on line 2
 * 3. Write " - Serial LCD"
 * 4. Write "  -> I2C, SPI, RS232"
 * 
 * (c)2022 Newhaven Display International, LLC.
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 ***********************************************************/

/**
 * @brief Initialize selected IO ports for I2C.
 * 
 * @param SCL Serial clock pin assigment.
 * @param SDA Serial data pin assignment.
 * @return none
 */

void initLCD_I2C(uint8_t SCL, uint8_t SDA)
{
  _interface = I2C;

  // Store pin assigmnents globally
  _SCL = SCL;
  _SDA = SDA;

  // Set IO modes
  pinMode(SCL, OUTPUT);
  pinMode(SDA, OUTPUT);
  
  // Set starting pin states
  digitalWrite(SCL, HIGH);
  digitalWrite(SDA, HIGH);
  
  // Wait for display to power ON
  delay(STARTUP_DELAY);
  clearScreen();
  clearScreen();
}

/**
 * @brief Initialize selected IO ports for SPI
 * 
 * @param SCL Serial clock pin assignment.
 * @param SDI Serial data pin assignment.
 * @param CS Chip/Slave select pin assignment.
 * @return none
 *

void initLCD_SPI(uint8_t SCL, uint8_t SDI, uint8_t CS)
{
  _interface = SPI;

  // Store pin assignments globally
  _SCL = SCL;
  _SDI = SDI;
  _CS = CS;

  // Set IO modes
  pinMode(CS, OUTPUT);
  pinMode(SCL, OUTPUT);
  pinMode(SDI, OUTPUT);

  // Set pin states
  digitalWrite(CS, HIGH);
  digitalWrite(SCL, HIGH);

  // Wait for display to power ON
  delay(STARTUP_DELAY);
  clearScreen();
} */

/**
 * @brief Initalize selected IO ports for RS232.
 * 
 * @param TX Data transmit pin assignment.
 * @return none
 *

void initLCD_RS232(uint8_t TX)
{
  _interface = RS232;

  // Store pin assignments globally
  _TX = TX;

  // Set IO modes
  pinMode(TX, OUTPUT);
  digitalWrite(TX, HIGH);

  // Wait for display to power ON
  delay(STARTUP_DELAY);
  clearScreen();
} */

/**
 * @brief Set chip/slave select HIGH and wait for 1ms.
 * 
 * @return none
 */
void setCS()
{
  digitalWrite(_CS, HIGH);
  delay(1);
}

/**
 * @brief Clear chip/slave select and wait for 1ms.
 * 
 * @return none
 */
void clearCS()
{
  digitalWrite(_CS, LOW);
  delay(1);
}

/**
 * @brief Clear the RX pin on the RS232 bus.
 * 
 * @return none
 *
void startBit()
{
  digitalWrite(_TX, LOW);
  delayMicroseconds(RS232_DELAY);
} */

/**
 * @brief Set the RX pin on the RS232 bus.
 * 
 * @return none
 *

void stopBit()
{
  digitalWrite(_TX, HIGH);
  delayMicroseconds(RS232_DELAY);
} */

/**
 * @brief Send a start condition on the I2C bus.
 * 
 * @return none
 */
void startCondition()
{
  clearSDA();
  clearSCL();
}

/**
 * @brief Send a stop condition on the I2C bus.
 * 
 * @return none
 */
void stopCondition()
{
  setSCL();
  setSDA();
}

/**
 * @brief Set the SDA/SDI pin high on the I2C/SPI bus.
 * 
 * @return none
 */
void setSDA()
{
  digitalWrite(_SDA, HIGH);
  delayMicroseconds(I2C_DELAY);
}

/**
 * @brief Clear the SDA/SDI pin on the I2C/SPI bus.
 * 
 * @return none
 */
void clearSDA()
{
  digitalWrite(_SDA, LOW);
  delayMicroseconds(I2C_DELAY);
}

/**
 * @brief Set the SCL/SCK pin on the I2C/SPI bus.
 * 
 * @return none
 */
void setSCL()
{
  digitalWrite(_SCL, HIGH);
  if(_interface == I2C)
  {
    delayMicroseconds(I2C_DELAY);
  }
}

/**
 * @brief Clear the SCL/SCK pin on the I2C/SPI bus.
 * 
 * @return none
 */
void clearSCL()
{
  digitalWrite(_SCL, LOW);
  if(_interface == I2C)
  {
    delayMicroseconds(I2C_DELAY);
  }
}

/**
 * @brief Set the I2C bus to write mode.
 * 
 * @return none
 */
void setWriteMode()
{
  putData_I2C((SLAVE_ADDRESS << 1) | 0x00);
}

/**
 * @brief Set the I2C bus to read mode.
 * 
 * @return none
 */
void setReadMode()
{
  putData_I2C((SLAVE_ADDRESS << 1) | 0x01);
}

/**
 * @brief Check if an ACK/NACK was received on the I2C bus.
 * 
 * @return uint8_t The ACK/NACK read from the display.
 */
uint8_t getACK()
{
  pinMode(_SDA, INPUT);
  setSCL();

  uint8_t ACK = digitalRead(_SDA);

  pinMode(_SDA, OUTPUT);
  clearSCL();

  return ACK;
}

/**
 * @brief Write 1 byte of data to the display.
 * 
 * @param data Byte of data to be written.
 * @return none
 */
void write(uint8_t data)
{
  switch(_interface)
  {
    case I2C:
      startCondition();
      setWriteMode();
      putData_I2C(data);
      stopCondition();
      break;
    /*case SPI:
      clearCS();
      putData_SPI(data);
      setCS();
      break;
    case RS232:
      startBit();
      putData_RS232(data);
      stopBit();
      break;
    default:
      break; */
  }
  delayMicroseconds(150);
}

/**
 * @brief Write an array of characters to the display.
 * 
 * @param data Pointer to the array of characters.
 * @return none
 */
void writeString(unsigned char* data)
{
  // Iterate through data until null terminator is found.
  while(*data != '\0')
  {
    write(*data);
    data++; // Increment pointer.
  }
}

/**
 * @brief Clock each bit of data on the I2C bus and read ACK.
 * 
 * @param data Byte of data to be put on the I2C data bus.
 * @return none
 */
void putData_I2C(uint8_t data)
{
  for(int i = 7; i >= 0; i--)
  {
    digitalWrite(_SDA, (data >> i) & 0x01);

    setSCL();
    clearSCL();
  }

  getACK();
}

/**
 * @brief Put each bit of data on the SPI data bus.
 * This function sends MSB (D7) first and LSB (D0) last.
 * 
 * @param data Byte of data to be put on the SPI data bus.
 * @return none
 *
void putData_SPI(uint8_t data)
{
  // Write data byte MSB first -> LSB last
  for(int i = 7; i >= 0; i--)
  {
    clearSCL();

    digitalWrite(_SDI, (data >> i) & 0x01);
    
    setSCL();
  }
} */

/**
 * @brief Put each bit of data on the RS232 data bus.
 * This function sends LSB (D0) first and MSB (D7) last.
 * 
 * @param data Byte of data to be put on the RS232 data bus.
 * @return none
 *
void putData_RS232(uint8_t data)
{
  // Write data byte LSB first -> MSB last
  for(int i = 0; i <= 7; i++)
  {
    digitalWrite(_TX, (data >> i) & 0x01);
    delayMicroseconds(RS232_DELAY);
  }
} */

/**
 * @brief Send the prefix data byte (0xFE).
 * 
 * @return none
 */
void prefix()
{
  write(0xFE);
}

/**
 * @brief Turn the display ON.
 * Display is turned ON by default.
 * 
 * @return none
 */
void displayON()
{
  prefix();
  write(0x41);
}

/**
 * @brief Turn the display OFF.
 * Display is turned ON by default.
 * 
 * @return none
 */
void displayOFF()
{
  prefix();
  write(0x42);
}

/**
 * @brief Set the display cursor position via DDRAM address.
 * 
 * @param position Desired DDRAM address.
 * @return none
 *
 *  From the Data Sheet: 
 *          Column 1   Column 20
 *  Line 1    0x00        0x13
 *  Line 2    0x40        0x53
 *  Line 3    0x14        0x27
 *  Line 4    0x54        0x67
 */
void setCursor(uint8_t position)
{
  prefix();
  write(0x45);
  write(position);
}

/**
 * @brief Move the cursor to line 1, column 1.
 * 
 * @return none
 */
void home()
{
  prefix();
  write(0x46);
}

/**
 * @brief Clear the display screen.
 * 
 * @return none
 */
void clearScreen()
{
  prefix();
  write(0x51);
  delay(2);
}

/**
 * @brief Set the display's contrast.
 * 0x00 <= contrast <= 0x32
 * Default: 0x28
 * 
 * @param contrast Desired contrast setting.
 * @return none 
 */
void setContrast(uint8_t contrast)
{
  prefix();
  write(0x52);
  write(contrast);
}

/**
 * @brief Set the display's brightness.
 * 0x01 <= brightness <= 0x08
 * brightness = 0x01 | Backlight OFF
 * brightness = 0x08 | Backlight ON (100%)
 * 
 * @param brightness Desired brightness setting.
 * @return none
 */
void setBrightness(uint8_t brightness)
{
  prefix();
  write(0x53);
  write(brightness);
}

/**
 * @brief Turn the underline cursor ON.
 * 
 * @return none
 */
void underlineCursorON()
{
  prefix();
  write(0x47);
}

/**
 * @brief Turn the underline cursor OFF.
 * 
 * @return none
 */
void underlineCursorOFF()
{
  prefix();
  write(0x48);
}

/**
 * @brief Turn the blinking cursor ON.
 * 
 * @return none
 */
void blinkingCursorON()
{
  prefix();
  write(0x4B);
}

/**
 * @brief Turn the blinking cursor OFF.
 * 
 * @return none
 */
void blinkingCursorOFF()
{
  prefix();
  write(0x4C);
}

/**
 * @brief Backspace the current character where the pointer is.
 * 
 * @return none
 */
void backspace()
{
  prefix();
  write(0x4E);
}
