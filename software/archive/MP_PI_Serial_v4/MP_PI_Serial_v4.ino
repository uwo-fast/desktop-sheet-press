#include <PID_v1.h>
#include <avr/io.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define DO   3
#define CS1  5
#define CS2  6
#define CLK  4

// Initialize the thermocouples
Adafruit_MAX31855 thermocouple1(CLK, CS1, DO);
Adafruit_MAX31855 thermocouple2(CLK, CS2, DO);

uint8_t errorFlagsMAX[] = {0, 0, 0, 0, 0, 0};

// Define the SSR control pins
#define SSR1 14 //A0
#define SSR2 15 //A1

const unsigned long period = 10*1000; // 10 seconds
double dutyCycle1 = 50*0.01;  // 50%
double dutyCycle2 = 50*0.01;  // 50%
unsigned long cycleStart1 = 0; // Set cycle to start at time t=0
unsigned long cycleStart2 = 0; // Set cycle to start at time t=0

// Initialize desired temperature in C
double desiredTemp = 150.0; 

// Initialize the measured temperature variables
double temp1 = 0.0;
double temp2 = 0.0;

// Define variables to store the last valid temperature readings
double lastValidTemp1 = 0.0;
double lastValidTemp2 = 0.0;

//Define Variables we'll be connecting to
double Setpoint, Input1, Output1, Input2, Output2;

//Define the aggressive and conservative Tuning Parameters
double Cp=3, Ci=3, Cd=3;  // Constant multipliers for: agg --> cons
double aggKp=5, aggKi=1, aggKd=0;  // Aggressive parameters, base
double consKp=aggKp/Cp, consKi=aggKi/Ci, consKd=aggKd/Cd;  // Conservative parameters

// Gap threshold
double gapThres = 5;

// Specify the links and initial tuning parameters
PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT); 
PID myPID2(&Input2, &Output2, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);

// SETUP -------------------------------------------
void setup() {

  while (!Serial) delay(1);
  Serial.begin(115200);
  
  Serial.println("MAX31855 test 1");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.println("Initializing sensor...");
  if (!thermocouple1.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }

  Serial.println("MAX31855 test 2");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.println("Initializing sensor...");
  if (!thermocouple2.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }

  // OPTIONAL: Can configure fault checks as desired (default is ALL)
  // Multiple checks can be logically OR'd together.
  // thermocouple1.setFaultChecks(MAX31855_FAULT_OPEN | MAX31855_FAULT_SHORT_VCC);  // short to GND fault is ignored
  // thermocouple2.setFaultChecks(MAX31855_FAULT_OPEN | MAX31855_FAULT_SHORT_VCC);  // short to GND fault is ignored
  // Initialize the 'CS' pins as output for MAX31855
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  // Initialize the SSR control pins as output
  pinMode(SSR1, OUTPUT);
  pinMode(SSR2, OUTPUT);

  Setpoint = desiredTemp;

  //turn the PID on
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);

  //Set the sample time to match the period
  myPID1.SetSampleTime(period);
  myPID2.SetSampleTime(period);

  myPID1.SetOutputLimits(0, 255);
  myPID2.SetOutputLimits(0, 255);
}


// MAIN LOOP -------------------------------------------
void loop() {

  // Check for new desired temperature from Serial Monitor
  if (Serial.available()) {
    String newTempString = Serial.readStringUntil('\n');
    desiredTemp = newTempString.toDouble();
    Input1 = desiredTemp; // Update Input1 with the new desired temperature
    Input2 = desiredTemp; // Update Input2 with the new desired temperature
  }

  // Read the temperatures
  temp1 = thermocouple1.readCelsius();
  temp2 = thermocouple2.readCelsius();

  // Error handling for thermocouples
  checkIsnan(temp1, temp2);

  // Update the inputs with the current temperature reading
  Input1 = temp1;
  Input2 = temp2;

  // Update dynamic tuning
  dynamicTuning();

  // Invoke the PID computation
  myPID1.Compute();
  myPID2.Compute();

  // Debug line for TCs
  // Serial.println("TC1: " + String(temp1, 2) + ", TC2: " + String(temp2, 2) + ".");

  // Update/monitor PWM and update PID if the cycle is finished
  slowPWM(SSR1, cycleStart1, period, dutyCycle1);
  slowPWM(SSR2, cycleStart2, period, dutyCycle2);
  
  printThermocoupleDataPrint(temp1, temp2, desiredTemp, Output1, Output2, errorFlagsMAX, sizeof(errorFlagsMAX)/sizeof(errorFlagsMAX[0]));
  
  delay(20);  // Put in a slight delay to help debounce the reading of the rotary encoder
}

// HELPERS -------------------------------------------

// Function for PWM generation
// Function for PWM generation
void slowPWM(int SSRn, unsigned long& cycleStart, double period, double dutyCycle) {
  unsigned long currentMillis = millis();
  
  // Calculate the duty cycle based on the PID output
  double calculatedDutyCycle = Output1 / 255.0;
  
  if (currentMillis - cycleStart < period * calculatedDutyCycle) {
    digitalWrite(SSRn, HIGH);
  } 
  else if (currentMillis - cycleStart < period) {
    digitalWrite(SSRn, LOW);
  } 
  else {
    cycleStart = currentMillis;
  }
}

// Error handling function to check if the MAX board gave 'nan' and if so the error
// Must pass the temps as reference for that they are updated
void checkIsnan(double& temp1, double& temp2) {

  uint8_t e1 = thermocouple1.readError();
  errorFlagsMAX[0] = (e1 & MAX31855_FAULT_OPEN) ? 1 : 0;
  errorFlagsMAX[1] = (e1 & MAX31855_FAULT_SHORT_GND) ? 1 : 0;
  errorFlagsMAX[2] = (e1 & MAX31855_FAULT_SHORT_VCC) ? 1 : 0;

  uint8_t e2 = thermocouple2.readError();
  errorFlagsMAX[3] = (e2 & MAX31855_FAULT_OPEN) ? 1 : 0;
  errorFlagsMAX[4] = (e2 & MAX31855_FAULT_SHORT_GND) ? 1 : 0;
  errorFlagsMAX[5] = (e2 & MAX31855_FAULT_SHORT_VCC) ? 1 : 0;

  // Set temp1 to the last valid measurement if it's NaN
  temp1 = isnan(temp1) ? lastValidTemp1 : temp1;

  // Set temp2 to the last valid measurement if it's NaN
  temp2 = isnan(temp2) ? lastValidTemp2 : temp2;

  // Update the last valid temperature measurements
  lastValidTemp1 = temp1;
  lastValidTemp2 = temp2;

}


void dynamicTuning() {

  double gap1 = abs(Setpoint-Input1); //distance away from setpoint
  if (gap1 < gapThres)
  { 
    //we're close to setpoint, use conservative tuning parameters
    myPID1.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID1.SetTunings(aggKp, aggKi, aggKd);
  }

  double gap2 = abs(Setpoint-Input2); //distance away from setpoint
  if (gap2 < gapThres)
  { 
    //we're close to setpoint, use conservative tuning parameters
    myPID2.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID2.SetTunings(aggKp, aggKi, aggKd);
  }
}

void printThermocoupleDataPrint(double temp1, double temp2, double desiredTemp, int Output1, int Output2, uint8_t* errorFlagsMAX, int errorFlagsMAXSize) {
  // Output comma separated values
  
  static int i = 0;  // i will keep its value between function calls

  if (i <= 50) {
    i++;
    return; // this will exit the function
  } 
  else {
    i = 0;
  }

  Serial.print(temp1); // This is the temperature measured from thermocouple 1 during the last cycle (unless error)
  Serial.print(", ");

  Serial.print(temp2); // This is the temperature measured from thermocouple 2 during the last cycle (unless error)
  Serial.print(", ");
  
  Serial.print(desiredTemp); // This is the current set temperature (plant input)
  Serial.print(", ");
  
  Serial.print(Output1); // This is the output control signal value for SSR1 (PWM 0-255, plant output)
  Serial.print(", ");
  
  Serial.print(Output2); // This is the output control signal value for SSR2 (PWM 0-255, plant output)
  Serial.print(", ");
  
  for (int i = 0; i < errorFlagsMAXSize; i++) {
    Serial.print(errorFlagsMAX[i]);
  }
  
  Serial.println();  
}
