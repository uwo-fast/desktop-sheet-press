#include <SPI.h>
#include <avr/io.h>
#include <PID_v1.h>
#include <Adafruit_MAX31855.h>

// Define the connections pins for the thermocouples
#define DO1   5
#define CS1   7
#define CLK1  6

#define DO2   5
#define CS2   8
#define CLK2  6

// Initialize the Thermocouple
Adafruit_MAX31855 thermocouple1(CLK1, CS1, DO1);
Adafruit_MAX31855 thermocouple2(CLK2, CS2, DO2);

// Define the SSR control pins
#define SSR1 9
#define SSR2 10

// Initialize desired temperature in C
double desiredTemp = 150.0; 

double temp1Prev = 0;
double temp2Prev = 0;

//Define Variables we'll be connecting to
double Setpoint, Input1, Output1, Input2, Output2;

//Define the aggressive and conservative Tuning Parameters
double aggKp=30, aggKi=1, aggKd=0;
double consKp=aggKp/2, consKi=aggKi/2, consKd=aggKd/2;

// Gap threshold
double gapThres = 5;

//Specify the links and initial tuning parameters
PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint, consKp, consKi, consKd, DIRECT);

void setup() {
  
  while (!Serial) delay(1);
  Serial.begin(9600);
  
  // Pins D9 and D10 - 7.5 Hz 10bit
  TCCR1A = 0b00000011; // 10bit
  TCCR1B = 0b00000101; // x1024 phase correct
  // Pins D9 and D10 - 15 Hz 10bit
  //TCCR1A = 0b00000011; // 10bit
  //TCCR1B = 0b00001101; // x1024 fast pwm
  
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
  thermocouple1.setFaultChecks(MAX31855_FAULT_OPEN | MAX31855_FAULT_SHORT_VCC);  // short to GND fault is ignored
  thermocouple2.setFaultChecks(MAX31855_FAULT_OPEN | MAX31855_FAULT_SHORT_VCC);  // short to GND fault is ignored

  Serial.println("DONE.");
  
  //turn the PID controllers on
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);

  // Initialize the 'CS' pins as output for MAX31855
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  // Initialize the SSR control pins as output
  pinMode(SSR1, OUTPUT);
  pinMode(SSR2, OUTPUT);
}

void loop() {

  // Check for new desired temperature from Serial Monitor
  if (Serial.available()) {
    String newTempString = Serial.readStringUntil('\n');
    desiredTemp = newTempString.toDouble();
  }

  // Read the temperatures
  double temp1 = thermocouple1.readCelsius();
  double temp2 = thermocouple2.readCelsius();


  if (isnan(temp1)) {
    uint8_t e = thermocouple1.readError();
    if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple 1 is open - no connections.");
    if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple 1 is short-circuited to GND.");
    if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple 1 is short-circuited to VCC.");
  }

  if (isnan(temp2)) {
    uint8_t e = thermocouple2.readError();
    if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple 2 is open - no connections.");
    if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple 2 is short-circuited to GND.");
    if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple 2 is short-circuited to VCC.");
  }

  // PI Controller
  double gap1 = abs(desiredTemp-temp1); //distance away from setpoint
  if (gap1 < gapThres)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID1.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID1.SetTunings(aggKp, aggKi, aggKd);
  }

  double gap2 = abs(desiredTemp-temp2); //distance away from setpoint
  if (gap2 < gapThres)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID2.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID2.SetTunings(aggKp, aggKi, aggKd);
  }

  Setpoint = desiredTemp;
  Input1 = temp1;
  Input2 = temp2;

  if(!isnan(temp1)) myPID1.Compute();
  if(!isnan(temp2)) myPID2.Compute();

  analogWrite(SSR1, Output1);
  analogWrite(SSR2, Output2);
  
  // Output comma separated values
  Serial.print(temp1); 
  Serial.print(", ");
  Serial.print(temp2); 
  Serial.print(", ");
  Serial.print(desiredTemp);
  Serial.print(", ");
  Serial.print(Output1); 
  Serial.print(", ");
  Serial.print(Output2); 
  Serial.println(", ");

  //temp1Prev = temp1;
  //temp2Prev = temp2;

  delay(1000); // wait for a second
}