#include <SPI.h>
#include <Adafruit_MAX31855.h>

// Define the connections pins for the thermocouples
#define DO1   6
#define CS1   7
#define CLK1  9

#define DO2   6
#define CS2   8
#define CLK2  9

// Initialize the Thermocouple
Adafruit_MAX31855 thermocouple1(CLK1, CS1, DO1);
Adafruit_MAX31855 thermocouple2(CLK2, CS2, DO2);

// Define the SSR control pins
#define SSR1 2
#define SSR2 3

// Initialize desired temperature in C
double desiredTemp = 50.0; 

// Initialize SSR states
bool SSR1State = LOW;
bool SSR2State = LOW;

void setup() {
  
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc
  Serial.begin(9600);
  
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



  // Initialize the 'CS' pins as output for MAX31855
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  // Initialize the SSR control pins as output
  pinMode(SSR1, OUTPUT);
  pinMode(SSR2, OUTPUT);
}

void loop() {
  // Read the temperatures
  double temp1 = thermocouple1.readCelsius();
  double temp2 = thermocouple2.readCelsius();

  // Check for new desired temperature from Serial Monitor
  if (Serial.available()) {
    String newTempString = Serial.readStringUntil('\n');
    desiredTemp = newTempString.toDouble();
  }

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

  if(temp1 < desiredTemp){
    digitalWrite(SSR1, HIGH); // Turn on heating element 1
    SSR1State = HIGH;
  }else{
    digitalWrite(SSR1, LOW); // Turn off heating element 1
    SSR1State = LOW;
  }

  if(temp2 < desiredTemp){
    digitalWrite(SSR2, HIGH); // Turn on heating element 2
    SSR2State = HIGH;
  }else{
    digitalWrite(SSR2, LOW); // Turn off heating element 2
    SSR2State = LOW;
  }
  
    // Output comma separated values
    Serial.print(temp1); 
    Serial.print(", ");
    Serial.print(temp2); 
    Serial.print(", ");
    Serial.print(desiredTemp);
    Serial.print(", ");
    Serial.print(SSR1State == HIGH ? "ON" : "OFF");
    Serial.print(", ");
    Serial.println(SSR2State == HIGH ? "ON" : "OFF");
  delay(1000); // wait for a second
}
