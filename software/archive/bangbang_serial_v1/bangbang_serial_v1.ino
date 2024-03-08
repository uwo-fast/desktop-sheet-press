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
#define SSR1 9
#define SSR2 10

// Initialize desired temperature in C
double desiredTemp = 50.0; 

// Initialize SSR states
bool SSR1State = LOW;
bool SSR2State = LOW;

void setup() {
  Serial.begin(9600);
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
    Serial.println("Failed to read from Thermocouple 1!");
  } else if (isnan(temp2)) {
    Serial.println("Failed to read from Thermocouple 2!");
  } else {
    // Control the heating elements
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
  }
  delay(1000); // wait for a second
}
