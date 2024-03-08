#include <SPI.h>
#include "Adafruit_MAX31855.h"

// digital IO pins.
#define MAXDO   6
#define MAXCS   7
#define MAXCLK  8

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

int setTemp = 30;

void setup() {
  
  // set up the LCD's number of columns and rows:

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
 
    Serial.begin(9600);

  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  Serial.println("DONE.");
}

void loop() {

  // basic readout test, just print the current temp
   //Serial.print("Internal Temp = ");
   //Serial.println(thermocouple.readInternal());

   double c = thermocouple.readCelsius();
   if (isnan(c)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("Plate temp = ");
     Serial.print(c);
   }
   //Serial.print("F = ");
   //Serial.println(thermocouple.readFahrenheit());

  Serial.print(" | Set temp = ");
  Serial.print(setTemp);
   
  if (c > setTemp)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    Serial.println(" | HEATING: OFF");    

  }
  if (c <= setTemp)
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
    Serial.println(" | HEATING: ON");    
  }  
   delay(1000);
}
