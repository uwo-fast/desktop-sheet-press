#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <LiquidCrystal.h>

// digital IO pins.
#define MAXDO   6
#define MAXCS   7
#define MAXCLK  8

// initialize the library by associating any needed LCD interface pin

// with the arduino pin number it is connected to

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

int setTemp = 30;

void setup() {
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.print("Temp:");
  lcd.setCursor(0,1);  
  lcd.print("Set:");


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

  if (Serial.available()){
    setTemp = Serial.parseInt();
  }

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

  lcd.setCursor(6, 0);
  lcd.print(c);
  lcd.setCursor(6,1);  
  lcd.print(setTemp);
   

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
