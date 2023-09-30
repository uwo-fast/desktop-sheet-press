#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "ArduPID.h"
#include <LiquidCrystal.h>

// digital IO pins.
#define MAXDO   6
#define MAXCS   7
#define MAXCLK  8

// initialize the controller
ArduPID myController;

double setTemp = 300;
double setTempReq;
double input;
double output;
double p = 30;
double i = 5;
double d = 0;
double maxTemp = 310;
double lowerBound;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

//variable to control serial output format to include labels
bool exportFormat = true;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void setup() {
  
  Serial.begin(9600);

  TCCR1B = TCCR1B & B11111000 | B00000101; // for PWM frequency of 30.64 Hz

  myController.begin(&input, &output, &setTemp, p, i, d);
    
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.print("Temp:");
  lcd.setCursor(0,1);  
  lcd.print("Set:");


  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
 
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

  if(Serial.available()){
    setTemp = Serial.parseInt();
  }

  if(setTemp>maxTemp){
    setTemp = maxTemp;
    Serial.print("Maximum temperature exceeded, set to maximum temperature: ");
    Serial.print(setTemp);
  }

lowerBound = setTemp / maxTemp * 255;
myController.setOutputLimits(lowerBound, 255);


   input = thermocouple.readCelsius();
   if (isnan(input)) {
     Serial.println("Something wrong with thermocouple!");
   }

  myController.compute();
  myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);

/*
if(exportFormat = true){
  Serial.print(setTemp);
  Serial.print("\t");
  Serial.print(input);
  Serial.print("\t");
  Serial.print(output);
  Serial.println("\t");
  } else { 
  Serial.print("Set temp:");
  Serial.print(setTemp);
  Serial.print("\t");  
  Serial.print("Input: ");
  Serial.print(input);
  Serial.print("\t");
  Serial.print("Output: ");
  Serial.println(output);
  Serial.print("\t");
  }*/

  lcd.setCursor(6, 0);
  lcd.print(input);
  lcd.setCursor(6,1);  
  lcd.print(setTemp);
   
  analogWrite(9, output);
  analogWrite(10, output);

/*
  if (input > setTemp)
  {
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    Serial.println(" | HEATING: OFF");    

  }
  if (input <= setTemp)
  {
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
    Serial.println(" | HEATING: ON");    
  }  */

  

  
   delay(1000);
}
