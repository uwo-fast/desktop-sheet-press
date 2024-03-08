#include <Wire.h>
#include <SPI.h>
#include "max6675.h"

#define MAXDO   12
//#define MAXCS1   10
#define MAXCS2   11
#define MAXCS3   8
#define MAXCS4   9
#define MAXCLK  13

#define I2C_ADR 18 //Set this however you want

typedef union //Define a float that can be broken up and sent via I2C
{
 float number;
 uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t RX_P1;
FLOATUNION_t RX_P2;
FLOATUNION_t RX_P3;
FLOATUNION_t RX_P4;

double temp1, temp2, temp3, temp4;

int CH1_T = 1;
int CH2_T = 1;
int CH3_T = 1;
int CH4_T = 1;
int setTemp = 300;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Ramprate,Target,Setpoint_O;
bool Rising; //1 Means temp rising

// initialize the Thermocouples
//MAX6675 CH1(MAXCLK, MAXCS1, MAXDO);
MAX6675 CH2(MAXCLK, MAXCS2, MAXDO);
MAX6675 CH3(MAXCLK, MAXCS3, MAXDO);
MAX6675 CH4(MAXCLK, MAXCS4, MAXDO);

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  // put your setup code here, to run once:
  Wire.begin(I2C_ADR);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  inputString.reserve(50);
  
  Serial.begin(115200);

  pinMode(10, OUTPUT);

  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  /*
  Serial.print("Initializing sensors...");
  if (!CH1.begin()) {
    Serial.println("ERROR CH1.");
    //while (1) delay(10);
  }
  if (!CH2.begin()) {
    Serial.println("ERROR CH2.");
    //while (1) delay(10);
  }
  if (!CH3.begin()) {
    Serial.println("ERROR CH3.");
    //while (1) delay(10);
  }
  if (!CH4.begin()) {
    Serial.println("ERROR CH4.");
    //while (1) delay(10);
  }
  Serial.println("DONE.");
  */
}

void loop() {

  //temp1 = CH1.readCelsius();
  temp2 = CH2.readCelsius();
  temp3 = CH3.readCelsius();
  temp4 = CH4.readCelsius(); 

  /*Serial.print(temp1);
  Serial.print(", ");
  Serial.print(temp2);
  Serial.print(", ");
  Serial.print(temp3);
  Serial.print(", ");*/
  Serial.print("Temp: ");
  Serial.print(temp4);
  Serial.print(", Setpoint: ");
  Serial.println(setTemp);
  delay(300);

  if (temp4 > setTemp)
  {
    digitalWrite(10, LOW);
    Serial.println("HEATING: OFF");    

  }
  if (temp4 <= setTemp)
  {
    digitalWrite(10, HIGH);
    Serial.println("HEATING: ON");    
  }  
}

void receiveEvent(int howMany) {
  RX_P1.number = 0; RX_P2.number = 0; RX_P3.number = 0; RX_P4.number = 0;
  int ByteCount = 0;
  while (1 <= Wire.available()) { // loop through all but the last
    if (ByteCount == 0)
    {
      RX_P1.bytes[0] = Wire.read(); // receive a byte as character
    }
    else if (ByteCount == 1)
    {
      RX_P2.bytes[0] = Wire.read(); // receive a byte as character
    }
    else if (ByteCount == 2)
    {
      RX_P3.bytes[0] = Wire.read(); // receive a byte as character
    }
    else if (ByteCount == 3)
    {
      RX_P4.bytes[ByteCount] = Wire.read(); // receive a byte as character
    }
    ByteCount++;
  }
  stringComplete = true;
  CH1_T = RX_P1.number;
  CH2_T = RX_P2.number;
  CH3_T = RX_P3.number;
  CH4_T = RX_P4.number;

}

void requestEvent() {
  //needs changing
  
  RX_P1.number = temp1;
  RX_P2.number = temp2;
  RX_P3.number = temp3;
  RX_P4.number = temp4;
  
  //Send parameters if set to be requested
  if(CH1_T){
    for (int i = 0; i <=3; i++)
    {
      Wire.write(RX_P1.bytes[i]);
    }
  }

  if(CH2_T){
    for (int i = 0; i <=3; i++)
    {
      Wire.write(RX_P2.bytes[i]);
    }
  }

  if(CH3_T){
    for (int i = 0; i <=3; i++)
    {
      Wire.write(RX_P3.bytes[i]);
    }
  }

  if(CH4_T){
    for (int i = 0; i <=3; i++)
    {
      Wire.write(RX_P4.bytes[i]);
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;

      CH1_T = inputString.substring(0).toInt();
      Serial.println("Got it!");
      CH2_T = inputString.substring(2).toInt();
      Serial.println("Got it!");
      CH3_T = inputString.substring(4).toInt();
      Serial.println("Got it!");
      CH4_T = inputString.substring(6).toInt();
      Serial.println("Got it!");
      
    }
  }
}
