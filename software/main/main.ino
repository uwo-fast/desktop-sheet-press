#include "mainStates.h"
#include "dial.h"
#include "sdcard.h"

void setup()
{
    Serial.begin(115200); // Initialize serial communication

    encoder = new ClickEncoder(PIN_ENC_DT, PIN_ENC_CLK, PIN_ENC_SW, 4); // Enable pullup resistors
    Timer1.initialize(1000);                                            // 2ms or 2000us
    Timer1.attachInterrupt(timerIsr);
    encLastPos = encNewPos = 0;
    pinMode(PIN_SSR1, OUTPUT);
    pinMode(PIN_SSR2, OUTPUT);

    if (!SD.begin(SD_CS))
    {
        Serial.println(F("SD Card failed to initialize"));
    }
    else
    {
        Serial.println(F("SD Card initialized"));
    }

    initializeSystem(); // Initialize the system, found in mainStates
}

void loop()
{
    machine.run(); // Run the state machine, function of StateMachine.h
}
