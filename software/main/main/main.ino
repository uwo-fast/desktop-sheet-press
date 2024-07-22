#include <Arduino.h>
#include "_states.h"
#include "serial.h"
#include "dataproc.h"
#include "control.h"
#include "relay.h"
#include "lcdgui.h"
#include "encodergui.h"
#include <ClickEncoder.h>
#include <TimerOne.h>

void setup() {
    Serial.begin(115200);
    initializeSystem();
    initializeTransitions();  
    initializeLcdGui();
    initializeEncoder();  
}

void loop() {
    handleSerialCommands();
    machine.run();
}
