#include "_states.h"
#include "serial.h"
#include "dataproc.h"
#include "control.h"
#include "temp.h"
#include "relay.h"
#include "lcdgui.h"
#include "encodergui.h"

StateMachine machine = StateMachine();

State* standbyState = machine.addState(&standby);
State* preheatingState = machine.addState(&preheating);
State* heatingState = machine.addState(&heating);
State* coolingState = machine.addState(&cooling);
State* errorState = machine.addState(&error);
State* settingsState = machine.addState(&settings);

Timing timing = {0, 0, DEF_HEATING_DURATION, 0, DEF_HEATING_DURATION, 0, 0}; 

String stateCommand = "";

void initializeSystem() {
    initializePIDs();
    pinMode(PIN_SSR1, OUTPUT);
    pinMode(PIN_SSR2, OUTPUT);
}

void initializeTransitions() {
    // Add transitions
    standbyState->addTransition(&toPreheating, preheatingState);
    standbyState->addTransition(&toSettings, settingsState);
    standbyState->addTransition(&toError, errorState);

    settingsState->addTransition(&toStandby, standbyState);
    
    preheatingState->addTransition(&toHeating, heatingState);
    preheatingState->addTransition(&toError, errorState);
    
    heatingState->addTransition(&toCooling, coolingState);
    heatingState->addTransition(&toError, errorState);
    
    coolingState->addTransition(&toStandby, standbyState);
    coolingState->addTransition(&toError, errorState);
    
    errorState->addTransition(&toStandby, standbyState);

    // Add transition to standby from any state on button hold
    preheatingState->addTransition(&toStandby, standbyState);
    heatingState->addTransition(&toStandby, standbyState);
    coolingState->addTransition(&toStandby, standbyState);
    settingsState->addTransition(&toStandby, standbyState);
    errorState->addTransition(&toStandby, standbyState);
}

MachineState getCurrentState(State* currentState) {
    if (currentState == standbyState) {
        return STANDBY;
    } else if (currentState == preheatingState) {
        return PREHEATING;
    } else if (currentState == heatingState) {
        return HEATING;
    } else if (currentState == coolingState) {
        return COOLING;
    } else if (currentState == errorState) {
        return ERROR;
    } else if (currentState == settingsState) {
        return SETTINGS;
    }
    return ERROR; // Default fallback state
}

void updateTiming(MachineState currentState) {
    unsigned long currentMillis = millis();
    if (currentState == PREHEATING || currentState == HEATING || currentState == COOLING) {
        timing.elapsedTime = currentMillis - timing.preStartTime;
    }
    if (currentState == HEATING) {
        if (timing.lastUpdateTime > 0) {
            unsigned long elapsedSinceLastUpdate = currentMillis - timing.lastUpdateTime;
            timing.durationRemaining = (timing.durationRemaining > elapsedSinceLastUpdate) ? (timing.durationRemaining - elapsedSinceLastUpdate) : 0;
        }
        timing.lastUpdateTime = currentMillis;
    }
}

// State functions
void standby() {
    if (machine.executeOnce) {
        Serial.println("State: Standby");
    }
    encoderEvent(standbyState);  // Handle encoder events
    if (millis() - timing.lastHighFreqUpdate >= HIGH_FREQ_INTERVAL) {
        timing.lastHighFreqUpdate = millis();
        TempData tempData = readTemps();
        tempData = processTempData(tempData);
        ControlData controlData = controlLogic(tempData);
        handleUserEvents(standbyState); // Handle user events
        printData(tempData, controlData, getStateName(standbyState), timing);
        writeRelays(controlData);
        updateLcdGui(tempData, controlData, getStateSymbol(standbyState), STANDBY);
    }
}

void preheating() {
    if (machine.executeOnce) {
        Serial.println("State: Preheating");
        timing.preStartTime = millis();
    }
    updateTiming(getCurrentState(preheatingState));
    encoderEvent(preheatingState);  // Handle encoder events
    if (millis() - timing.lastHighFreqUpdate >= HIGH_FREQ_INTERVAL) {
        timing.lastHighFreqUpdate = millis();
        TempData tempData = readTemps();
        tempData = processTempData(tempData);
        ControlData controlData = controlLogic(tempData);
        handleUserEvents(preheatingState); // Handle user events
        printData(tempData, controlData, getStateName(preheatingState), timing);
        writeRelays(controlData);
        updateLcdGui(tempData, controlData, getStateSymbol(preheatingState), getCurrentState(preheatingState));
    }
}

void heating() {
    if (machine.executeOnce) {
        Serial.println("State: Heating");
        timing.heatStartTime = millis();
    } 
    updateTiming(getCurrentState(heatingState));
    if (timing.durationRemaining <= 0) {
        stateCommand = "cool";
    }
    encoderEvent(heatingState);  // Handle encoder events
    if (millis() - timing.lastHighFreqUpdate >= HIGH_FREQ_INTERVAL) {
        timing.lastHighFreqUpdate = millis();
        TempData tempData = readTemps();
        tempData = processTempData(tempData);
        ControlData controlData = controlLogic(tempData);
        handleUserEvents(heatingState); // Handle user events
        printData(tempData, controlData, getStateName(heatingState), timing);
        writeRelays(controlData);
        updateLcdGui(tempData, controlData, getStateSymbol(heatingState), getCurrentState(heatingState));
    }
}

void cooling() {
    if (machine.executeOnce) {
        Serial.println("State: Cooling");
    }
    updateTiming(getCurrentState(coolingState));
    encoderEvent(coolingState);  // Handle encoder events
    if (millis() - timing.lastHighFreqUpdate >= HIGH_FREQ_INTERVAL) {
        timing.lastHighFreqUpdate = millis();
        TempData tempData = readTemps();
        tempData = processTempData(tempData);
        ControlData controlData = controlLogic(tempData);
        handleUserEvents(coolingState); // Handle user events
        printData(tempData, controlData, getStateName(coolingState), timing);
        writeRelays(controlData);
        updateLcdGui(tempData, controlData, getStateSymbol(coolingState), getCurrentState(coolingState));
    }
}

void error() {
    if (machine.executeOnce) {
        Serial.println("State: Error");
    }
    encoderEvent(errorState);  // Handle encoder events
    if (millis() - timing.lastHighFreqUpdate >= HIGH_FREQ_INTERVAL) {
        timing.lastHighFreqUpdate = millis();
        TempData tempData = readTemps();
        tempData = processTempData(tempData);
        ControlData controlData = controlLogic(tempData);
        handleUserEvents(errorState); // Handle user events
        printData(tempData, controlData, getStateName(errorState), timing);
        writeRelays(controlData);
        updateLcdGui(tempData, controlData, getStateSymbol(errorState), getCurrentState(errorState));
    }
}

void settings() {
    if (machine.executeOnce) {
        Serial.println("State: Settings");
    }
    encoderEvent(settingsState);  // Handle encoder events
    if (millis() - timing.lastHighFreqUpdate >= HIGH_FREQ_INTERVAL) {
        timing.lastHighFreqUpdate = millis();
        TempData tempData = readTemps();
        tempData = processTempData(tempData);
        ControlData controlData = controlLogic(tempData);
        handleUserEvents(settingsState); // Handle user events
        printData(tempData, controlData, getStateName(settingsState), timing);
        updateLcdGui(tempData, controlData, getStateSymbol(settingsState), getCurrentState(settingsState)); // Update the settings GUI
    }
}

// Transition functions
bool toPreheating() {
    return stateCommand.equals("preheat");
}

bool toHeating() {
    return stateCommand.equals("heat");
}

bool toCooling() {
    return stateCommand.equals("cool");
}

bool toStandby() {
    return stateCommand.equals("standby");
}

bool toError() {
    return stateCommand.equals("error");
}

bool toSettings() {
    return stateCommand.equals("settings");
}
