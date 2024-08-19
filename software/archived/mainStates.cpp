#include "mainStates.h"

#ifdef SERIALCMD
#include "serial.h"
#endif

#ifdef SDCARD
#include "sdcard.h"
#endif

#include "config.h"
#include "progdata.h"

#include "timing.h"
#include "control.h"
#include "temp.h"
#include "relay.h"
#include "eepromutil.h"
#include "monitoring.h"

// #include "gui.h"
#include "dial.h"

ProgramData pData; // ProgramData struct, found in progdata.h

StateMachine machine = StateMachine();

State *standbyState = machine.addState(&standby, "standby");
State *preheatingState = machine.addState(&preheating, "preheating");
State *heatingState = machine.addState(&heating, "heating");
State *errorState = machine.addState(&error, "error");

Timing timing = {0};
TempData tempData;
ControlData controlData;

MachineState stateCommand = STANDBY;

void initializeSystem()
{

    loadEeprom();
    initializePIDs(pData);
    initTCs();

    initializeTransitions();

    // initializeLcdGui();
}

void initializeTransitions()
{
    // Add transitions
    standbyState->addTransition(&toPreheating, preheatingState);
    standbyState->addTransition(&toError, errorState);

    preheatingState->addTransition(&toHeating, heatingState);
    preheatingState->addTransition(&toError, errorState);

    heatingState->addTransition(&toError, errorState);

    errorState->addTransition(&toStandby, standbyState);

    // Add transition to standby from any state on button hold
    preheatingState->addTransition(&toStandby, standbyState);
    heatingState->addTransition(&toStandby, standbyState);
    errorState->addTransition(&toStandby, standbyState);
}

void mainProgram()
{
    State *currentState = machine.getCurrentState();

#ifdef SERIALCMD
    handleSerialCommands();
#endif

    updateTiming(currentState);
    processEncoderEvents();
    updateEeprom();

    if (millis() - timing.lut.control >= CONTROL_INTERVAL)
    {
        timing.lut.control = millis();
        tempData = readTemps();
        tempData = processTempData(tempData);
        checkThermalRunaway(tempData);
        controlData = controlLogic(tempData, currentState);
        writeRelays(controlData);
    }
    if (millis() - timing.lut.gui >= GUI_INTERVAL)
    {
        timing.lut.gui = millis();
        // updateLcdGui();
    }
    if (millis() - timing.lut.log >= LOG_INTERVAL)
    {
        timing.lut.log = millis();
#ifdef SERIALCMD
        printData(tempData, controlData, timing, machine.getStateName());
#endif
        if (currentState == preheatingState || currentState == heatingState)
        {
#ifdef SDCARD
            // logData(tempData, controlData, timing, machine.getStateName());
#endif
        }
    }
}

// State functions
void standby()
{
    mainProgram();
}

void preheating()
{
    if (machine.executeOnce)
    {
        timing.pit.preStart = millis();
#ifdef SDCARD
/*         if (createNewLogFile())
        {
            Serial.println(F("New log file created"));
        }
        else
        {
            Serial.println(F("Failed to create new log file"));
        } */
#endif
    }

    mainProgram();
}

void heating()
{
    if (machine.executeOnce)
    {
        timing.pit.heatStart = millis();
    }

    mainProgram();
}

void error()
{
    mainProgram();
}

// Transition functions
bool toPreheating()
{
    return stateCommand == PREHEATING;
}

bool toHeating()
{
    return stateCommand == HEATING;
}

bool toStandby()
{
    return stateCommand == STANDBY;
}

bool toError()
{
    return stateCommand == ERROR;
}