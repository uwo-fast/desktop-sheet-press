#include "serial.h"
#include "control.h"
#include "_states.h"
#include <Arduino.h>

void handleSerialCommands() {
    static String received = "";
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        received += inChar;
        if (inChar == '\n') {  // when a complete command is received
            received.trim();  // remove potential leading/trailing white space
            if (received.startsWith("st=")) {
                double newSetpoint = received.substring(3).toDouble();
                for (int i = 0; i < NUM_SENSORS; i++) {
                    setpoint[i] = newSetpoint;
                }
            } else if (received.startsWith("kp=")) {
                double newKp = received.substring(3).toDouble();
                for (int i = 0; i < NUM_SENSORS; i++) {
                    pidControllers[i]->SetTunings(newKp, pidControllers[i]->GetKi(), pidControllers[i]->GetKd());
                }
            } else if (received.startsWith("ki=")) {
                double newKi = received.substring(3).toDouble();
                for (int i = 0; i < NUM_SENSORS; i++) {
                    pidControllers[i]->SetTunings(pidControllers[i]->GetKp(), newKi, pidControllers[i]->GetKd());
                }
            } else if (received.startsWith("kd=")) {
                double newKd = received.substring(3).toDouble();
                for (int i = 0; i < NUM_SENSORS; i++) {
                    pidControllers[i]->SetTunings(pidControllers[i]->GetKp(), pidControllers[i]->GetKi(), newKd);
                }
            } else if (received == "preheat") {
                stateCommand = "preheat";
            } else if (received == "heat") {
                stateCommand = "heat";
            } else if (received == "cool") {
                stateCommand = "cool";
            } else if (received == "standby") {
                stateCommand = "standby";
            } else if (received == "error") {
                stateCommand = "error";
            } else if (received == "settings") {
                stateCommand = "settings";
            }
            received = "";  // clear received data
        }
    }
}

void printData(const TempData& tempData, const ControlData& controlData, const char* stateName, const Timing& timing) {
    Serial.print("State:");
    Serial.print(stateName);
    Serial.print(" T1:");
    Serial.print(tempData.temperatures[0], 0);
    Serial.print(", T2:");
    Serial.print(tempData.temperatures[1], 0);
    Serial.print(", ST:");
    Serial.print(setpoint[0], 0);  // Assuming both setpoints are the same
    Serial.print(", O1:");
    Serial.print(controlData.outputs[0], 0);
    Serial.print(", O2:");
    Serial.print(controlData.outputs[1], 0);
    Serial.print(", Kp:");
    Serial.print(pidControllers[0]->GetKp(), 2);
    Serial.print(", Ki:");
    Serial.print(pidControllers[0]->GetKi(), 2);
    Serial.print(", Kd:");
    Serial.print(pidControllers[0]->GetKd(), 2);
    Serial.print(", Elapsed Time:");
    Serial.print(timing.elapsedTime / 1000);  // Print elapsed time in seconds
    Serial.print("s, Remaining Heating Time:");
    Serial.print(timing.durationRemaining / 1000);  // Print remaining heating time in seconds
    Serial.println("s");
}

// Function to get the full state name
const char* getStateName(State* state) {
    if (state == standbyState) {
        return "Standby";
    } else if (state == preheatingState) {
        return "Preheating";
    } else if (state == heatingState) {
        return "Heating";
    } else if (state == coolingState) {
        return "Cooling";
    } else if (state == errorState) {
        return "Error";
    } else {
        return "Unknown";
    }
}
