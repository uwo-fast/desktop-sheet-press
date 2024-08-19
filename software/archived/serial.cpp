#include "serial.h"
#include <Arduino.h>
#include "main.h"
#include "eepromutil.h"

void handleSerialCommands()
{
    static char received[32] = "";
    static byte idx = 0;

    while (Serial.available() > 0)
    {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || idx >= sizeof(received) - 1)
        {                         // when a complete command is received or buffer is full
            received[idx] = '\0'; // null-terminate the string

            if (strncmp(received, "st=", 3) == 0)
            {
                float newSetpoint = atof(received + 3);
                for (int i = 0; i < NUM_SENSORS; i++)
                {
                    pData.setpoint[i] = newSetpoint;
                }
            }
            else if (strncmp(received, "kp=", 3) == 0)
            {
                float newKp = atof(received + 3);
                for (int i = 0; i < NUM_SENSORS; i++)
                {
                    pData.Kp[i] = newKp;
                    pidControllers[i]->SetTunings(pData.Kp[i], pData.Ki[i], pData.Kd[i]);
                }
            }
            else if (strncmp(received, "ki=", 3) == 0)
            {
                float newKi = atof(received + 3);
                for (int i = 0; i < NUM_SENSORS; i++)
                {
                    pData.Ki[i] = newKi;
                    pidControllers[i]->SetTunings(pData.Kp[i], pData.Ki[i], pData.Kd[i]);
                }
            }
            else if (strncmp(received, "kd=", 3) == 0)
            {
                float newKd = atof(received + 3);
                for (int i = 0; i < NUM_SENSORS; i++)
                {
                    pData.Kd[i] = newKd;
                    pidControllers[i]->SetTunings(pData.Kp[i], pData.Ki[i], pData.Kd[i]);
                }
            }
            else if (strcmp(received, "prep") == 0)
            {
                stateCommand = PREPARING;
            }
            else if (strcmp(received, "active") == 0)
            {
                stateCommand = ACTIVE;
            }
            else if (strcmp(received, "standby") == 0)
            {
                stateCommand = STANDBY;
            }
            else if (strcmp(received, "eeprom=reset") == 0)
            {
                resetEeprom(EE_PARTIAL_RESET);
            }
            else if (strcmp(received, "eeprom=fullreset") == 0)
            {
                resetEeprom(EE_FULL_RESET);
            }

            idx = 0; // clear index
        }
        else
        {
            received[idx++] = inChar;
        }
    }
}

void printData(const TempData &tempData, const ControlData &controlData, const Timing &timing, const char *stateName)
{
    Serial.print(F("State:"));
    Serial.print(stateName);
    Serial.print(F(" T1:"));
    Serial.print(tempData.temperatures[0], 2);
    Serial.print(F(", T2:"));
    Serial.print(tempData.temperatures[1], 2);
    Serial.print(F(", ST:"));
    Serial.print(pData.setpoint[0], 0); // Assuming both setpoints are the same
    Serial.print(F(", O1:"));
    Serial.print(controlData.outputs[0], 0);
    Serial.print(F(", O2:"));
    Serial.print(controlData.outputs[1], 0);
    Serial.print(F(", Kp:"));
    Serial.print(pData.Kp[0], 2);
    Serial.print(F(", Ki:"));
    Serial.print(pData.Ki[0], 2);
    Serial.print(F(", Kd:"));
    Serial.print(pData.Kd[0], 2);
    Serial.print(F(", Elapsed Time:"));
    Serial.print(timing.ct.elapsed / 1000); // Print elapsed time in seconds
    Serial.print(F("s, Remaining Heating Time:"));
    Serial.print(timing.ct.durationRemaining / 1000); // Print remaining active time in seconds
    Serial.println(F("s"));
}
