#ifdef SERIALCMD
/**
 * @brief Handles incoming serial commands.
 *
 * This function reads characters from the serial input, builds a command string,
 * and executes the corresponding actions when a full command is received. Supported
 * commands include setting a new setpoint, adjusting the duration, changing the
 * machine state, and resetting EEPROM data.
 */
void handleSerialCommands()
{
    static char received[32] = "";
    static byte idx = 0;

    while (Serial.available() > 0)
    {
        char inChar = (char)Serial.read();
        // when a complete command is received or buffer is full
        if (inChar == '\n' || idx >= sizeof(received) - 1)
        {
            received[idx] = '\0'; // null-terminate the string

            if (strncmp(received, "st=", 3) == 0)
            {
                double newSetpoint = atof(received + 3);
                for (int i = 0; i < 2; i++)
                {
                    pData.setSetpoint(i, newSetpoint);
                }
            }
            else if (strncmp(received, "dt=", 3) == 0)
            {
                unsigned long newSetDuration = atof(received + 3);
                setDurationSetter(newSetDuration);
            }
            else if (strncmp(received, "dt+", 3) == 0)
            {
                unsigned long durationDelta = atof(received + 3);
                setDurationSetter(pData.getSetDuration() / MINUTE + durationDelta);
            }
            else if (strncmp(received, "dt-", 3) == 0)
            {
                unsigned long durationDelta = atof(received + 3);
                setDurationSetter(pData.getSetDuration() / MINUTE - durationDelta);
            }
            else if (strncmp(received, "rt=", 3) == 0)
            {
                unsigned long newRemainingDuration = atof(received + 3);
                remainingDurationSetter(newRemainingDuration);
            }
            else if (strncmp(received, "kp=", 3) == 0)
            {
                double newKp = atof(received + 3);
                for (int i = 0; i < 2; i++)
                {
                    pData.setKp(i, newKp);
                    pidControllers[i].SetTunings(newKp, pData.getKi(i), pData.getKd(i));
                }
            }
            else if (strncmp(received, "ki=", 3) == 0)
            {
                double newKi = atof(received + 3);
                for (int i = 0; i < 2; i++)
                {
                    pData.setKi(i, newKi);
                    pidControllers[i].SetTunings(pData.getKp(i), newKi, pData.getKd(i));
                }
            }
            else if (strncmp(received, "kd=", 3) == 0)
            {
                double newKd = atof(received + 3);
                for (int i = 0; i < 2; i++)
                {
                    pData.setKd(i, newKd);
                    pidControllers[i].SetTunings(pData.getKp(i), pData.getKi(i), newKd);
                }
            }
            else if ((strncmp(received, "o1=", 3) == 0) && (currMState == SUDO))
            {
                double newOutput = atof(received + 3);
                pData.output[0] = newOutput;
            }
            else if ((strncmp(received, "o2=", 3) == 0) && (currMState == SUDO))
            {
                double newOutput = atof(received + 3);
                pData.output[1] = newOutput;
            }
            else if (strcmp(received, "prep") == 0)
            {
                SET_PREPARING();
            }
            else if (strcmp(received, "active") == 0)
            {
                SET_ACTIVE();
            }
            else if (strcmp(received, "term") == 0)
            {
                SET_TERMINATING();
            }
            else if (strcmp(received, "standby") == 0)
            {
                SET_STANDBY();
            }
            else if (strcmp(received, "sudo") == 0)
            {
                SET_SUDO();
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

/**
 * @brief Prints the current system state and relevant data to the serial monitor.
 *
 * This function outputs the current state, temperatures, setpoint, control outputs,
 * elapsed time, remaining time, and PID parameters to the serial monitor.
 *
 */
void printData()
{
    Serial.print(F("State: "));
    Serial.print(getMachineStateName(currMState));
    Serial.print(F(", T1:"));
    Serial.print(pData.temp[0], 2);
    Serial.print(F(", T2:"));
    Serial.print(pData.temp[1], 2);
    Serial.print(F(", ST:"));
    Serial.print(pData.getSetpoint(0), 0);
    if (pData.getTraStatus() == -1)
    {
        Serial.print(F("*"));
    }
    else if (pData.traStatus == -2)
    {
        Serial.print(F("**"));
    }
    Serial.print(F(", O1:"));
    Serial.print(pData.output[0]);
    Serial.print(F(", O2:"));
    Serial.print(pData.output[1]);
    Serial.print(F(", Elapsed:"));
    Serial.print((double)timing.ct.elapsed / MINUTE, 2);
    Serial.print(F("m, Remaining:"));
    Serial.print((double)timing.ct.durationRemaining / MINUTE, 2);
    Serial.print(F("/"));
    Serial.print((double)pData.getSetDuration() / MINUTE, 2);
    Serial.print(F("m"));
    Serial.print(F(", Kp:"));
    Serial.print(pData.getKp(0), 3);
    Serial.print(F(", Ki:"));
    Serial.print(pData.getKi(0), 3);
    Serial.print(F(", Kd:"));
    Serial.print(pData.getKd(0), 3);
    Serial.println(".");
}
#endif // SERIALCMD
