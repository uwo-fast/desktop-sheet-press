
// Temperature functions
int readTemps()
{

    static double lastValidTemp[2] = {0};
    static uint8_t consecutiveErrors[2] = {0};
    uint8_t maxConsecutiveErrors = 10;

    double newTemp[2] = {0};

    // Read temperatures from the thermocouples
    for (int i = 0; i < 2; i++)
    {
        newTemp[i] = thermocouples[i].readCelsius();
        pData.tempError[i] = thermocouples[i].readError();
    }

    // Process the temperature data
    for (int i = 0; i < 2; i++)
    {
        if (newTemp[i] == 0 || pData.tempError[i])
        {
            consecutiveErrors[i]++;

            if (consecutiveErrors[i] >= maxConsecutiveErrors)
            {
                // Set temperature to 0 if error limit is reached
                pData.temp[i] = 0;
            }
            else
            {
                // Replace with last valid temperature if current reading is zero or an error
                pData.temp[i] = lastValidTemp[i];
            }
        }
        else
        {
            // Update last valid temperature
            lastValidTemp[i] = newTemp[i];
            pData.temp[i] = newTemp[i];
            consecutiveErrors[i] = 0; // Reset error count on valid reading
        }
    }
}
