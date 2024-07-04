
// Function to implement slow PWM for SSR control
void slowPWM(int SSRn, unsigned long &cycleStart, double period, double output)
{
	// Get current time in milliseconds
	unsigned long currentMillis = millis();

	double dutyCycle = output / 255;

	// If within the ON part of the cycle, turn the SSR on
	if (currentMillis - cycleStart < period * dutyCycle)
	{
		digitalWrite(SSRn, HIGH);
	}
	// If within the OFF part of the cycle, turn the SSR off
	else if (currentMillis - cycleStart >= period * dutyCycle && currentMillis - cycleStart < period)
	{
		digitalWrite(SSRn, LOW);
	}
	// If the cycle is complete, reset the cycle start time
	else
	{
		cycleStart = currentMillis;
	}
}
