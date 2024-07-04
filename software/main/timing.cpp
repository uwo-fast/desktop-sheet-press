
/*-------------------------------Check Sleep for Standby----------------------------------*/
/**
 *  \brief    Issues a standby timeout event if the standby time has expired without any activity.
 *  \remarks  The standby timeout period is defined in ms in the file header.
 */
void checkSleep()
{
	// make sure it doesnt go to sleep if the process is active
	if (currentProcessState.getState() == ACTIVE_PROCESS)
	{
		lastActiveTime = millis();
	}
	// The last active time is updated every time some activity occurs. If the standby timeout
	// period has expired without any activity then a timeout event is issued.
	if (lastActiveTime + STANDBY_TIME_OUT < millis())
		uEvent = EV_STBY_TIMEOUT;
}

