// PID Process Parameters
double Setpoint, Input1, Output1, Input2, Output2;


// PID object setup for both thermocouples
// PID myPID1(&Input1, &Output1, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);
PID myPID1(&Input1, &Output1, &Setpoint, DEF_KP / MILLI_UNIT, DEF_KI / MILLI_UNIT, DEF_KD / MILLI_UNIT, DIRECT); /**< PID1 object gets input from thermocouple 1 and ouputs to relay 1*/

PID myPID2(&Input2, &Output2, &Setpoint, DEF_KP / MILLI_UNIT, DEF_KI / MILLI_UNIT, DEF_KD / MILLI_UNIT, DIRECT); /**< PID2 object gets input from thermocouple 2 and ouputs to relay 2*/


// Optional function to dynamically adjust PID tuning parameters based on gap to setpoint
void dynamicTuning()
{
	double gap1 = abs(Setpoint - Input1);
	if (gap1 < pData.gapThreshold)
	{ // Less aggressive tuning parameters for small gap
		myPID1.SetTunings(pData.kp / pData.cp / MILLI_UNIT, pData.ki / pData.ci / MILLI_UNIT, pData.kd / pData.cd / MILLI_UNIT);
	}
	else
	{ // More aggressive tuning parameters for large gap
		myPID1.SetTunings(pData.kp / MILLI_UNIT, pData.ki / MILLI_UNIT, pData.kd / MILLI_UNIT);
	}

	double gap2 = abs(Setpoint - Input2);
	if (gap2 < pData.gapThreshold)
	{ // Less aggressive tuning parameters for small gap
		myPID2.SetTunings(pData.kp / pData.cp / MILLI_UNIT, pData.ki / pData.ci / MILLI_UNIT, pData.kd / pData.cd / MILLI_UNIT);
	}
	else
	{ // More aggressive tuning parameters for large gap
		myPID2.SetTunings(pData.kp / MILLI_UNIT, pData.ki / MILLI_UNIT, pData.kd / MILLI_UNIT);
	}
}
