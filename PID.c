#include "pid.h"

void runPID(void *Parameters)
{
	float currError = 0, prevError = 0, errorSum = 0, errorDiff = 0,
			outD = 0, outP = 0, outI = 0, output = 0;
	int delayTicks;

	struct PIDParameters * params = Parameters;

	CoWaitForSingleFlag(params->InitCompleteFlagID, 0);

	while (1)
	{
		if (RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_AUTO_MODE && RegisterMap[REGISTER_CONTROL_MODE].Bits & params->ControlBitMask)
		{
			CoPendSem(params->ThisPIDSemID, 0);

			if(!params->IsTopLevelUnit)
				CoPendSem(params->PrevPIDSemID, 0);

			CoPendSem(params->DataSemID, 0);

			currError = *(params->SetPoint) - *(params->ProcessVariable);
			errorSum += currError;
			errorDiff = currError - prevError;

			prevError = currError;

			outD = *(params->KD) * errorDiff;
			outI = *(params->KI) * errorSum;
			outP = *(params->KP) * currError;

			output = outD + outP + outI;

			*(params->OutputVariable) = output;

			CoPostSem(params->ThisPIDSemID);

			if(!params->IsTopLevelUnit)
				CoPostSem(params->PrevPIDSemID);

			CoPostSem(params->DataSemID);
		}
		else
		{
			// Reset if the PID controller is turned off
			currError = 0;
			errorSum  = 0;
			errorDiff = 0;
			prevError = 0;
		}

		delayTicks = delaymsToTicks(*(params->UpdateTime_ms));

		CoTickDelay(delayTicks);
	}
}
