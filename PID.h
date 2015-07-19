#ifndef PID_H
#define PID_H

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "coOS.h"

#include "system.h"
#include "util.h"


struct PIDParameters
{
	float *SetPoint;
	float *KD;
	float *KI;
	float *KP;
	unsigned int *UpdateTime_ms;
	float *ProcessVariable;
	float *OutputVariable;
	unsigned int ControlBitMask;
	OS_EventID PrevPIDSemID;
	OS_EventID ThisPIDSemID;
	OS_EventID DataSemID;
	int IsTopLevelUnit;
	OS_FlagID InitCompleteFlagID;
};

// Normal PID controller.
void runPID(void *Parameters);

#endif
