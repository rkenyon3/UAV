#ifndef SYSTEM_H
#define SYSTEM_H

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_i2c.h"
#include "coOS.h"

#include "PID.h"
#include "ahrsInterface.h"
#include "ultrasonicsInterface.h"
#include "util.h"

#define FALSE 0
#define TRUE !FALSE

#define MOTOR_MIN_PULSE_LENGTH 600
#define MOTOR_MAX_PULSE_LENGTH 2400
#define MOTOR_TEST_PULSE_LENGTH 800

#define NUMBER_OF_MOTORS 4
#define MOTOR_1 0
#define MOTOR_2 1
#define MOTOR_3 2
#define MOTOR_4 3

struct InitTaskParameters
{
	OS_FlagID InitCompleteFlagID;
	OS_FlagID TestMotorsFlagID;
};

struct SumBlockParameters
{
	int ThrustMultiplier;
	int PitchMultiplier;
	int RollMultiplier;
	int YawMultiplier;
	int MotorNumber;
};

struct SummingTaskParameters
{
	struct SumBlockParameters SumBlockParams[NUMBER_OF_MOTORS];
	OS_EventID PitchRateSemID;
	OS_EventID RollRateSemID;
	OS_EventID YawRateSemID;
	OS_EventID DataSemID;
	int UpdatePeriodms;
	OS_FlagID TestMotorsFlagID;
};
// Outputs of low level PID controllers
extern float * ThrustOut,
			 * PitchRateOut,
			 * RollRateOut,
			 * YawRateOut;

// Initialises the system. Sets up all peripherals.
void initialiseSystemTask(void * Parameters);
int setUpUART(USART_TypeDef* USARTx, int BaudRate);

/* Runs all of the summing blocks as a single function.
 * Doing so reduces the number of tasks required. The
 * minimum refresh time used is the lowest of all the
 * times provided.
 */
void runSummingBlocks(void *Parameters);

/* Delay functions. Blocking calls. Not advised for use with operating system.
 * Microsecond delays should be accurate to within 1 us.
 */
void delayMicroSeconds(uint16_t NumberOfMicroseconds);
void delayMilliSeconds(uint16_t NumberOfMilliseconds);
void delaySeconds(uint16_t NumberOfSeconds);

// Light blue LED on PC8
void setBlueLED(void);
// Light green LED on PC9
void setGreenLED(void);
// Turn off blue LED on PC8
void clearBlueLED(void);
// Turn off green LED on PC9
void clearGreenLED(void);

/* Flashes the green LED to indicate an error. Blocking call.
 * Acts as a way to preserve the system state, but allows no
 * further actions.
 */
void showErrorCondition(uint8_t errorCode);
// Shows solid green to indicate operation.
void showOperationalCondition(void);

/* Lights the blue LED for the specified number of milliseconds.
 * On and off time before returning are both equal to Time
 */
void flashBlue(int16_t Time, uint32_t NumberOfFlashes);
void toggleBlue();

#endif
