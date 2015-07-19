#ifndef AHRSINTERFACE_H
#define AHRSINTERFACE_H

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "coOS.h"
#include "math.h"

#include "system.h"
#include "control.h"
#include "util.h"

// define comms parameters
#define EULER_BROADCAST_FREQUENCY 				0
#define QUATERNION_BROADCAST_FREQUENCY 			0
#define ACCEL_BROADCAST_FREQUENCY 				0
#define AHRS_COMMS_DEFAULT_BAUDRATE				115200
#define AHRS_COMMS_BAUDRATE						115200

// define data polled
#define POLL_EULER_DATA							TRUE
#define POLL_QUATERNION_DATA					FALSE
#define POLL_ACCEL_DATA							TRUE
#define POLL_PERIOD_MS							100

// defines for PT byte masks
#define UM7_PT_HAS_DATA 						0x80
#define UM7_PT_IS_BATCH 						0x40
#define UM7_PT_BATCH_LENGTH_BITS 				0x3C
#define UM7_PT_BATCH_LENGTH_BIT_3				0x20
#define UM7_PT_BATCH_LENGTH_BIT_2				0x10
#define UM7_PT_BATCH_LENGTH_BIT_1				0x08
#define UM7_PT_BATCH_LENGTH_BIT_0				0x04
#define UM7_PT_HIDDEN							0x02
#define UM7_PT_COMMAND_FAILED					0x01

#define UM7_ACCEL_PROC_X_REGISTER_ADDRESS		0x65
#define UM7_ACCEL_PROC_Y_REGISTER_ADDRESS		0x66
#define UM7_ACCEL_PROC_Z_REGISTER_ADDRESS		0x67
#define UM7_ACCEL_PROC_TIME_REGISTER_ADDRESS	0x68
#define UM7_QUAT_AB_REGISTER_ADDRESS			0x6D
#define UM7_QUAT_CD_REGISTER_ADDRESS			0x6E
#define UM7_QUAT_TIME_REGISTER_ADDRESS			0x6F
#define UM7_PITCH_ROLL_REGISTER_ADDRESS			0x70
#define UM7_YAW_REGISTER_ADDRESS 				0x71
#define UM7_PITCH_ROLL_RATE_REGISTER_ADDRESS	0x72
#define UM7_YAW_RATE_REGISTER_ADDRESS			0x73
#define UM7_EULER_TIME_REGISTER_ADDRESS			0x74
#define UM7_GET_FIRMWARE_VERSION_CMD_ADDRESS	0xAA


/* Configures the UM7 sensor. Assumes that only processed Euler, quaternion and acceleration data
 * will be required. To stop one of these transmitting, set its broadcastRate to 0.
 *
 * Returns 0 if successful, 1 if invalid BaudRate, 2 if failed due to other transmission in progress,
 * or 3 if failed due to no response from sensor. */
int AHRSSetUpSensor(uint32_t BaudRate, uint8_t EulerBroadcastRate, uint8_t QuaternionBroadcastRate, uint8_t AccelBroadcastRate);

/* Sends a command to read data from the specified address.
 * Data from response will be written to the appropriate
 * variables, and may be read after this function returns 0.
 *
 * Returns 0 if successful, 1 if sensor not initialised, 2
 * if failed due to other transmission in progress, or 3 if
 * failed due to no response. */
int AHRSReadRegister(uint8_t Address, uint8_t BatchLength);

/* Polls the AHRS for all available data. Response is picked
 * up on an interrupt and used to update the variables declared
 * below.
 *
 * Returns 0 if successful, 1 if sensor not initialised, 2
 * if failed due to other transmission in progress, or 3 if
 * failed due to no response. */
int AHRSPoll(void);

// ----- Variables -----
/* Note that these values are estimates
 * and may be unreliable. */
extern float * const PitchRate,
			 * const RollRate,
			 * const YawRate,
		     * const Pitch,
			 * const Roll,
			 * const Yaw,
			 * const XVel,
			 * const YVel,
			 * const ZVel,
			 * const XPos,
			 * const YPos,
			 * const ZPos,
			 * const XPosGlobal,
			 * const YPosGlobal,
			 * const ZPosGlobal,
			 * const QuaternionA,
			 * const QuaternionB,
			 * const QuaternionC,
			 * const QuaternionD;
extern char FirmwareVersion[5];
extern int EulerUpdated, QuatUpdated, OdometryUpdated;

#endif
