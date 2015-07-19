#include "stm32f0xx.h"
#include "ahrsInterface.h"
#include "pid.h"
#include "coOS.h"
#include "system.h"

#define DEFAULT_STACK_SIZE 64

OS_STK InitStack[DEFAULT_STACK_SIZE];

OS_STK SumTaskStack[DEFAULT_STACK_SIZE];

OS_STK RollRatePIDStack[DEFAULT_STACK_SIZE];
OS_STK PitchRatePIDStack[DEFAULT_STACK_SIZE];
OS_STK YawRatePIDStack[DEFAULT_STACK_SIZE];

OS_STK RollPIDStack[DEFAULT_STACK_SIZE];
OS_STK PitchPIDStack[DEFAULT_STACK_SIZE];

OS_STK XVelPIDStack[DEFAULT_STACK_SIZE];
OS_STK YVelPIDStack[DEFAULT_STACK_SIZE];
OS_STK ZVelPIDStack[DEFAULT_STACK_SIZE];

OS_STK XPosPIDStack[DEFAULT_STACK_SIZE];
OS_STK YPosPIDStack[DEFAULT_STACK_SIZE];
OS_STK ZPosPIDStack[DEFAULT_STACK_SIZE];
OS_STK YawPosPIDStack[DEFAULT_STACK_SIZE];

OS_STK AHRSPollStack[4*DEFAULT_STACK_SIZE];
OS_STK UltrasonicsStack[DEFAULT_STACK_SIZE];

static OS_EventID SemPitchRate;
static OS_EventID SemRollRate;
static OS_EventID SemYawRate;
static OS_EventID SemPitch;
static OS_EventID SemRoll;
static OS_EventID SemXVel;
static OS_EventID SemYVel;
static OS_EventID SemZVel;
static OS_EventID SemXPos;
static OS_EventID SemYPos;
static OS_EventID SemZPos;
static OS_EventID SemYawPos;
static OS_EventID SemData;

static OS_FlagID InitCompleteFlag;
static OS_FlagID TestMotorsFlag;

static struct PollingParameters PollingParams;

static struct InitTaskParameters InitTaskParams;

static struct SummingTaskParameters SumTaskParams;

static struct PIDParameters RollRatePIDParameters,	PitchRatePIDParameters,	YawRatePIDParameters,
							RollPIDParameters,		PitchPIDParameters,
							XVelPIDParameters,		YVelPIDParameters,		ZVelPIDParameters,
							XPosPIDParameters,		YPosPIDParameters,		ZPosPIDParameters,		YawPosPIDParameters;

// Main handles basic set up of the OS, then the initialisation task takes over to complete system set up.
int main(void)
{
	OS_TID initTaskID;
	CoInitOS();

	SemPitchRate 		= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemRollRate 		= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemYawRate 			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemPitch 			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemRoll		 		= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemXVel 			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemYVel 			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemZVel 			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemXPos 			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemYPos 			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemZPos 			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemYawPos			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);
	SemData 			= CoCreateSem(1,1,EVENT_SORT_TYPE_PRIO);

	InitCompleteFlag	= CoCreateFlag(0,0);
	TestMotorsFlag		= CoCreateFlag(0,0);

	initTaskID = CoCreateTask(initialiseSystemTask,	&InitTaskParams, 			1, &InitStack[DEFAULT_STACK_SIZE], 	DEFAULT_STACK_SIZE);

	CoCreateTask(runAHRSPolling, 		&PollingParams,				1,	&AHRSPollStack[DEFAULT_STACK_SIZE], 	DEFAULT_STACK_SIZE);

	CoCreateTask(runUltrasonicPolling,	&PollingParams,				1,	&UltrasonicsStack[DEFAULT_STACK_SIZE], 	DEFAULT_STACK_SIZE);

	CoCreateTask(runSummingBlocks, 		&SumTaskParams,				1,	&SumTaskStack[DEFAULT_STACK_SIZE],		DEFAULT_STACK_SIZE);

	CoCreateTask(runPID, 				&RollRatePIDParameters, 	2, 	&RollRatePIDStack[DEFAULT_STACK_SIZE], 	DEFAULT_STACK_SIZE);
	CoCreateTask(runPID, 				&PitchRatePIDParameters,	2, 	&PitchRatePIDStack[DEFAULT_STACK_SIZE], DEFAULT_STACK_SIZE);
	CoCreateTask(runPID, 				&YawRatePIDParameters, 		2, 	&YawRatePIDStack[DEFAULT_STACK_SIZE], 	DEFAULT_STACK_SIZE);

	CoCreateTask(runPID, 				&RollPIDParameters, 		3, 	&RollPIDStack[DEFAULT_STACK_SIZE], 		DEFAULT_STACK_SIZE);
	CoCreateTask(runPID, 				&RollPIDParameters, 		3, 	&PitchPIDStack[DEFAULT_STACK_SIZE],		DEFAULT_STACK_SIZE);

	CoCreateTask(runPID,		 		&XVelPIDParameters, 		4, 	&XVelPIDStack[DEFAULT_STACK_SIZE], 		DEFAULT_STACK_SIZE);
	CoCreateTask(runPID,				&YVelPIDParameters, 		4, 	&YVelPIDStack[DEFAULT_STACK_SIZE], 		DEFAULT_STACK_SIZE);
	CoCreateTask(runPID, 				&ZVelPIDParameters, 		4, 	&ZVelPIDStack[DEFAULT_STACK_SIZE], 		DEFAULT_STACK_SIZE);

	CoCreateTask(runPID, 				&XPosPIDParameters, 		5, 	&XPosPIDStack[DEFAULT_STACK_SIZE], 		DEFAULT_STACK_SIZE);
	CoCreateTask(runPID,		 		&YPosPIDParameters, 		5, 	&YPosPIDStack[DEFAULT_STACK_SIZE], 		DEFAULT_STACK_SIZE);
	CoCreateTask(runPID,				&ZPosPIDParameters, 		5, 	&ZPosPIDStack[DEFAULT_STACK_SIZE], 		DEFAULT_STACK_SIZE);
	CoCreateTask(runPID, 				&YawPosPIDParameters, 		5, 	&YawPosPIDStack[DEFAULT_STACK_SIZE],	DEFAULT_STACK_SIZE);

	RegisterMap[REGISTER_CONTROL_MODE].Bits &= ~CONTROL_MODE_AUTO_MODE;

	InitTaskParams.InitCompleteFlagID 	= InitCompleteFlag;
	InitTaskParams.TestMotorsFlagID		= TestMotorsFlag;

	SumTaskParams.SumBlockParams[0].MotorNumber			= MOTOR_1;
	SumTaskParams.SumBlockParams[0].PitchMultiplier		=  -1;
	SumTaskParams.SumBlockParams[0].RollMultiplier		=   0;
	SumTaskParams.SumBlockParams[0].YawMultiplier		=  -1;
	SumTaskParams.SumBlockParams[0].ThrustMultiplier	=   1;

	SumTaskParams.SumBlockParams[1].MotorNumber 		= MOTOR_2;
	SumTaskParams.SumBlockParams[1].PitchMultiplier		=   1;
	SumTaskParams.SumBlockParams[1].RollMultiplier		=   0;
	SumTaskParams.SumBlockParams[1].YawMultiplier		=  -1;
	SumTaskParams.SumBlockParams[1].ThrustMultiplier	=   1;

	SumTaskParams.SumBlockParams[2].MotorNumber 		= MOTOR_3;
	SumTaskParams.SumBlockParams[2].PitchMultiplier		=   0;
	SumTaskParams.SumBlockParams[2].RollMultiplier		=   1;
	SumTaskParams.SumBlockParams[2].YawMultiplier		=   1;
	SumTaskParams.SumBlockParams[2].ThrustMultiplier	=   1;

	SumTaskParams.SumBlockParams[3].MotorNumber 		= MOTOR_4;
	SumTaskParams.SumBlockParams[3].PitchMultiplier		=   0;
	SumTaskParams.SumBlockParams[3].RollMultiplier		=  -1;
	SumTaskParams.SumBlockParams[3].YawMultiplier		=   1;
	SumTaskParams.SumBlockParams[3].ThrustMultiplier	=   1;

	SumTaskParams.PitchRateSemID	= SemPitchRate;
	SumTaskParams.RollRateSemID		= SemRollRate;
	SumTaskParams.YawRateSemID		= SemYawRate;
	SumTaskParams.DataSemID			= SemData;

	SumTaskParams.TestMotorsFlagID	= TestMotorsFlag;

	SumTaskParams.UpdatePeriodms	= 100;


	RollRatePIDParameters.ControlBitMask		= CONTROL_MODE_RUN_ROLL_RATE_PID;
	RollRatePIDParameters.KD					= &RegisterMap[REGISTER_ROLL_RATE_PID_KD].Data;
	RollRatePIDParameters.KI					= &RegisterMap[REGISTER_ROLL_RATE_PID_KI].Data;
	RollRatePIDParameters.KP					= &RegisterMap[REGISTER_ROLL_RATE_PID_KP].Data;
	RollRatePIDParameters.OutputVariable		= RollRateOut;
	RollRatePIDParameters.ProcessVariable		= RollRate;
	RollRatePIDParameters.SetPoint				= &RegisterMap[REGISTER_ROLL_RATE_PID_SET_POINT].Data;
	RollRatePIDParameters.UpdateTime_ms			= &RegisterMap[REGISTER_ROLL_RATE_PID_UPDATE_TIME].Bits;
	RollRatePIDParameters.ThisPIDSemID			= SemRollRate;
	RollRatePIDParameters.PrevPIDSemID			= SemRoll;
	RollRatePIDParameters.DataSemID				= SemData;
	RollRatePIDParameters.IsTopLevelUnit		= FALSE;
	RollRatePIDParameters.InitCompleteFlagID	= InitCompleteFlag;

	PitchRatePIDParameters.ControlBitMask		= CONTROL_MODE_RUN_PITCH_RATE_PID;
	PitchRatePIDParameters.KD					= &RegisterMap[REGISTER_PITCH_RATE_PID_KD].Data;
	PitchRatePIDParameters.KI					= &RegisterMap[REGISTER_PITCH_RATE_PID_KI].Data;
	PitchRatePIDParameters.KP					= &RegisterMap[REGISTER_PITCH_RATE_PID_KP].Data;
	PitchRatePIDParameters.OutputVariable		= PitchRateOut;
	PitchRatePIDParameters.ProcessVariable		= PitchRate;
	PitchRatePIDParameters.SetPoint				= &RegisterMap[REGISTER_PITCH_RATE_PID_SET_POINT].Data;
	PitchRatePIDParameters.UpdateTime_ms		= &RegisterMap[REGISTER_PITCH_RATE_PID_UPDATE_TIME].Bits;
	PitchRatePIDParameters.ThisPIDSemID			= SemPitchRate;
	PitchRatePIDParameters.PrevPIDSemID			= SemPitch;
	PitchRatePIDParameters.DataSemID			= SemData;
	PitchRatePIDParameters.IsTopLevelUnit		= FALSE;
	PitchRatePIDParameters.InitCompleteFlagID	= InitCompleteFlag;

	YawRatePIDParameters.ControlBitMask			= CONTROL_MODE_RUN_YAW_RATE_PID;
	YawRatePIDParameters.KD						= &RegisterMap[REGISTER_YAW_RATE_PID_KD].Data;
	YawRatePIDParameters.KI						= &RegisterMap[REGISTER_YAW_RATE_PID_KI].Data;
	YawRatePIDParameters.KP						= &RegisterMap[REGISTER_YAW_RATE_PID_KP].Data;
	YawRatePIDParameters.OutputVariable			= YawRateOut;
	YawRatePIDParameters.ProcessVariable		= YawRate;
	YawRatePIDParameters.SetPoint				= &RegisterMap[REGISTER_YAW_RATE_PID_SET_POINT].Data;
	YawRatePIDParameters.UpdateTime_ms			= &RegisterMap[REGISTER_YAW_RATE_PID_UPDATE_TIME].Bits;
	YawRatePIDParameters.ThisPIDSemID			= SemYawRate;
	YawRatePIDParameters.PrevPIDSemID			= SemYawPos;
	YawRatePIDParameters.DataSemID				= SemData;
	YawRatePIDParameters.IsTopLevelUnit			= FALSE;
	YawRatePIDParameters.InitCompleteFlagID		= InitCompleteFlag;



	RollPIDParameters.ControlBitMask			= CONTROL_MODE_RUN_ROLL_PID;
	RollPIDParameters.KD						= &RegisterMap[REGISTER_ROLL_PID_KD].Data;
	RollPIDParameters.KI						= &RegisterMap[REGISTER_ROLL_PID_KI].Data;
	RollPIDParameters.KP						= &RegisterMap[REGISTER_ROLL_PID_KP].Data;
	RollPIDParameters.OutputVariable			= &RegisterMap[REGISTER_ROLL_RATE_PID_SET_POINT].Data;
	RollPIDParameters.ProcessVariable			= Roll;
	RollPIDParameters.SetPoint					= &RegisterMap[REGISTER_ROLL_PID_SET_POINT].Data;
	RollPIDParameters.UpdateTime_ms				= &RegisterMap[REGISTER_ROLL_PID_UPDATE_TIME].Bits;
	RollPIDParameters.ThisPIDSemID				= SemRoll;
	RollPIDParameters.PrevPIDSemID				= SemYVel;
	RollPIDParameters.DataSemID					= SemData;
	RollPIDParameters.IsTopLevelUnit			= FALSE;
	RollPIDParameters.InitCompleteFlagID		= InitCompleteFlag;

	PitchPIDParameters.ControlBitMask			= CONTROL_MODE_RUN_PITCH_PID;
	PitchPIDParameters.KD						= &RegisterMap[REGISTER_PITCH_PID_KD].Data;
	PitchPIDParameters.KI						= &RegisterMap[REGISTER_PITCH_PID_KI].Data;
	PitchPIDParameters.KP						= &RegisterMap[REGISTER_PITCH_PID_KP].Data;
	PitchPIDParameters.OutputVariable			= &RegisterMap[REGISTER_PITCH_RATE_PID_SET_POINT].Data;
	PitchPIDParameters.ProcessVariable			= Pitch;
	PitchPIDParameters.SetPoint					= &RegisterMap[REGISTER_PITCH_PID_SET_POINT].Data;
	PitchPIDParameters.UpdateTime_ms			= &RegisterMap[REGISTER_PITCH_PID_UPDATE_TIME].Bits;
	PitchPIDParameters.ThisPIDSemID				= SemPitch;
	PitchPIDParameters.PrevPIDSemID				= SemXVel;
	PitchPIDParameters.DataSemID				= SemData;
	PitchPIDParameters.IsTopLevelUnit			= FALSE;
	PitchPIDParameters.InitCompleteFlagID		= InitCompleteFlag;



	XVelPIDParameters.ControlBitMask			= CONTROL_MODE_RUN_X_VEL_PID;
	XVelPIDParameters.KD						= &RegisterMap[REGISTER_X_VEL_PID_KD].Data;
	XVelPIDParameters.KI						= &RegisterMap[REGISTER_X_VEL_PID_KI].Data;
	XVelPIDParameters.KP						= &RegisterMap[REGISTER_X_VEL_PID_KP].Data;
	XVelPIDParameters.OutputVariable			= &RegisterMap[REGISTER_PITCH_PID_SET_POINT].Data;
	XVelPIDParameters.ProcessVariable			= XVel;
	XVelPIDParameters.SetPoint					= &RegisterMap[REGISTER_X_VEL_PID_SET_POINT].Data;
	XVelPIDParameters.UpdateTime_ms				= &RegisterMap[REGISTER_X_VEL_PID_UPDATE_TIME].Bits;
	XVelPIDParameters.ThisPIDSemID				= SemXVel;
	XVelPIDParameters.PrevPIDSemID				= SemXPos;
	XVelPIDParameters.DataSemID					= SemData;
	XVelPIDParameters.IsTopLevelUnit			= FALSE;
	XVelPIDParameters.InitCompleteFlagID		= InitCompleteFlag;

	YVelPIDParameters.ControlBitMask			= CONTROL_MODE_RUN_Y_VEL_PID;
	YVelPIDParameters.KD						= &RegisterMap[REGISTER_Y_VEL_PID_KD].Data;
	YVelPIDParameters.KI						= &RegisterMap[REGISTER_Y_VEL_PID_KI].Data;
	YVelPIDParameters.KP						= &RegisterMap[REGISTER_Y_VEL_PID_KP].Data;
	YVelPIDParameters.OutputVariable			= &RegisterMap[REGISTER_ROLL_PID_SET_POINT].Data;
	YVelPIDParameters.ProcessVariable			= YVel;
	YVelPIDParameters.SetPoint					= &RegisterMap[REGISTER_Y_VEL_PID_SET_POINT].Data;
	YVelPIDParameters.UpdateTime_ms				= &RegisterMap[REGISTER_Y_VEL_PID_UPDATE_TIME].Bits;
	YVelPIDParameters.ThisPIDSemID				= SemYVel;
	YVelPIDParameters.PrevPIDSemID				= SemYPos;
	YVelPIDParameters.DataSemID					= SemData;
	YVelPIDParameters.IsTopLevelUnit			= FALSE;
	YVelPIDParameters.InitCompleteFlagID		= InitCompleteFlag;

	ZVelPIDParameters.ControlBitMask			= CONTROL_MODE_RUN_Z_VEL_PID;
	ZVelPIDParameters.KD						= &RegisterMap[REGISTER_Z_VEL_PID_KD].Data;
	ZVelPIDParameters.KI						= &RegisterMap[REGISTER_Z_VEL_PID_KI].Data;
	ZVelPIDParameters.KP						= &RegisterMap[REGISTER_Z_VEL_PID_KP].Data;
	ZVelPIDParameters.OutputVariable			= ThrustOut;
	ZVelPIDParameters.ProcessVariable			= ZVel;
	ZVelPIDParameters.SetPoint					= &RegisterMap[REGISTER_Z_VEL_PID_SET_POINT].Data;
	ZVelPIDParameters.UpdateTime_ms				= &RegisterMap[REGISTER_Z_VEL_PID_UPDATE_TIME].Bits;
	ZVelPIDParameters.ThisPIDSemID				= SemZVel;
	ZVelPIDParameters.PrevPIDSemID				= SemZPos;
	ZVelPIDParameters.DataSemID					= SemData;
	ZVelPIDParameters.IsTopLevelUnit			= FALSE;
	ZVelPIDParameters.InitCompleteFlagID		= InitCompleteFlag;



	XPosPIDParameters.ControlBitMask			= CONTROL_MODE_RUN_X_POS_PID;
	XPosPIDParameters.KD						= &RegisterMap[REGISTER_X_POS_PID_KD].Data;
	XPosPIDParameters.KI						= &RegisterMap[REGISTER_X_POS_PID_KI].Data;
	XPosPIDParameters.KP						= &RegisterMap[REGISTER_X_POS_PID_KP].Data;
	XPosPIDParameters.OutputVariable			= &RegisterMap[REGISTER_X_VEL_PID_SET_POINT].Data;
	XPosPIDParameters.ProcessVariable			= XPos;
	XPosPIDParameters.SetPoint					= &RegisterMap[REGISTER_X_POS_PID_SET_POINT].Data;
	XPosPIDParameters.UpdateTime_ms				= &RegisterMap[REGISTER_X_POS_PID_UPDATE_TIME].Bits;
	XPosPIDParameters.ThisPIDSemID				= SemXPos;
	XPosPIDParameters.DataSemID					= SemData;
	XPosPIDParameters.IsTopLevelUnit			= TRUE;
	XPosPIDParameters.InitCompleteFlagID		= InitCompleteFlag;

	YPosPIDParameters.ControlBitMask			= CONTROL_MODE_RUN_Y_POS_PID;
	YPosPIDParameters.KD						= &RegisterMap[REGISTER_Y_POS_PID_KD].Data;
	YPosPIDParameters.KI						= &RegisterMap[REGISTER_Y_POS_PID_KI].Data;
	YPosPIDParameters.KP						= &RegisterMap[REGISTER_Y_POS_PID_KP].Data;
	YPosPIDParameters.OutputVariable			= &RegisterMap[REGISTER_Y_VEL_PID_SET_POINT].Data;
	YPosPIDParameters.ProcessVariable			= YPos;
	YPosPIDParameters.SetPoint					= &RegisterMap[REGISTER_Y_POS_PID_SET_POINT].Data;
	YPosPIDParameters.UpdateTime_ms				= &RegisterMap[REGISTER_Y_POS_PID_UPDATE_TIME].Bits;
	YPosPIDParameters.ThisPIDSemID				= SemYPos;
	YPosPIDParameters.DataSemID					= SemData;
	YPosPIDParameters.IsTopLevelUnit			= TRUE;
	YPosPIDParameters.InitCompleteFlagID		= InitCompleteFlag;

	ZPosPIDParameters.ControlBitMask			= CONTROL_MODE_RUN_Z_POS_PID;
	ZPosPIDParameters.KD						= &RegisterMap[REGISTER_Z_POS_PID_KD].Data;
	ZPosPIDParameters.KI						= &RegisterMap[REGISTER_Z_POS_PID_KI].Data;
	ZPosPIDParameters.KP						= &RegisterMap[REGISTER_Z_POS_PID_KP].Data;
	ZPosPIDParameters.OutputVariable			= &RegisterMap[REGISTER_Z_VEL_PID_SET_POINT].Data;
	ZPosPIDParameters.ProcessVariable			= ZPos;
	ZPosPIDParameters.SetPoint					= &RegisterMap[REGISTER_Z_POS_PID_SET_POINT].Data;
	ZPosPIDParameters.UpdateTime_ms				= &RegisterMap[REGISTER_Z_POS_PID_UPDATE_TIME].Bits;
	ZPosPIDParameters.ThisPIDSemID				= SemZPos;
	ZPosPIDParameters.DataSemID					= SemData;
	ZPosPIDParameters.IsTopLevelUnit			= TRUE;
	ZPosPIDParameters.InitCompleteFlagID		= InitCompleteFlag;

	YawPosPIDParameters.ControlBitMask			= CONTROL_MODE_RUN_YAW_POS_PID;
	YawPosPIDParameters.KD						= &RegisterMap[REGISTER_YAW_POS_PID_KD].Data;
	YawPosPIDParameters.KI						= &RegisterMap[REGISTER_YAW_POS_PID_KI].Data;
	YawPosPIDParameters.KP						= &RegisterMap[REGISTER_YAW_POS_PID_KP].Data;
	YawPosPIDParameters.OutputVariable			= &RegisterMap[REGISTER_YAW_RATE_PID_SET_POINT].Data;
	YawPosPIDParameters.ProcessVariable			= Yaw;
	YawPosPIDParameters.SetPoint				= &RegisterMap[REGISTER_YAW_POS_PID_SET_POINT].Data;
	YawPosPIDParameters.UpdateTime_ms			= &RegisterMap[REGISTER_YAW_POS_PID_UPDATE_TIME].Bits;
	YawPosPIDParameters.ThisPIDSemID			= SemYawPos;
	YawPosPIDParameters.DataSemID				= SemData;
	YawPosPIDParameters.IsTopLevelUnit			= TRUE;
	YawPosPIDParameters.InitCompleteFlagID		= InitCompleteFlag;

 	CoStartOS();

    while(1);
}
