#include "system.h"

// Outputs of low level PID controllers
float * ThrustOut 		= &RegisterMap[REGISTER_THRUST_OUTPUT].Data,
      * PitchRateOut 	= &RegisterMap[REGISTER_PITCH_RATE_OUTPUT].Data,
      * RollRateOut 	= &RegisterMap[REGISTER_ROLL_RATE_OUTPUT].Data,
      * YawRateOut 		= &RegisterMap[REGISTER_YAW_RATE_OUTPUT].Data;

void setUpCommandI2C(void);
void setUpUltrasonicI2C(void);
void setUpGPIO(void);
void setUpTimers(void);
void setUpVariables(void);
uint32_t clampValue(uint32_t Value, uint32_t LowerLimit, uint32_t UpperLimit);

void initialiseSystemTask(void * Parameters)
{
	int returnCode;
	uint32_t startTime, endTime, waitTime = 10000, i;

	struct InitTaskParameters * params = Parameters;

	setUpGPIO();										// LEDs used for general indicators (green for operational/error state, blue for other)
	setUpTimers();										// PWM, blocking delays, AHRS polling, odometry/motion calculation millisecond timer.
	setUpCommandI2C();									// Command input / odometry out
	setUpUltrasonicI2C();								// Ultrasonic sensor comms
	setUpUART(USART2, AHRS_COMMS_DEFAULT_BAUDRATE);		// AHRS comms
	setUpVariables();									// Initialise various global variables, mainly odometry and control related.

	setGreenLED();
	setBlueLED();




	// Set up and test AHRS
	returnCode = AHRSSetUpSensor(AHRS_COMMS_DEFAULT_BAUDRATE, EULER_BROADCAST_FREQUENCY, QUATERNION_BROADCAST_FREQUENCY, ACCEL_BROADCAST_FREQUENCY);
	if(returnCode)
		showErrorCondition(ERR_NO_AHRS_RESPONSE);

	returnCode = AHRSReadRegister(0xAA, 1);
	if(returnCode)
		showErrorCondition(ERR_NO_AHRS_RESPONSE);



	// Test ultrasonic sensors
	returnCode = ultrasonicsReadSensor(ULTRASONIC_SENSOR_TOP_ADDRESS);
	if(returnCode == -1)
		showErrorCondition(ERR_NO_US_TOP_RESPONSE);

	returnCode = ultrasonicsReadSensor(ULTRASONIC_SENSOR_BOTTOM_ADDRESS);
	if(returnCode == -1)
		showErrorCondition(ERR_NO_US_BOTTOM_RESPONSE);

	returnCode = ultrasonicsReadSensor(ULTRASONIC_SENSOR_FRONT_ADDRESS);
	if(returnCode == -1)
		showErrorCondition(ERR_NO_US_FRONT_RESPONSE);

	returnCode = ultrasonicsReadSensor(ULTRASONIC_SENSOR_BACK_ADDRESS);
	if(returnCode == -1)
		showErrorCondition(ERR_NO_US_BACK_RESPONSE);

	returnCode = ultrasonicsReadSensor(ULTRASONIC_SENSOR_LEFT_ADDRESS);
	if(returnCode == -1)
		showErrorCondition(ERR_NO_US_LEFT_RESPONSE);

	returnCode = ultrasonicsReadSensor(ULTRASONIC_SENSOR_RIGHT_ADDRESS);
	if(returnCode == -1)
		showErrorCondition(ERR_NO_US_RIGHT_RESPONSE);




	// Test Pi Comms
	RegisterMap[REGISTER_STATUS].Bits |= REGISTER_STATUS_IS_WAITING_FOR_PI_READ;

	startTime = TIM2->CNT;
	endTime = startTime + waitTime;

	while(TIM2->CNT < endTime && !(RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_CONFIRM_PI_COMMS));

	if(!(RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_CONFIRM_PI_COMMS))
	{
		showErrorCondition(ERR_NO_PI_COMMS);
	}

	RegisterMap[REGISTER_STATUS].Bits &= ~(REGISTER_STATUS_IS_WAITING_FOR_PI_READ);





	// Test Motors
	RegisterMap[REGISTER_MOTOR_1_PWM].Bits = MOTOR_MIN_PULSE_LENGTH;
	RegisterMap[REGISTER_MOTOR_2_PWM].Bits = MOTOR_MIN_PULSE_LENGTH;
	RegisterMap[REGISTER_MOTOR_3_PWM].Bits = MOTOR_MIN_PULSE_LENGTH;
	RegisterMap[REGISTER_MOTOR_4_PWM].Bits = MOTOR_MIN_PULSE_LENGTH;
	CoSetFlag(params->TestMotorsFlagID);


	// Wait for confirmation before continuing, since this will start the motors
	RegisterMap[REGISTER_STATUS].Bits |= REGISTER_STATUS_IS_WAITING_FOR_MOTOR_CHECK;

	while(!(RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_CONFIRM_MOTOR_TEST));
	RegisterMap[REGISTER_CONTROL_MODE].Bits &= ~(CONTROL_MODE_CONFIRM_MOTOR_TEST);

	for(i = REGISTER_MOTOR_1_PWM; i <= REGISTER_MOTOR_4_PWM; i++)
	{
		RegisterMap[i].Bits = MOTOR_TEST_PULSE_LENGTH;
		startTime = TIM2->CNT;
		endTime = startTime + waitTime;

		while(TIM2->CNT < endTime && !(RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_CONFIRM_MOTOR_TEST));

		RegisterMap[i].Bits = MOTOR_MIN_PULSE_LENGTH;

		if(!(RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_CONFIRM_MOTOR_TEST))
		{
			showErrorCondition(ERR_NO_MOTOR_CONFIRMATION);
		}
	}


	RegisterMap[REGISTER_STATUS].Bits &= (REGISTER_STATUS_IS_WAITING_FOR_MOTOR_CHECK);
	RegisterMap[REGISTER_STATUS].Bits |= REGISTER_STATUS_IS_INITIALISED;
	clearBlueLED();

	CoSetFlag(params->InitCompleteFlagID);
	CoExitTask();
}

/* Sets up GPIOC pins 8 and 9 as outputs
 * These are connected to the LEDs on
 * the board. The green LED is used to
 * show operational or error status. */
void setUpGPIO(void)
{
	GPIO_InitTypeDef gpioInitStruct;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	gpioInitStruct.GPIO_Mode	= GPIO_Mode_OUT;
	gpioInitStruct.GPIO_OType	= GPIO_OType_PP;
	gpioInitStruct.GPIO_Pin		= GPIO_Pin_8 | GPIO_Pin_9;
	gpioInitStruct.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	gpioInitStruct.GPIO_Speed	= GPIO_Speed_2MHz;

	GPIO_Init(GPIOC, &gpioInitStruct);
}

/* Sets up the various timers used by the system:
 * Timer	Use
 * TIM2		Internal millisecond clock
 * TIM3		PWM output
 * TIM6		Internal blocking delays. Not advised for use with OS
 */
void setUpTimers(void)
{
	GPIO_InitTypeDef gpioInitStruct;
	TIM_TimeBaseInitTypeDef timeBaseInitStruct;
	TIM_OCInitTypeDef timOCInitStruct;


	// TIM2 used for internal millisecond clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	timeBaseInitStruct.TIM_ClockDivision		= TIM_CKD_DIV1;
	timeBaseInitStruct.TIM_CounterMode			= TIM_CounterMode_Up;
	timeBaseInitStruct.TIM_Period				= (uint32_t)0xFFFFFFFF;
	timeBaseInitStruct.TIM_Prescaler			= 48000-1;
	timeBaseInitStruct.TIM_RepetitionCounter	= 0;

	TIM_TimeBaseInit(TIM2, &timeBaseInitStruct);
	TIM_Cmd(TIM2, ENABLE);


	/* TIM3 used for PWM outputs. */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	gpioInitStruct.GPIO_Mode	= GPIO_Mode_AF;
	gpioInitStruct.GPIO_OType	= GPIO_OType_PP;
	gpioInitStruct.GPIO_Pin		= GPIO_Pin_6 | GPIO_Pin_7;
	gpioInitStruct.GPIO_PuPd	= GPIO_PuPd_UP;
	gpioInitStruct.GPIO_Speed	= GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);

	gpioInitStruct.GPIO_Pin		= GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &gpioInitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);


	TIM_TimeBaseStructInit(&timeBaseInitStruct);
	timeBaseInitStruct.TIM_ClockDivision		= TIM_CKD_DIV1;
	timeBaseInitStruct.TIM_CounterMode			= TIM_CounterMode_Up;
	timeBaseInitStruct.TIM_Period				= 50000;
	timeBaseInitStruct.TIM_Prescaler			= 48-1;
	timeBaseInitStruct.TIM_RepetitionCounter	= 0;

	TIM_TimeBaseInit(TIM3, &timeBaseInitStruct);

	timOCInitStruct.TIM_OCMode		= TIM_OCMode_PWM1;
	timOCInitStruct.TIM_OCPolarity	= TIM_OCPolarity_High;
	timOCInitStruct.TIM_OutputState	= TIM_OutputState_Enable;
	timOCInitStruct.TIM_Pulse		= 1;

	TIM_OC3Init(TIM3, &timOCInitStruct);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &timOCInitStruct);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	TIM_Cmd(TIM3, ENABLE);



	// TIM6 used for internal blocking delays.
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	timeBaseInitStruct.TIM_ClockDivision		= TIM_CKD_DIV1;
	timeBaseInitStruct.TIM_CounterMode			= TIM_CounterMode_Up;
	timeBaseInitStruct.TIM_Period				= (uint16_t)0xFFFF;
	timeBaseInitStruct.TIM_Prescaler			= 48-1;
	timeBaseInitStruct.TIM_RepetitionCounter	= 0;

	TIM_TimeBaseInit(TIM6, &timeBaseInitStruct);
	TIM_Cmd(TIM6, ENABLE);
}

void setUpCommandI2C()
{
	GPIO_InitTypeDef gpioInitStruct;
	I2C_InitTypeDef i2cInitStruct;

	GPIO_StructInit(&gpioInitStruct);
	I2C_StructInit(&i2cInitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);


	gpioInitStruct.GPIO_Mode 	= GPIO_Mode_AF;
	gpioInitStruct.GPIO_OType	= GPIO_OType_OD;
	gpioInitStruct.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	gpioInitStruct.GPIO_Speed	= GPIO_Speed_Level_3;
	gpioInitStruct.GPIO_Pin		= GPIO_Pin_8 | GPIO_Pin_9;

	GPIO_Init(GPIOB, &gpioInitStruct);


	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1); // SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1); // SDA


	I2C_DeInit(I2C1);
	I2C_Cmd(I2C1, DISABLE);
	i2cInitStruct.I2C_Ack 					= I2C_Ack_Enable;
	i2cInitStruct.I2C_AnalogFilter			= I2C_AnalogFilter_Enable;
	i2cInitStruct.I2C_DigitalFilter			= 0;
	i2cInitStruct.I2C_Mode					= I2C_Mode_I2C;
	i2cInitStruct.I2C_OwnAddress1			= 0x2C << 1;  // Shift left is to allow for the Read/Write bit. Turns out the peripheral doesn't account for this automatically.
	i2cInitStruct.I2C_Timing				= 0x10805E89; // Came from http://hsel.co.uk/2014/08/13/stm32f0-mini-tutorial-using-the-i2c-peripheral-to-communicate-with-a-hmc5883l-digital-compass-ic/
	i2cInitStruct.I2C_AcknowledgedAddress 	= I2C_AcknowledgedAddress_7bit;


	I2C_Init(I2C1, &i2cInitStruct);

	I2C_ITConfig(I2C1, I2C_IT_RXNE, ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_TXIS, ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_ADDR, ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_STOPI, ENABLE);
	NVIC_EnableIRQ(I2C1_IRQn);
	I2C_StretchClockCmd(I2C1, ENABLE);

	I2C_Cmd(I2C1, ENABLE);
}

void setUpUltrasonicI2C(void)
{
	GPIO_InitTypeDef gpioInitStruct;
	I2C_InitTypeDef i2cInitStruct;

	GPIO_StructInit(&gpioInitStruct);
	I2C_StructInit(&i2cInitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);


	gpioInitStruct.GPIO_Mode 	= GPIO_Mode_AF;
	gpioInitStruct.GPIO_OType	= GPIO_OType_OD;
	gpioInitStruct.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	gpioInitStruct.GPIO_Speed	= GPIO_Speed_Level_3;
	gpioInitStruct.GPIO_Pin		= GPIO_Pin_10 | GPIO_Pin_11;

	GPIO_Init(GPIOB, &gpioInitStruct);


	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1); // SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1); // SDA


	I2C_DeInit(I2C2);
	I2C_Cmd(I2C2, DISABLE);
	i2cInitStruct.I2C_Ack 					= I2C_Ack_Enable;
	i2cInitStruct.I2C_AnalogFilter			= I2C_AnalogFilter_Enable;
	i2cInitStruct.I2C_DigitalFilter			= 0;
	i2cInitStruct.I2C_Mode					= I2C_Mode_I2C;
	i2cInitStruct.I2C_Timing				= 0x10805E89; // Came from http://hsel.co.uk/2014/08/13/stm32f0-mini-tutorial-using-the-i2c-peripheral-to-communicate-with-a-hmc5883l-digital-compass-ic/
	i2cInitStruct.I2C_AcknowledgedAddress 	= I2C_AcknowledgedAddress_7bit;


	I2C_Init(I2C2, &i2cInitStruct);

	I2C_StretchClockCmd(I2C2, ENABLE);

	I2C_Cmd(I2C2, ENABLE);
}

/* Sets up the specified USART at the specified BaudRate.
 * Returns 1 if an invalid I2C is specified, 0 if successful.
 */
int setUpUART(USART_TypeDef* USARTx, int BaudRate)
{
	GPIO_InitTypeDef gpioInitStruct;
	USART_InitTypeDef usartInitStruct;

	if(USARTx != USART1 && USARTx != USART2)
		return 1;

	USART_Cmd(USARTx, DISABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	if(USARTx == USART1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	else
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	if (USARTx == USART1)
	{
		// USART1 AF - note comment in GPIO_PinAFConfig is wrong, GPIO_AF_1 should be used.
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
	}
	else
	{
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	}

	// Set up USART1 or USART2 Rx and Tx pins (PA.10 and PA.9, or PA.3 and PA.2 respectively)
	gpioInitStruct.GPIO_Mode	= GPIO_Mode_AF;
	gpioInitStruct.GPIO_OType	= GPIO_OType_PP;
	gpioInitStruct.GPIO_PuPd	= GPIO_PuPd_UP;
	gpioInitStruct.GPIO_Speed	= GPIO_Speed_50MHz;
	if (USARTx == USART1)
		gpioInitStruct.GPIO_Pin	= GPIO_Pin_9 | GPIO_Pin_10;
	else
		gpioInitStruct.GPIO_Pin	= GPIO_Pin_2 | GPIO_Pin_3;

	GPIO_Init(GPIOA, &gpioInitStruct);

	// Set up USART1/2 properties
	usartInitStruct.USART_BaudRate				= BaudRate;
	usartInitStruct.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
	usartInitStruct.USART_Mode					= USART_Mode_Rx | USART_Mode_Tx;
	usartInitStruct.USART_Parity				= USART_Parity_No;
	usartInitStruct.USART_StopBits				= USART_StopBits_1;
	usartInitStruct.USART_WordLength			= USART_WordLength_8b;

	USART_Init(USARTx, &usartInitStruct);

	// Enable Rx interrupt. Tx interrupt is enabled as needed to prevent continuous interrupting.
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
	if(USARTx == USART1)
	{
		NVIC_EnableIRQ(USART1_IRQn);
		USART1->RDR;
	}
	else
	{
		NVIC_EnableIRQ(USART2_IRQn);
		USART2->RDR;
	}

	USART_Cmd(USARTx, ENABLE);

	return 0;
}
void setUpVariables(void)
{
	*(PitchRate)		= 0;
	*(RollRate)			= 0;
	*(YawRate)			= 0;
	*(Pitch)			= 0;
	*(Roll)				= 0;
	*(Yaw)				= 0;
	*(XVel)				= 0;
	*(YVel)				= 0;
	*(ZVel)				= 0;
	*(XPos)				= 0;
	*(YPos)				= 0;
	*(ZPos)				= 0;
	*(XPosGlobal)		= 0;
	*(YPosGlobal)		= 0;
	*(ZPosGlobal)		= 0;
	*(QuaternionA)		= 0;
	*(QuaternionB)		= 0;
	*(QuaternionC)		= 0;
	*(QuaternionD)		= 0;
}

void runSummingBlocks(void *Parameters)
{
	uint32_t output[NUMBER_OF_MOTORS] = {0};
	int delayTicks = 0, i;
	float minExpectedOutput = 0.0f, maxExpectedOutput = 1000.0f;

	//TODO: Figure out if this actually makes any sense at all.
	struct SummingTaskParameters * params = Parameters;

	CoWaitForSingleFlag(params->TestMotorsFlagID, 0);

	while(1)
	{
		if(RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_AUTO_MODE)
		{
			for(i = 0; i < NUMBER_OF_MOTORS; i++)
			{
				float summedOutput = *ThrustOut 	* params->SumBlockParams[i].ThrustMultiplier
								   + *PitchRateOut	* params->SumBlockParams[i].PitchMultiplier
								   + *RollRateOut 	* params->SumBlockParams[i].RollMultiplier
								   + *YawRateOut	* params->SumBlockParams[i].YawMultiplier;

				uint32_t pulseLength = (uint32_t)mapf(summedOutput, minExpectedOutput, maxExpectedOutput, (float)MOTOR_MIN_PULSE_LENGTH, (float)MOTOR_MAX_PULSE_LENGTH);

				output[params->SumBlockParams[i].MotorNumber] = clampValue(pulseLength, MOTOR_MIN_PULSE_LENGTH, MOTOR_MAX_PULSE_LENGTH);
			}
		}

		if(!(RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_OVERIDE_MOTOR_CONTROL))
		{
			for(i = 0; i < NUMBER_OF_MOTORS; i++)
			{
				switch(i)
				{
					case MOTOR_1:
						RegisterMap[REGISTER_MOTOR_1_PWM].Bits = output[i];
						break;

					case MOTOR_2:
						RegisterMap[REGISTER_MOTOR_2_PWM].Bits = output[i];
						break;

					case MOTOR_3:
						RegisterMap[REGISTER_MOTOR_3_PWM].Bits = output[i];
						break;

					case MOTOR_4:
						RegisterMap[REGISTER_MOTOR_4_PWM].Bits = output[i];
						break;

					default:
						RegisterMap[REGISTER_MOTOR_1_PWM].Bits = MOTOR_MIN_PULSE_LENGTH;
						RegisterMap[REGISTER_MOTOR_2_PWM].Bits = MOTOR_MIN_PULSE_LENGTH;
						RegisterMap[REGISTER_MOTOR_3_PWM].Bits = MOTOR_MIN_PULSE_LENGTH;
						RegisterMap[REGISTER_MOTOR_4_PWM].Bits = MOTOR_MIN_PULSE_LENGTH;
				}
			}
		}

		TIM3->CCR1 = RegisterMap[REGISTER_MOTOR_1_PWM].Bits;
		TIM3->CCR2 = RegisterMap[REGISTER_MOTOR_2_PWM].Bits;
		TIM3->CCR3 = RegisterMap[REGISTER_MOTOR_3_PWM].Bits;
		TIM3->CCR4 = RegisterMap[REGISTER_MOTOR_4_PWM].Bits;

		delayTicks = delaymsToTicks(params->UpdatePeriodms);

		CoTickDelay(delayTicks);
	}
}

void delayMicroSeconds(uint16_t NumberOfMicroseconds)
{
	uint16_t count = 0;
	TIM_SetCounter(TIM6, (uint16_t)0);

	do
	{
		count = TIM_GetCounter(TIM6);
	}
	while(count < NumberOfMicroseconds);
}
void delayMilliSeconds(uint16_t NumberOfMilliseconds)
{
	uint16_t i;

	for(i=0;i<NumberOfMilliseconds;i++)
	{
		delayMicroSeconds(1000);
	}
}
void delaySeconds(uint16_t NumberOfSeconds)
{
	uint16_t i;

	for(i = 0; i < NumberOfSeconds; i++)
	{
		delayMilliSeconds(1000);
	}
}
// Light blue LED on PC8
void setBlueLED(void)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
}
// Light green LED on PC9
void setGreenLED(void)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
}
// Turn off blue LED on PC8
void clearBlueLED(void)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
}
// Turn off green LED on PC9
void clearGreenLED(void)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
}

void showErrorCondition(uint8_t errorCode)
{
	uint32_t longFlashTime = 1000, shortFlashTime = 500, waitTime = 500;
	uint32_t longFlashTicks = delaymsToTicks(longFlashTime),
			 shortFlashTicks = delaymsToTicks(shortFlashTime),
			 waitFlashTicks = delaymsToTicks(waitTime);
	uint8_t longFlashes = errorCode >> 4,
			shortFlashes = errorCode & 0x0F,
			i;

	RegisterMap[REGISTER_STATUS].Bits |= REGISTER_STATUS_IS_ERROR | errorCode;

	clearGreenLED();
	clearBlueLED();
	CoTickDelay(waitFlashTicks);

	while(1)
	{
		for(i = 0; i < longFlashes; i++)
		{
			setBlueLED();
			CoTickDelay(longFlashTicks);
			clearBlueLED();
			CoTickDelay(waitFlashTicks);
		}
		for(i = 0; i < shortFlashes; i++)
		{
			setBlueLED();
			CoTickDelay(shortFlashTicks);
			clearBlueLED();
			CoTickDelay(waitFlashTicks);
		}
	}
}
void showOperationalCondition(void)
{
	setGreenLED();
	RegisterMap[REGISTER_STATUS].Bits = REGISTER_STATUS_IS_INITIALISED;
}
void flashBlue(int16_t Time, uint32_t NumberOfFlashes)
{
	uint32_t i;
	for(i = 0; i < NumberOfFlashes; i++)
	{
		setBlueLED();
		delayMilliSeconds(Time);
		clearBlueLED();
		delayMilliSeconds(Time);
	}
}
void toggleBlue()
{
	GPIOC->ODR ^= GPIO_Pin_8;
}
void flashErrorCode(uint8_t ErrorCodeHex)
{
	uint8_t i = 0, shortFlashesCount = ErrorCodeHex & 0x0F, longFlashesCount = (ErrorCodeHex >> 4) & 0x0F;
	uint16_t longTime = 900, shortTime = 500, waitTime = 500;
	uint32_t longTimeTicks = delaymsToTicks(longTime),
			 shortTimeTicks = delaymsToTicks(shortTime),
			 waitTimeTicks = delaymsToTicks(waitTime);

	for(i = 0; i < longFlashesCount; i++)
	{
		setBlueLED();
		CoTickDelay(longTimeTicks);
		clearBlueLED();
		CoTickDelay(waitTimeTicks);
	}

	for(i = 0; i < shortFlashesCount; i++)
	{
		setBlueLED();
		CoTickDelay(shortTimeTicks);
		clearBlueLED();
		CoTickDelay(waitTimeTicks);
	}

	CoTickDelay(5*waitTimeTicks);

}
// Returns Value, clamped between upper and lower limits.
uint32_t clampValue(uint32_t Value, uint32_t LowerLimit, uint32_t UpperLimit)
{
	if(Value <= LowerLimit)
		return LowerLimit;
	else if(Value >= UpperLimit)
		return UpperLimit;
	else
		return Value;
}
