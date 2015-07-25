#include "control.h"

#define CONTROL_INCOMING_DATA_BUFFER_SIZE 71
#define CONTROL_OUTGOING_DATA_BUFFER_SIZE 71

// Structure for holding received packet information
typedef struct Control_Packet_Struct
{
	uint8_t Address;
	uint8_t PT;
	uint16_t Checksum;
	uint8_t data_length;
	uint8_t data[64];
} Control_Packet;

enum Error_Packet_Type {Error_Packet_Checksum, Error_Packet_General, Error_Packet_Cmd_Rejected};
union Register RegisterMap[REGISTER_MAP_LENGTH];
uint8_t AutoModeAddressWhiteList[AUTO_MODE_WHITELIST_LENGTH] = {REGISTER_YAW_RATE_PID_SET_POINT,
																REGISTER_X_VEL_PID_SET_POINT,
																REGISTER_Y_VEL_PID_SET_POINT,
																REGISTER_Z_VEL_PID_SET_POINT,
																REGISTER_CONTROL_MODE
																};

uint8_t rxBuffer[CONTROL_INCOMING_DATA_BUFFER_SIZE];
char txBuffer[CONTROL_OUTGOING_DATA_BUFFER_SIZE];


void GenerateErrorPacket(uint8_t OriginalPacketAddress, enum Error_Packet_Type ErrType);
void serialisePacket(Control_Packet *Packet);
void processNewControlPacket(Control_Packet *Packet);
int parseControlSerialData(uint8_t *RXData, uint8_t RXLength, Control_Packet *Packet);


/* Processes a new packet from the Pi.
 *
 * If the packet contains write instructions,
 * either write the new data to the relevant
 * registers and generate a confirmation packet
 * or generate an error packet if the system
 * is in AUTO mode and the user attempts to
 * access a restricted variable.
 *
 * If the packet contains read instructions,
 * fill the outgoing data buffer with the
 * data from the start address onwards and
 * append a checksum.
 *
 * Generates an error packet if the user
 * attempts to write to or read from a location
 * beyond the end of the register map. Does
 * not prevent the user from writing to unused
 * memory locations.
 */
void processNewControlPacket(Control_Packet *Packet)
{
	Control_Packet returnPacket;
	uint8_t batchLength = Packet->PT & CONTROL_PACKET_PT_BATCH_LENGTH_BITS, currentAddress, i, j;
	int isWrite = Packet->PT & CONTROL_PACKET_PT_HAS_DATA, isNotWritePermitted = FALSE;

	if(RegisterMap[REGISTER_STATUS].Bits & REGISTER_STATUS_IS_WAITING_FOR_PI_READ)
		RegisterMap[REGISTER_CONTROL_MODE].Bits |= CONTROL_MODE_CONFIRM_PI_COMMS;

	/* Generate error if the user attempts to access data
	 * beyond the bounds of the register map. */
	if(Packet->Address + batchLength >= REGISTER_MAP_LENGTH)
	{
		GenerateErrorPacket(Packet->Address, Error_Packet_General);
		return;
	}

	/* Generate an error if the user attempts to access
	 * restricted memory locations. */
	if(isWrite)
	{
		if(RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_AUTO_MODE)
		{
			for(i = 0; i < batchLength; i++)
			{
				int validAddress = FALSE;
				for(j = 0; j < AUTO_MODE_WHITELIST_LENGTH && !isNotWritePermitted; j++)
				{
					if(Packet->Address + i == AutoModeAddressWhiteList[j])
					{
						validAddress = TRUE;
					}
				}
				if(!validAddress)
					isNotWritePermitted = TRUE;
			}
			if(isNotWritePermitted)
			{
				// Generate error return packet.
				GenerateErrorPacket(Packet->Address, Error_Packet_General);
				return;
			}
		}
	}

	if(isWrite)
	{
		currentAddress = Packet->Address;

		for(i = 0; i < Packet->data_length; i+=4)
		{
			RegisterMap[currentAddress].Bits = Packet->data[i + 3] << 24 | Packet->data[i + 2] << 16 | Packet->data[i + 1] << 8 | Packet->data[i];
			currentAddress++;
		}

		// Construct confirmation packet
		returnPacket.Address = Packet->Address;
		returnPacket.PT =  0; // No data, no error.
		returnPacket.data_length = 0;
	}
	else
	{
		currentAddress = Packet->Address;

		returnPacket.Address = Packet->Address;
		returnPacket.PT = CONTROL_PACKET_PT_HAS_DATA | batchLength;
		returnPacket.data_length = Packet->data_length;

		for(i = 0; i < Packet->data_length; i+=4)
		{
			returnPacket.data[i] 	 =  RegisterMap[currentAddress].Bits & 0xFF;
			returnPacket.data[i + 1] = (RegisterMap[currentAddress].Bits >> 8) & 0xFF;
			returnPacket.data[i + 2] = (RegisterMap[currentAddress].Bits >> 16) & 0xFF;
			returnPacket.data[i + 3] = (RegisterMap[currentAddress].Bits >> 24) & 0xFF;
			currentAddress++;
		}
	}

	serialisePacket(&returnPacket);
}

/* Generates a packet that indicates an error occurred
 * and the starting address of the serial read or write
 * that caused the error, then serialises the packet
 * and loads it into the txBuffer.
 */
void GenerateErrorPacket(uint8_t OriginalPacketAddress, enum Error_Packet_Type ErrType)
{
	// Sleepy coding from here. Beware of errors.
	Control_Packet errPacket;

	errPacket.Address 		= OriginalPacketAddress;
	errPacket.data_length	= 0;

	if(ErrType == Error_Packet_Checksum)
		errPacket.PT = 0x40;
	else if(ErrType == Error_Packet_General)
		errPacket.PT = 0x20;
	else if(ErrType == Error_Packet_Cmd_Rejected)
		errPacket.PT = 0x10;

	serialisePacket(&errPacket);
}

/* Converts a packet into a serial form suitable
 * for transmission over the I2C line, then
 * stores this data in the txBuffer, ready to
 * be transmitted.
 */
void serialisePacket(Control_Packet *Packet)
{
	uint8_t i = 0;
	uint16_t checksum;

	txBuffer[0] = 's';
	txBuffer[1] = 'n';
	txBuffer[2] = 'p';
	txBuffer[3] = Packet->PT;
	txBuffer[4] = Packet->Address;
	for(i = 0; i < Packet->data_length; i++)
		txBuffer[i+5] = Packet->data[i];

	checksum = calcCheckSum(txBuffer, i + 5);
	txBuffer[i+5] = (checksum >> 8) & 0x00FF;
	txBuffer[i+6] = checksum & 0x00FF;
}

/* Parses a buffer of serial data to determine if it
 * contains a control packet. If a full packet is
 * present, it is extracted and copied into rxBuffer.
 * This function is modified from the equivalent function
 * provided for the AHRS (see ahrsInterface.c).
 *
 * Returns 1 if the buffer cannot contain a full packet
 * (min 7 bytes), 2 if the buffer is contains no packet
 * header, 3 if the packet contains a partial packet,
 * or 4 if the packet has a bad checksum.
 */
int parseControlSerialData(uint8_t *RXData, uint8_t RXLength, Control_Packet *PacketToFill)
{
	uint8_t index = 0, dataLength = 0, i = 0, PT = 0;
	uint16_t calcChecksum = 0, recChecksum = 0;
	int isWrite = FALSE;

	// Reject if the buffer is too short to contain data (min 7 bytes).
	if(RXLength < 7)
		return 1;

	// Attempt to find a packet header 'snp'.
	for(index = 0; index < RXLength-2; index++)
	{
		if(RXData[index] == 's' && RXData[index + 1] == 'n' && RXData[index + 2] == 'p')
			break;
	}

	// check if the packet header was found
	if(index == RXLength-2)
		return 2;

	/* check if the buffer could contain a packet from
	 * the header starting index. */
	if(RXLength - index < 7)
		return 3;


	PT = RXData[index+3];

	// Determine read or write
	if(PT & CONTROL_PACKET_PT_HAS_DATA)
		isWrite = TRUE;
	else
		isWrite = FALSE;

	dataLength = 4 * (PT & CONTROL_PACKET_PT_BATCH_LENGTH_BITS);

	if(isWrite)
	{
		if(RXLength - index < dataLength + 7)
			return 3;
	}

	PacketToFill->Address = RXData[index + 4];
	PacketToFill->PT = PT;
	PacketToFill->data_length = dataLength;

	// Extract data and calculate checksum
	calcChecksum = 's' + 'n' + 'p' + PacketToFill->Address + PacketToFill->PT;
	if(isWrite)
	{
		for(i = 0; i < dataLength; i++)
		{
			PacketToFill->data[i] = RXData[index + 5 + i];
			calcChecksum += RXData[index + 5 + i];
		}
		recChecksum = (RXData[index + 5 + dataLength] << 8) | RXData[index + 6 + dataLength];
	}
	else
		recChecksum = (RXData[index + 5] << 8) | RXData[index + 6];

	if(calcChecksum != recChecksum)
		return 4;

	PacketToFill->Checksum = recChecksum;

	return 0;
}
void runAHRSPolling(void * Parameters)
{
	struct PollingParameters * params = Parameters;

	CoWaitForSingleFlag(params->InitCompleteFlagID,0);

	while(1)
	{
		if(RegisterMap[REGISTER_CONTROL_MODE].Bits & CONTROL_MODE_AUTO_MODE)
		{
			CoPendSem(params->DataSemaphoreID, 0);

			if(!AHRSPoll())
				showErrorCondition(ERR_AHRS_FAILED);

			CoPostSem(params->DataSemaphoreID);
		}
		CoTickDelay(delaymsToTicks(RegisterMap[REGISTER_AHRS_POLLING_PERIOD].Bits));
	}
}
void runUltrasonicPolling(void * Parameters)
{
	int returnCode, i;

	struct PollingParameters * params = Parameters;

	CoWaitForSingleFlag(params->InitCompleteFlagID,0);

	for(i = 0; i < 6; i++)
	{
		CoPendSem(params->DataSemaphoreID, 0);

		returnCode = ultrasonicsReadSensor(ULTRASONIC_SENSOR_TOP_ADDRESS + (i << 1));
		if(returnCode == -1)
			showErrorCondition(ERR_NO_US_TOP_RESPONSE + 1);

		CoPostSem(params->DataSemaphoreID);
	}

	CoTickDelay(delaymsToTicks(RegisterMap[REGISTER_ULTRASONIC_POLLING_PERIOD].Bits));
}
void I2C1_IRQHandler(void)
{
	static uint8_t rxIndex = 0, txIndex = 0;
	int invalidPacket = TRUE;
	Control_Packet incomingPacket;

	if (I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR))
	{
		if (I2C1->ISR & I2C_ISR_DIR)	// Read transfer, slave transmitting
		{
			txIndex = 0;
			I2C1->TXDR = txBuffer[txIndex];
			txIndex++;
		}
		else
		{
			rxIndex = 0;
		}

		I2C_ClearITPendingBit(I2C1, I2C_IT_ADDR);
		I2C_ClearFlag(I2C1, I2C_FLAG_ADDR);
	}
	else if(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE))
	{
		I2C_ClearFlag(I2C1, I2C_FLAG_RXNE);
		char c = I2C1->RXDR;
		rxBuffer[rxIndex] = c;
		rxIndex++;

		invalidPacket = parseControlSerialData(rxBuffer, rxIndex, &incomingPacket);

		if(!invalidPacket)
		{
			rxIndex = 0;

			processNewControlPacket(&incomingPacket);
		}
		else if(invalidPacket == 4)
		{
			// Address set to 0xFF as the address of a corrupt packet cannot be reliably known.
			GenerateErrorPacket(0xFF, Error_Packet_Checksum);
		}
	}
	else if(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS))
	{
		I2C_ClearFlag(I2C1, I2C_FLAG_TXIS);
		I2C1->TXDR = txBuffer[txIndex];
		txIndex++;
	}
	else if(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_STOPI);
		I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);

		I2C1->ISR |= I2C_ISR_TXE;
	}
	I2C1->ICR &= ~I2C_ICR_ADDRCF; // Clear bit to release clock stretching and allow transmission to continue
}
