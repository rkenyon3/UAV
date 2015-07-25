#include "ahrsInterface.h"
#include "string.h"

#define UM7_INCOMING_DATA_BUFFER_SIZE 71
#define UM7_OUTGOING_DATA_BUFFER_SIZE 71


// Structure for holding received packet information
typedef struct UM7_packet_struct
{
	uint8_t Address;
	uint8_t PT;
	uint16_t Checksum;
	uint8_t data_length;
	uint8_t data[64];
} UM7_packet;

uint8_t incomingData[UM7_INCOMING_DATA_BUFFER_SIZE];
char outgoingData[UM7_OUTGOING_DATA_BUFFER_SIZE];
char * dataOutPtr = outgoingData;
int sendingData = FALSE;
int sendingDataLength = 0;
uint8_t lastResponseAddress = 0;
int lastResponseSeen = TRUE;
int respondToIncoming = FALSE;

int isInitialised = FALSE;

// ------------ Global variables --------------

// TODO: check that pointer initialisation works as expected.
char FirmwareVersion[5];

float * const PitchRate		= &RegisterMap[REGISTER_PITCH_RATE_ESTIMATE_LOCAL].Data;
float * const RollRate		= &RegisterMap[REGISTER_ROLL_RATE_ESTIMATE_LOCAL].Data;
float * const YawRate		= &RegisterMap[REGISTER_YAW_RATE_ESTIMATE_LOCAL].Data;

float * const Pitch			= &RegisterMap[REGISTER_PITCH_ESTIMATE_LOCAL].Data;
float * const Roll			= &RegisterMap[REGISTER_ROLL_ESTIMATE_LOCAL].Data;
float * const Yaw			= &RegisterMap[REGISTER_YAW_ESTIMATE_LOCAL].Data;

float * const XVel			= &RegisterMap[REGISTER_X_VEL_ESTIMATE_LOCAL].Data;
float * const YVel			= &RegisterMap[REGISTER_Y_VEL_ESTIMATE_LOCAL].Data;
float * const ZVel			= &RegisterMap[REGISTER_Z_VEL_ESTIMATE_LOCAL].Data;

float * const XPos			= &RegisterMap[REGISTER_X_POS_ESTIMATE_LOCAL].Data;
float * const YPos			= &RegisterMap[REGISTER_Y_POS_ESTIMATE_LOCAL].Data;
float * const ZPos			= &RegisterMap[REGISTER_Z_POS_ESTIMATE_LOCAL].Data;

float * const XPosGlobal	= &RegisterMap[REGISTER_X_POS_ESTIMATE_GLOBAL].Data;
float * const YPosGlobal	= &RegisterMap[REGISTER_Y_POS_ESTIMATE_GLOBAL].Data;
float * const ZPosGlobal	= &RegisterMap[REGISTER_Z_POS_ESTIMATE_GLOBAL].Data;

float * const QuaternionA	= &RegisterMap[REGISTER_QUATERNION_A].Data;
float * const QuaternionB	= &RegisterMap[REGISTER_QUATERNION_B].Data;
float * const QuaternionC	= &RegisterMap[REGISTER_QUATERNION_C].Data;
float * const QuaternionD	= &RegisterMap[REGISTER_QUATERNION_D].Data;

int EulerUpdated = FALSE, QuatUpdated = FALSE, OdometryUpdated = FALSE;

// ---------- End Global variables ------------

int xAccelDone 	 		= FALSE, 	yAccelDone 	= FALSE,	zAccelDone 		= FALSE,	accelTimeDone 	= FALSE,
	pitchRollRateDone 	= FALSE, 	yawRateDone = FALSE, 	pitchRollDone	= FALSE, 	yawDone 		= FALSE, 	eulerTimeDone 	= FALSE,
	quatABDone    		= FALSE, 	quatCDDone 	= FALSE, 	quatTimeDone 	= FALSE;

float xAccel = 0, yAccel = 0, zAccel = 0, accelTime = 0, eulerTime = 0, quatTime = 0;

UM7_packet incomingPacket;

uint32_t sendData(char *DataToSend, uint8_t DataLength);
uint32_t sendPacket(UM7_packet *PacketToSend);
void AHRSProcessNewPacket(UM7_packet *NewPacket);
int awaitCommandConfirmed(uint8_t Address);
void setUpSensorComms(void);
void updateOdometry(void);
uint8_t AHRS_parse_serial_data(uint8_t* rx_data, uint8_t rx_length, UM7_packet* packet);

void setUpSensorComms(void)
{
	uint32_t i;
	for(i = 0; i < UM7_INCOMING_DATA_BUFFER_SIZE; i++)
		incomingData[i] = 0;

	for(i = 0; i < UM7_OUTGOING_DATA_BUFFER_SIZE; i++)
		outgoingData[i] = 0;
}
/* Configures the UM7 sensor. Assumes that only processed Euler, quaternion and acceleration data
 * will be required. To stop one of these transmitting, set its broadcastRate to 0. Returns 0 if successful,
 * 1 if invalid BaudRate, 2 if failed due to other transmission in progress, or 3 if failed due to no
 * response from sensor. */
int AHRSSetUpSensor(uint32_t BaudRate, uint8_t EulerBroadcastRate, uint8_t QuaternionBroadcastRate, uint8_t AccelBroadcastRate)
{
	UM7_packet configPacket;
	uint32_t i, limit;

	setUpSensorComms();

	configPacket.Address = 0x00; // Comms config register address.
	configPacket.data_length = 8 * 4; // Number of bytes (8 registers * 4 bytes per register)
	configPacket.PT = UM7_PT_HAS_DATA | UM7_PT_IS_BATCH | (8 << 2);

	// Init
	limit = configPacket.data_length;
	for(i = 0; i < limit; i++)
	{
		configPacket.data[i] = 0;
	}

	// Reg 0x00 - CREG_COM_SETTINGS - data[0-3]
	// Set baud rate (Reg 0x00, bits 31:28, data[0])
	switch(BaudRate)
	{
		case 9600:
			configPacket.data[0] |= 0UL << 4;
			break;

		case 14400:
			configPacket.data[0] |= 1UL << 4;
			break;

		case 19200:
			configPacket.data[0] |= 2UL << 4;
			break;

		case 38400:
			configPacket.data[0] |= 3UL << 4;
			break;

		case 57600:
			configPacket.data[0] |= 4UL << 4;
			break;

		case 115200:
			configPacket.data[0] |= 5UL << 4;
			break;

		case 128000:
			configPacket.data[0] |= 6UL << 4;
			break;

		case 153600:
			configPacket.data[0] |= 7UL << 4;
			break;

		case 230400:
			configPacket.data[0] |= 8UL << 4;
			break;

		case 256000:
			configPacket.data[0] |= 9UL << 4;
			break;

		case 460800:
			configPacket.data[0] |= 10UL << 4;
			break;

		case 921600:
			configPacket.data[0] |= 11UL << 4;
			break;

		default:
			return 1;
	}

	// Registers 0x01 and 0x02 are default (raw data rate) - data[4-11]

	// Reg 0x03 - CREG_COM_RATES3 - processed data - data[12-15]
	configPacket.data[12] = AccelBroadcastRate;

	// Register 0x04 - CREG_COM_RATES4 not used here and are default (all proc rate) - data[16-19]

	// Reg 0x05 - CREG_COM_RATES5 - orientation estimates - data[20-23]
	configPacket.data[20] = QuaternionBroadcastRate;
	configPacket.data[21] = EulerBroadcastRate;

	// All subsequent registers are set to default value of 0

	if (!sendPacket(&configPacket))
		return 2; // Failed due to transmission in progress

	// Wait for data to be sent before reconfiguring baud rate
	while(sendingData);

	//setUpUART(USART2, AHRS_COMMS_BAUDRATE);

	// Enable listening to responses so that the sensor can respond.
	respondToIncoming = TRUE;

	/* Wait until configuration packet has been sent,
	 * before continuing. Prevents interfering with
	 * transmission of packet.
	 */
	if(!awaitCommandConfirmed(configPacket.Address))
		return 3; // Failed due to no response.

	isInitialised = TRUE;

	return 0;
}
/* Sends a command to read data from the specified address.
 * Data from response will be written to the appropriate
 * variables, and may be read after this function returns 0.
 * Returns 1 if failed due to other transmission in progress,
 * or 2 if failed due to no response. */
int AHRSReadRegister(uint8_t Address, uint8_t BatchLength)
{
	UM7_packet regReadPacket;

	if(!isInitialised)
		return 1;
	else
	{
		regReadPacket.Address = Address; // Address to read.
		if(BatchLength == 0)
			regReadPacket.PT = 0x00;
		else
			regReadPacket.PT = UM7_PT_IS_BATCH | BatchLength << 2;
		regReadPacket.data_length = 0;

		if (!sendPacket(&regReadPacket))
			return 2; // Failed due to transmission in progress

		if(!awaitCommandConfirmed(regReadPacket.Address))
			return 3; // Failed due to no response.
	}

	return 0;
}

/* Processes a newly received packet to write the data
 * into the relevant registers. Performs no validity checks,
 * as the packet should already have been checked by the
 * initial parser.
 */
void AHRSProcessNewPacket(UM7_packet *NewPacket)
{
	union Register tmp;
	int16_t temp = 0;
	uint8_t i = 0, j = 0;
	uint8_t currAddress = NewPacket->Address;
	const float eulerDivisor = 91.02222;	// from manual
	const float eulerRateDivisor = 16.0;	// from manual
	const float quatDivisor = 29789.09091;	// from manual


	for(i = 0; i < NewPacket->data_length; i+=4)
	{
		// Switch based on packet address and process data as appropriate
		switch(currAddress)
		{
			case UM7_ACCEL_PROC_X_REGISTER_ADDRESS:
				tmp.Bits = NewPacket->data[i] << 3 | NewPacket->data[i + 1] << 2 | NewPacket->data[i + 2] << 1 | NewPacket->data[i + 3];
				xAccel = tmp.Data;
				xAccelDone = TRUE;
				break;

			case UM7_ACCEL_PROC_Y_REGISTER_ADDRESS:
				tmp.Bits = NewPacket->data[i] << 3 | NewPacket->data[i + 1] << 2 | NewPacket->data[i + 2] << 1 | NewPacket->data[i + 3];
				yAccel = tmp.Data;
				yAccelDone = TRUE;
				break;

			case UM7_ACCEL_PROC_Z_REGISTER_ADDRESS:
				tmp.Bits = NewPacket->data[i] << 3 | NewPacket->data[i + 1] << 2 | NewPacket->data[i + 2] << 1 | NewPacket->data[i + 3];
				zAccel = tmp.Data;
				zAccelDone = TRUE;
				break;

			case UM7_ACCEL_PROC_TIME_REGISTER_ADDRESS:
				accelTime = (float)(NewPacket->data[i] << 3 | NewPacket->data[i + 1] << 2 | NewPacket->data[i + 2] << 1 | NewPacket->data[i + 3]);
				accelTimeDone = TRUE;
				break;


			case UM7_PITCH_ROLL_REGISTER_ADDRESS:
				// Determine pitch first (lower 16 bits of register)
				temp = NewPacket->data[i] << 8 | NewPacket->data[i + 1];
				*Pitch = (float)(temp / eulerDivisor);

				// Next find roll (upper 16 bits of register)
				temp = NewPacket->data[i + 2] << 8 | NewPacket->data[i + 3];
				*Roll = (float)(temp / eulerDivisor);

				pitchRollDone = TRUE;
				break;

			case UM7_YAW_REGISTER_ADDRESS:
				// Yaw is stored in the upper 16 bits
				temp = NewPacket->data[i] << 8 | NewPacket->data[i + 1];
				*Yaw = (float)(temp / eulerDivisor);

				yawDone = TRUE;
				break;

			case UM7_PITCH_ROLL_RATE_REGISTER_ADDRESS:
				// Determine pitch rate first (lower 16 bits of register)
				temp = NewPacket->data[i] << 8 | NewPacket->data[i + 1];
				*PitchRate = (float)(temp / eulerRateDivisor);

				// Next find roll (upper 16 bits of register)
				temp = NewPacket->data[i + 2] << 8 | NewPacket->data[i + 3];
				*RollRate = (float)(temp / eulerRateDivisor);

				pitchRollRateDone = TRUE;
				break;

			case UM7_YAW_RATE_REGISTER_ADDRESS:
				// Yaw is stored in the upper 16 bits
				temp = NewPacket->data[i] << 8 | NewPacket->data[i + 1];
				*YawRate = (float)(temp / eulerRateDivisor);

				yawRateDone = TRUE;
				break;

			case UM7_EULER_TIME_REGISTER_ADDRESS:
				eulerTime = (float)(NewPacket->data[i] << 3 | NewPacket->data[i + 1] << 2 | NewPacket->data[i + 2] << 1 | NewPacket->data[i + 3]);
				eulerTimeDone = TRUE;
				break;


			case UM7_QUAT_AB_REGISTER_ADDRESS:
				// Determine quaternion component A first (upper 16 bits of register)
				temp = NewPacket->data[i + 2] << 8 | NewPacket->data[i + 3];
				*QuaternionA = (float)(temp / quatDivisor);

				// Next find quaternion component B (lower 16 bits of register)
				temp = NewPacket->data[i] << 8 | NewPacket->data[i + 1];
				*QuaternionB = (float)(temp / quatDivisor);

				quatABDone = TRUE;
				break;

			case UM7_QUAT_CD_REGISTER_ADDRESS:
				// Determine quaternion component C first (upper 16 bits of register)
				temp = NewPacket->data[i + 2] << 8 | NewPacket->data[i + 3];
				*QuaternionC = (float)(temp / quatDivisor);

				// Next find quaternion component D (lower 16 bits of register)
				temp = NewPacket->data[i] << 8 | NewPacket->data[i + 1];
				*QuaternionD = (float)(temp / quatDivisor);

				quatCDDone = TRUE;
				break;

			case UM7_QUAT_TIME_REGISTER_ADDRESS:
				quatTime = (float)(NewPacket->data[i] << 3 | NewPacket->data[i + 1] << 2 | NewPacket->data[i + 2] << 1 | NewPacket->data[i + 3]);
				quatTimeDone = TRUE;
				break;

			case UM7_GET_FIRMWARE_VERSION_CMD_ADDRESS:
				for(j = 0; j < 4; j++)
					FirmwareVersion[j] = NewPacket->data[i + j];

				FirmwareVersion[4] = '\0';
				break;
		}
		// calculate address of register which next set of data came from.
		currAddress += 0x01;
	}

	lastResponseAddress=NewPacket->Address;
	lastResponseSeen = FALSE;
}

/* Polls the sensor for various items of data, and updates odometry as needed.
 * Returns 0 if successful, 1 if sensor not initialised, 2 if another transmission
 * interfered, or 3 if no response was recieved.
 */
int AHRSPoll(void)
{
	int readReturnCode = 0;

	// Publicly visible attitude and odometry estimates
	EulerUpdated 	= FALSE;
	QuatUpdated 	= FALSE;
	OdometryUpdated = FALSE;

	// Private variables recording which parameters are up to date. Updated in processNewPacket
	xAccelDone 		= FALSE;
	yAccelDone 		= FALSE;
	zAccelDone 		= FALSE;
	accelTimeDone 	= FALSE;
	pitchRollDone 	= FALSE;
	yawDone 		= FALSE;
	eulerTimeDone 	= FALSE;
	quatABDone    	= FALSE;
	quatCDDone 		= FALSE;
	quatTimeDone 	= FALSE;

	if(POLL_ACCEL_DATA)
		readReturnCode = AHRSReadRegister(UM7_ACCEL_PROC_X_REGISTER_ADDRESS, 4);


	if(POLL_EULER_DATA && readReturnCode == 0)
	{
		readReturnCode = AHRSReadRegister(UM7_PITCH_ROLL_REGISTER_ADDRESS, 5);
	}

	if(POLL_QUATERNION_DATA && readReturnCode == 0)
		readReturnCode = AHRSReadRegister(UM7_QUAT_AB_REGISTER_ADDRESS, 3);


	if(xAccelDone && yAccelDone && zAccelDone && accelTimeDone
			&& pitchRollDone && yawDone && eulerTimeDone
			&& readReturnCode == 0)
	{
		updateOdometry();
	}

	return readReturnCode;
}

void updateOdometry(void)
{
	/* see http://www.chrobotics.com/docs/AN-1007-EstimatingVelocityAndPositionUsingAccelerometers.pdf
	 * and http://www.chrobotics.com/docs/AN-1005-UnderstandingEulerAngles.pdf for details. */

	// Note: assume first index for matrices is the row index
	// Note: g is set to {0, 0, 0} as the sensor appears to be removing gravity itself.

	float Rv1v2[3][3], Rv2B[3][3], RBI[3][3], am[3], amRotated[3],
		  aI[3], g[] = {0.0f, 0.0f, 0.0f}, interUpdateTime = 0.0f,
		  pitchRad =  *Pitch * (M_PI/180.0f), rollRad = *Roll * (M_PI/180.0f);//, yawRad = *Yaw * (M_PI/180.0f);
	uint32_t prevUpdateTime = RegisterMap[REGISTER_ODOMETRY_UPDATE_TIME].Bits;
	uint32_t updateTime;

	RegisterMap[REGISTER_ODOMETRY_UPDATE_TIME].Bits = TIM2->CNT;
	updateTime = RegisterMap[REGISTER_ODOMETRY_UPDATE_TIME].Bits;

	Rv1v2[0][0] = cos(pitchRad);
	Rv1v2[0][1] = 0;
	Rv1v2[0][2] = -sin(pitchRad);
	Rv1v2[1][0] = 0;
	Rv1v2[1][1] = 1;
	Rv1v2[1][2] = 0;
	Rv1v2[2][0] = sin(pitchRad);
	Rv1v2[2][1] = 0;
	Rv1v2[2][2] = cos(pitchRad);

	Rv2B[0][0]  = 1;
	Rv2B[0][1]  = 0;
	Rv2B[0][2]  = 0;
	Rv2B[1][0]  = 0;
	Rv2B[1][1]  = cos(rollRad);
	Rv2B[1][2]  = sin(rollRad);
	Rv2B[2][0]  = 0;
	Rv2B[2][1]  = -sin(rollRad);
	Rv2B[2][2]  = cos(rollRad);

	MultiplyMatrices(Rv1v2,Rv2B,RBI);

	am[0] = xAccel;
	am[1] = yAccel;
	am[2] = zAccel;

	MultiplyMatrixVector(RBI, am, amRotated);

	AddVectors(amRotated, g, aI);

	interUpdateTime = (float)(updateTime/1000.0f) - (float)(prevUpdateTime/1000.0f);

	*XVel = *XVel + aI[0]*interUpdateTime;
	*YVel = *YVel + aI[1]*interUpdateTime;
	*ZVel = *ZVel + aI[2]*interUpdateTime;
}


// parse_serial_data
// This function parses the data in ‘rx_data’ with length ‘rx_length’ and attempts to find a packet
// in the data. If a packet is found, the structure ‘packet’ is filled with the packet data.
// If there is not enough data for a full packet in the provided array, parse_serial_data returns 1.
// If there is enough data, but no packet header was found, parse_serial_data returns 2.
// If a packet header was found, but there was insufficient data to parse the whole packet,
// then parse_serial_data returns 3. This could happen if not all of the serial data has been
// received when parse_serial_data is called.
// If a packet was received, but the checksum was bad, parse_serial_data returns 4.
// If a good packet was received, parse_serial_data fills the UM7_packet structure and returns 0.
// --- This function was taken from the CHR-UM7 user manual ---
uint8_t AHRS_parse_serial_data(uint8_t* rx_data, uint8_t rx_length, UM7_packet* packet)
{
	uint8_t index;
	// Make sure that the data buffer provided is long enough to contain a full packet
	// The minimum packet length is 7 bytes
	if(rx_length < 7)
	{
		return 1;
	}

	// Try to find the 'snp' start sequence for the packet
	for(index = 0; index < (rx_length - 2); index++)
	{
		// Check for 'snp'. If found, immediately exit the loop
		if(rx_data[index] == 's' && rx_data[index+1] == 'n' && rx_data[index+2] == 'p')
		{
			break;
		}
	}

	uint8_t packet_index = index;
	// Check to see if the variable ‘packet_index’ is equal to (rx_length - 2). If it is, then the above
	// loop executed to completion and never found a packet header.
	if(packet_index == (rx_length - 2))
	{
		return 2;
	}

	// If we get here, a packet header was found. Now check to see if we have enough room
	// left in the buffer to contain a full packet. Note that at this point, the variable ‘packet_index’
	// contains the location of the ‘s’ character in the buffer (the first byte in the header)
	if((rx_length - packet_index) < 7)
	{
		return 3;
	}

	// We’ve found a packet header, and there is enough space left in the buffer for at least
	// the smallest allowable packet length (7 bytes). Pull out the packet type byte to determine
	// the actual length of this packet
	uint8_t PT = rx_data[packet_index + 3];

	// Do some bit-level manipulation to determine if the packet contains data and if it is a batch
	// We have to do this because the individual bits in the PT byte specify the contents of the
	// packet.
	uint8_t packet_has_data = (PT >> 7) & 0x01; // Check bit 7 (HAS_DATA)
	uint8_t packet_is_batch = (PT >> 6) & 0x01; // Check bit 6 (IS_BATCH)
	uint8_t batch_length = (PT >> 2) & 0x0F; // Extract the batch length (bits 2 through 5)

	// Now finally figure out the actual packet length
	uint8_t data_length = 0;
	if(packet_has_data)
	{
		if(packet_is_batch)
		{
			// Packet has data and is a batch. This means it contains ‘batch_length' registers, each
			// of which has a length of 4 bytes
			data_length = 4*batch_length;
		}
		else // Packet has data but is not a batch. This means it contains one register (4 bytes)
		{
			data_length = 4;
		}
	}
	else // Packet has no data
	{
		data_length = 0;
	}

	// At this point, we know exactly how long the packet is. Now we can check to make sure
	// we have enough data for the full packet.
	if((rx_length - packet_index) < (data_length + 5))
	{
		return 3;
	}

	// If we get here, we know that we have a full packet in the buffer. All that remains is to pull
	// out the data and make sure the checksum is good.
	// Start by extracting all the data
	packet->Address = rx_data[packet_index + 4];
	packet->PT = PT;

	// Get the data bytes and compute the checksum all in one step
	packet->data_length = data_length;
	uint16_t computed_checksum = 's' + 'n' + 'p' + packet->PT + packet->Address;
	for( index = 0; index < data_length; index++ )
	{
		// Copy the data into the packet structure’s data array
		packet->data[index] = rx_data[packet_index + 5 + index];
		// Add the new byte to the checksum
		computed_checksum += packet->data[index];
	}

	// Now see if our computed checksum matches the received checksum
	// First extract the checksum from the packet
	uint16_t received_checksum = (rx_data[packet_index + 5 + data_length] << 8);
	received_checksum |= rx_data[packet_index + 6 + data_length];

	// Now check to see if they don’t match
	if( received_checksum != computed_checksum )
	{
		return 4;
	}

	// At this point, we’ve received a full packet with a good checksum. It is already
	// fully parsed and copied to the ‘packet’ structure, so return 0 to indicate that a packet was
	// processed.
	return 0;
}

/* Begins sending a packet to the sensor. Calculates
 * checksum and initiates transmission. Not a blocking
 * call. Returns 1 if transmission starts successfully,
 * otherwise 0. Requires that PT be set up in the
 * calling function. */
uint32_t sendPacket(UM7_packet *PacketToSend)
{
	char rawData[UM7_OUTGOING_DATA_BUFFER_SIZE];
	uint16_t checksum;
	uint8_t i = 0;

	// Construct header
	rawData[0] = 's';
	rawData[1] = 'n';
	rawData[2] = 'p';
	rawData[3] = PacketToSend->PT;
	rawData[4] = PacketToSend->Address;

	/* If the packet is a write operation (has data),
	 * copy as many packetData as applicable to rawData. */
	if(PacketToSend->PT & UM7_PT_HAS_DATA)
	{
		for(i = 0; i < PacketToSend->data_length; i++)
			rawData[i+5] = PacketToSend->data[i];
	}
	i += 5;

	checksum = calcCheckSum(rawData, i);

	rawData[i] = (checksum >> 8) & 0x00FF;
	i++;
	rawData[i] = checksum & 0x00FF;
	i++;


	return sendData(rawData, i);

}
/* Begins sending data to the sensor. This is not a blocking
 * call, as the actual transmission is interrupt driven. Returns
 * 1 if the data starts to transmit successfully, otherwise 0.
 */
uint32_t sendData(char *DataToSend, uint8_t DataLength)
{
	uint8_t i;
	// Only continue if not sending anything already.
	if (!sendingData)
	{
		sendingData = TRUE;
		sendingDataLength = DataLength;
		// Move pointer back to start of buffer.
		dataOutPtr = outgoingData;

		// Copy data into buffer. Can't use strcpy due to zeros being interpreted as nulls.
		for (i = 0; i < DataLength; i++)
		{
			outgoingData[i] = *(DataToSend + i);
		}

		// Allow USART ISR to take over.
		NVIC_DisableIRQ(USART2_IRQn);
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);	// USART1 Tx interrupt enable
		NVIC_EnableIRQ(USART2_IRQn);

		return TRUE;
	}

	return FALSE;
}

int awaitCommandConfirmed(uint8_t Address)
{
	int isConfirmed = FALSE;
	uint32_t i = 0, timeout = 5000;

	for(i = 0; i < timeout; i++)
	{
		if(lastResponseAddress == Address && !lastResponseSeen)
		{
			lastResponseSeen = TRUE;
			isConfirmed = TRUE;
			break;
		}
		// TODO: Convert this to work on OS delays.
		delayMicroSeconds((uint16_t)1000); // Spins with 1 ms delays.
	}

	return isConfirmed;
}

void USART2_IRQHandler(void)
{
	static uint8_t rxIndex = 0;
	static int bytesSent = 0;
	uint8_t invalidPacket = TRUE;

	// Handle RXNE interrupt
	if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE))
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		USART_ClearFlag(USART2, USART_FLAG_RXNE);

		//if(respondToIncoming)
		if(rxIndex < UM7_INCOMING_DATA_BUFFER_SIZE)
		{
			incomingData[rxIndex] = USART2->RDR;
			rxIndex++;

//			invalidPacket = AHRS_parse_serial_data(incomingData, rxIndex, &incomingPacket);
//
//			if(!invalidPacket)
//			{
//				rxIndex = 0;
//
//				AHRSProcessNewPacket(&incomingPacket);
//			}
		}
		else
		{
			rxIndex++;
		}
	}
	// Handle TXE interrupt
	else if (USART_GetFlagStatus(USART2, USART_FLAG_TXE))
	{
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
		USART_ClearFlag(USART2, USART_FLAG_TXE);

		USART2->TDR = *dataOutPtr;
		dataOutPtr++;
		bytesSent++;

		if(bytesSent >= sendingDataLength)
		{
			NVIC_DisableIRQ(USART2_IRQn);
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);	// Disable USART2 TXE interrupt when done sending.
			NVIC_EnableIRQ(USART2_IRQn);

			bytesSent = 0;
			sendingData = FALSE;
		}
	}
	else if(USART_GetFlagStatus(USART2, USART_FLAG_IDLE))
	{
	}
}
