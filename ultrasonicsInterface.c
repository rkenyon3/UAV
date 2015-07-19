#include "ultrasonicsInterface.h"

void ultrasonicsSetupSensors(void)
{
	// TODO: Write this if needed.
}
int ultrasonicsReadSensor(uint8_t SensorAddressToRead)
{
	uint16_t distance;
	uint8_t lowByte, highByte;

	// Write to set pointer and command
	I2C_TransferHandling(I2C2, SensorAddressToRead, 2, I2C_Reload_Mode, I2C_Generate_Start_Write);

	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(I2C2, 0x00); // Pointer register
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TCR) == RESET);

	I2C_SendData(I2C2, 0x51); // range in cm command
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TCR) == RESET);

	I2C_TransferHandling(I2C2, SensorAddressToRead, 1, I2C_Reload_Mode, I2C_Generate_Stop);

	// delay to allow the sensor to range
	CoTimeDelay(0,0,0,100);

	// write to set pointer
	I2C_TransferHandling(I2C2, SensorAddressToRead, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(I2C2, 0x02); // Pointer register
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_TCR) == RESET);

	I2C_TransferHandling(I2C2, SensorAddressToRead, 1, I2C_Reload_Mode, I2C_Generate_Stop);


	// read register (2 bytes);
	I2C_TransferHandling(I2C2, SensorAddressToRead, 2, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE)==RESET);
	highByte = I2C_ReceiveData(I2C2);

	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE)==RESET);
	lowByte = I2C_ReceiveData(I2C2);


	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF) == RESET);

	I2C_ClearFlag(I2C2, I2C_FLAG_STOPF);


	// return result
	// return -1 if error
	if(highByte == 255)
		return -1;

	distance = (highByte << 8) | lowByte;

	return (int)distance;
}








