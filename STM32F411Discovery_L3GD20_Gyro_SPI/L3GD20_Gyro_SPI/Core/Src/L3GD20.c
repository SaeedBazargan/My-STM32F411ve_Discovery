#include "L3GD20.h"

// <---- ------------ Variables ------------ ---->
uint16_t _CS_PinNumber = 0x0000;
GPIO_TypeDef *_CS_GPIO;

uint8_t temp;
uint8_t transmit_buffer[2] = {0, 0};

// <---- ------------ Main L3GD20 Functions ------------ ---->
// <---- ------------ L3GD20 Initialize ------------ ---->
L3GD20_Gyro_Result L3GD20_Init(SPI_HandleTypeDef* SPIx , GPIO_TypeDef *CS_GPIOx, uint16_t CS_PinNumber, L3GD20TypeDef* datastruct)
{
	_CS_GPIO = CS_GPIOx;
	_CS_PinNumber = CS_PinNumber;

	uint8_t WHO_AM_I = (uint8_t)L3GD20_WHO_AM_I_ADDR;

	L3GD20_ReadData(SPIx, &temp, WHO_AM_I, 1);
	if(temp != I_AM_L3GD20_TR)
		return L3GD20_RESULT_ERROR;

	// <---- ------------ Setup All Registers ------------ ---->
	// <---- ------------ Control Register_1 ------------ ---->
	// <---- ------------ DataRate, Bandwidth, PowerMode, Axes Enable ------------ ---->
	transmit_buffer[0] = L3GD20_CTRL_REG1_ADDR;
	transmit_buffer[1] = datastruct -> Gyro_DataRate | datastruct -> Gyro_Bandwidth | ((datastruct -> Gyro_PWR) << 3) | datastruct -> Gyro_Axes;
	L3GD20_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);

	HAL_Delay(25);

	L3GD20_ReadData(SPIx, &temp, L3GD20_CTRL_REG1_ADDR, 1);
	if(temp != transmit_buffer[1])
		return L3GD20_RESULT_ERROR;

	// <---- ------------ Control Register_2 ------------ ---->
	// <---- ------------ High-Pass Filter Mode, High-Pass Filter Cut-off Frequency ------------ ---->
	transmit_buffer[0] = L3GD20_CTRL_REG2_ADDR;
	transmit_buffer[1] = datastruct -> Gyro_FilterMode | datastruct -> Gyro_FilterCutFreq;
	L3GD20_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);

	HAL_Delay(25);

	L3GD20_ReadData(SPIx, &temp, L3GD20_CTRL_REG2_ADDR, 1);
	if(temp != transmit_buffer[1])
		return L3GD20_RESULT_ERROR;

	// <---- ------------ Control Register_3 ------------ ---->
	transmit_buffer[0] = L3GD20_CTRL_REG3_ADDR;
	transmit_buffer[1] = 0x00;					//Default
	L3GD20_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);

	HAL_Delay(25);

	L3GD20_ReadData(SPIx, &temp, L3GD20_CTRL_REG3_ADDR, 1);
	if(temp != transmit_buffer[1])
		return L3GD20_RESULT_ERROR;

	// <---- ------------ Control Register_4 ------------ ---->
	// <---- ------------ FullScale ------------ ---->
	transmit_buffer[0] = L3GD20_CTRL_REG4_ADDR;
	transmit_buffer[1] = datastruct -> Gyro_Scale;
	L3GD20_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);

	HAL_Delay(25);

	L3GD20_ReadData(SPIx, &temp, L3GD20_CTRL_REG4_ADDR, 1);
	if(temp != transmit_buffer[1])
		return L3GD20_RESULT_ERROR;

	// <---- ------------ Control Register_5 ------------ ---->
	// <---- ------------ High-Pass Filter Enable ------------ ---->
	transmit_buffer[0] = L3GD20_CTRL_REG5_ADDR;
	transmit_buffer[1] = datastruct -> Gyro_FilterEn | datastruct -> Gyro_FilterCutFreq;
	L3GD20_WriteData(SPIx, transmit_buffer[0], transmit_buffer[1]);

	HAL_Delay(25);

	L3GD20_ReadData(SPIx, &temp, L3GD20_CTRL_REG5_ADDR, 1);
	if(temp != transmit_buffer[1])
		return L3GD20_RESULT_ERROR;

	return L3GD20_RESULT_OK;
}
// <---- ------------ L3GD20 Read Data ------------ ---->
L3GD20_Gyro_Result L3GD20_ReadData(SPI_HandleTypeDef* SPIx, uint8_t* buffer, uint8_t addr, uint8_t num)
{
	uint8_t reg = addr | 0x80;

	HAL_GPIO_WritePin(_CS_GPIO, _CS_PinNumber, GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPIx, &reg, 1, 50);
	HAL_SPI_Receive(SPIx, buffer, num, 50);
	HAL_GPIO_WritePin(_CS_GPIO, _CS_PinNumber, GPIO_PIN_SET);
	
	/* Return OK */
	return L3GD20_RESULT_OK;    
}
// <---- ------------ L3GD20 Write Data ------------ ---->
L3GD20_Gyro_Result L3GD20_WriteData(SPI_HandleTypeDef* SPIx, uint8_t addr, uint8_t data)
{
	uint8_t buffer[2] = {addr, data};

	HAL_GPIO_WritePin(_CS_GPIO, _CS_PinNumber, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_SPI_Transmit(SPIx, buffer, 2, 50);
	HAL_GPIO_WritePin(_CS_GPIO, _CS_PinNumber, GPIO_PIN_SET);

	return L3GD20_RESULT_OK;
}




