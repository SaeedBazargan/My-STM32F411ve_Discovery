#include "LSM303DL.h"

// <---- ------------ Variables ------------ ---->
uint8_t temp = 0x00;
uint8_t transmit_buffer[2] = {0, 0};
uint8_t I2C_ADDRESS = 0x00;

// <---- ------------ Main LSM303DL Functions ------------ ---->
// <---- ------------ LSM303DL Initialize ------------ ---->
LSM303DLHC_Accel_Result LSM303DL_Init(I2C_HandleTypeDef* I2Cx , LSM303DLTypeDef* datastruct)
{
	uint8_t IsDeviceReady = 0xFF;

    IsDeviceReady = HAL_I2C_IsDeviceReady(I2Cx, ACC_I2C_ADDRESS, 1, 100);
	if(IsDeviceReady == HAL_OK)
		I2C_ADDRESS = ACC_I2C_ADDRESS;

    LSM303DL_ReadData(I2Cx, &temp, LSM303DLHC_WHO_AM_I_ADDR, 1);
	if(temp != I_AM_LMS303DLHC)
		return LSM303DL_RESULT_ERROR;

	// <---- ------------ Setup All Registers ------------ ---->
	// <---- ------------ Control Register_1 ------------ ---->
	// <---- ------------ DataRate, PowerMode, Axes Enable ------------ ---->
	transmit_buffer[0] = LSM303DLHC_CTRL_REG1_A;
	transmit_buffer[1] = datastruct -> Accel_DataRate | ((datastruct -> Accel_PWR) << 3) | datastruct -> Accel_Axes;
	LSM303DL_WriteData(I2Cx, transmit_buffer[0], transmit_buffer[1]);

	HAL_Delay(25);

	LSM303DL_ReadData(I2Cx, &temp, LSM303DLHC_CTRL_REG1_A, 1);
	if(temp != transmit_buffer[1])
		return LSM303DL_RESULT_ERROR;

	// <---- ------------ Control Register_2 ------------ ---->
	// <---- ------------ High-Pass Filter Mode, High-Pass Filter Cut-off Frequency ------------ ---->
	transmit_buffer[0] = LSM303DLHC_CTRL_REG2_A;
	transmit_buffer[1] = datastruct -> Accel_FilterMode | datastruct -> Accel_FilterCutFreq;
	LSM303DL_WriteData(I2Cx, transmit_buffer[0], transmit_buffer[1]);

	HAL_Delay(25);

	LSM303DL_ReadData(I2Cx, &temp, LSM303DLHC_CTRL_REG2_A, 1);
	if(temp != transmit_buffer[1])
		return LSM303DL_RESULT_ERROR;

	// <---- ------------ Control Register_3 ------------ ---->
	transmit_buffer[0] = LSM303DLHC_CTRL_REG3_A;
	transmit_buffer[1] = 0x00;					//Default
	LSM303DL_WriteData(I2Cx, transmit_buffer[0], transmit_buffer[1]);

	HAL_Delay(25);

	LSM303DL_ReadData(I2Cx, &temp, LSM303DLHC_CTRL_REG3_A, 1);
	if(temp != transmit_buffer[1])
		return LSM303DL_RESULT_ERROR;

	// <---- ------------ Control Register_4 ------------ ---->
	// <---- ------------ FullScale, High-Resolution ------------ ---->
	transmit_buffer[0] = LSM303DLHC_CTRL_REG4_A;
	transmit_buffer[1] = datastruct -> Accel_Scale | datastruct -> Accel_HighResEn;
	LSM303DL_WriteData(I2Cx, transmit_buffer[0], transmit_buffer[1]);

	HAL_Delay(25);

	LSM303DL_ReadData(I2Cx, &temp, LSM303DLHC_CTRL_REG4_A, 1);
	if(temp != transmit_buffer[1])
		return LSM303DL_RESULT_ERROR;

	return LSM303DL_RESULT_OK;
}

// <---- ------------ LSM303DL Read Data ------------ ---->
LSM303DLHC_Accel_Result LSM303DL_ReadData(I2C_HandleTypeDef* I2Cx, uint8_t* buffer, uint8_t addr, uint8_t num)
{
	uint8_t reg = addr | 0x80;

	HAL_I2C_Master_Transmit(I2Cx, I2C_ADDRESS, &reg, 1, 50);
	HAL_I2C_Master_Receive(I2Cx, I2C_ADDRESS, buffer, num, 50);
	
	/* Return OK */
	return LSM303DL_RESULT_OK;
}

// <---- ------------ LSM303DL Write Data ------------ ---->
LSM303DLHC_Accel_Result LSM303DL_WriteData(I2C_HandleTypeDef* I2Cx, uint8_t addr, uint8_t data)
{
    uint8_t buffer[2] = {addr, data};

	HAL_I2C_Master_Transmit(I2Cx, I2C_ADDRESS, buffer, 2, 50);
	
	/* Return OK */
	return LSM303DL_RESULT_OK;
}



