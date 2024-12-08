#include "L3GD20.h"

// <---- ------------ Variables ------------ ---->
uint16_t CS_PinNumber = 0x0000;
GPIO_TypeDef *CS_GPIO;
// <---- ------------ Main L3GD20 Functions ------------ ---->
// <---- ------------ L3GD20 Initialize ------------ ---->
L3GD20_Gyro_Result L3GD20_Init(SPI_HandleTypeDef* SPIx , GPIO_TypeDef *CS_GPIOx, uint16_t CS_PinNumber, L3GD20TypeDef* datastruct)
{
    
}
// <---- ------------ L3GD20 Read Data ------------ ---->
L3GD20_Gyro_Result L3GD20_ReadData(SPI_HandleTypeDef* SPIx, uint8_t* buffer, uint8_t addr, uint8_t num)
{
	uint8_t reg = addr | 0x80;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPIx, &reg, 1, 10);
	HAL_SPI_Receive(SPIx, buffer, num, 10);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	
	/* Return OK */
	return L3GD20_RESULT_OK;    
}
// <---- ------------ L3GD20 Write Data ------------ ---->
L3GD20_Gyro_Result L3GD20_WriteData(SPI_HandleTypeDef* SPIx, uint8_t addr, uint8_t data)
{}




