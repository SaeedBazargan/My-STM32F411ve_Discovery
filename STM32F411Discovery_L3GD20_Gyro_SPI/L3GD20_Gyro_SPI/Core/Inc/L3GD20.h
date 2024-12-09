#ifndef __HAL_L3GD20_H__
#define __HAL_L3GD20_H__

#include "stm32f4xx_hal.h"

// <---- ------------ START REGISTER MAPPING ------------ ---->
#define L3GD20_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           0x2D  /* Output Register Z */ 

#define L3GD20_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define L3GD20_INT1_CFG_ADDR          0x30  /* Interrupt 1 configuration Register */
#define L3GD20_INT1_SRC_ADDR          0x31  /* Interrupt 1 source Register */
#define L3GD20_INT1_TSH_XH_ADDR       0x32  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_XL_ADDR       0x33  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_YH_ADDR       0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_YL_ADDR       0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_ZH_ADDR       0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_TSH_ZL_ADDR       0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_DURATION_ADDR     0x38  /* Interrupt 1 DURATION register */

#define I_AM_L3GD20                 ((uint8_t)0xD4)
#define I_AM_L3GD20_TR              ((uint8_t)0xD5)

// <---- ------------ Power Mode Selection ------------ ---->
typedef enum
{
	L3GD20_MODE_POWERDOWN  = (uint8_t)0x00,
	L3GD20_MODE_ACTIVE     = (uint8_t)0x01
}L3GD20_Gyro_PwrTypeDef;

// <---- ------------ Output DataRate Selection ------------ ---->
typedef enum
{
    L3GD20_OUTPUT_DATARATE_1 = ((uint8_t)0x00),
    L3GD20_OUTPUT_DATARATE_2 = ((uint8_t)0x40),
    L3GD20_OUTPUT_DATARATE_3 = ((uint8_t)0x80),
    L3GD20_OUTPUT_DATARATE_4 = ((uint8_t)0xC0)
}L3GD20_Gyro_DataRateTypeDef;

// <---- ------------ Axes Selection ------------ ---->
typedef enum
{
    L3GD20_X_ENABLE     = ((uint8_t)0x02),
    L3GD20_Y_ENABLE     = ((uint8_t)0x01),
    L3GD20_Z_ENABLE     = ((uint8_t)0x04),
    L3GD20_AXES_ENABLE  = ((uint8_t)0x07),
    L3GD20_AXES_DISABLE = ((uint8_t)0x00)
}L3GD20_Gyro_AxesTypeDef;

// <---- ------------ Bandwidth Selection ------------ ---->
typedef enum
{
    L3GD20_BANDWIDTH_1 = ((uint8_t)0x00),
    L3GD20_BANDWIDTH_2 = ((uint8_t)0x10),
    L3GD20_BANDWIDTH_3 = ((uint8_t)0x20),
    L3GD20_BANDWIDTH_4 = ((uint8_t)0x30)
}L3GD20_Gyro_BandwidthTypeDef;

// <---- ------------ Full Scale Selection ------------ ---->
typedef enum
{
    L3GD20_FULLSCALE_250        = ((uint8_t)0x00),
    L3GD20_FULLSCALE_500        = ((uint8_t)0x10),
    L3GD20_FULLSCALE_2000       = ((uint8_t)0x20),
    L3GD20_FULLSCALE_SELECTION  = ((uint8_t)0x30)
}L3GD20_Gyro_ScaleTypeDef;

// <---- ------------ High Pass Filter Status ------------ ---->
typedef enum
{
    L3GD20_HIGHPASSFILTER_DISABLE = ((uint8_t)0x00),
    L3GD20_HIGHPASSFILTER_ENABLE  = ((uint8_t)0x10)
}L3GD20_Gyro_FilterStatusTypeDef;

// <---- ------------ High Pass Filter Mode ------------ ---->
typedef enum
{
    L3GD20_HPM_NORMAL_MODE_RES = ((uint8_t)0x00),
    L3GD20_HPM_REF_SIGNAL      = ((uint8_t)0x10),
    L3GD20_HPM_NORMAL_MODE     = ((uint8_t)0x20),
    L3GD20_HPM_AUTORESET_INT   = ((uint8_t)0x30)
}L3GD20_Gyro_FilterModeTypeDef;

// <---- ------------ High Pass CUT-OFF Frequency ------------ ---->
typedef enum
{
    L3GD20_HPFCF_0 = ((uint8_t)0x00),
    L3GD20_HPFCF_1 = ((uint8_t)0x01),
    L3GD20_HPFCF_2 = ((uint8_t)0x02),
    L3GD20_HPFCF_3 = ((uint8_t)0x03),
    L3GD20_HPFCF_4 = ((uint8_t)0x04),
    L3GD20_HPFCF_5 = ((uint8_t)0x05),
    L3GD20_HPFCF_6 = ((uint8_t)0x06),
    L3GD20_HPFCF_7 = ((uint8_t)0x07),
    L3GD20_HPFCF_8 = ((uint8_t)0x08),
    L3GD20_HPFCF_9 = ((uint8_t)0x09)
}L3GD20_Gyro_FilterCutFreqTypeDef;

// <---- ------------ L3GD20 Response ------------ ---->
typedef enum
{
	L3GD20_RESULT_OK    = ((uint8_t)0x00),
	L3GD20_RESULT_ERROR = ((uint8_t)0X01)
}L3GD20_Gyro_Result;

// <---- ------------ Main L3GD20 Structure ------------ ---->
typedef struct
{
	L3GD20_Gyro_PwrTypeDef              Gyro_PWR;
	L3GD20_Gyro_DataRateTypeDef         Gyro_DataRate;
	L3GD20_Gyro_AxesTypeDef             Gyro_Axes;
	L3GD20_Gyro_BandwidthTypeDef        Gyro_Bandwidth;
    L3GD20_Gyro_ScaleTypeDef            Gyro_Scale;
    L3GD20_Gyro_FilterStatusTypeDef     Gyro_FilterEn;
    L3GD20_Gyro_FilterModeTypeDef       Gyro_FilterMode;
    L3GD20_Gyro_FilterCutFreqTypeDef	Gyro_FilterCutFreq;
}L3GD20TypeDef;

// <---- ------------ Main L3GD20 Functions ------------ ---->
L3GD20_Gyro_Result L3GD20_Init(SPI_HandleTypeDef* SPIx , GPIO_TypeDef *CS_GPIOx, uint16_t CS_PinNumber, L3GD20TypeDef* datastruct);
L3GD20_Gyro_Result L3GD20_ReadData(SPI_HandleTypeDef* SPIx, uint8_t* buffer, uint8_t addr, uint8_t num);
L3GD20_Gyro_Result L3GD20_WriteData(SPI_HandleTypeDef* SPIx, uint8_t addr, uint8_t data);

#endif /*__HAL_L3GD20_H__*/
