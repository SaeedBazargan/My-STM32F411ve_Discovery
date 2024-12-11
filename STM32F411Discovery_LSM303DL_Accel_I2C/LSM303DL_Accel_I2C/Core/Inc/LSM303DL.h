#ifndef __HAL_LSM303DL_H__
#define __HAL_LSM303DL_H__

#include "stm32f4xx_hal.h"

// <---- ------------ START REGISTER MAPPING ------------ ---->
#define ACC_I2C_ADDRESS                      0x32

#define LSM303DLHC_WHO_AM_I_ADDR             0x0F  /* device identification register */
#define LSM303DLHC_CTRL_REG1_A               0x20  /* Control register 1 acceleration */
#define LSM303DLHC_CTRL_REG2_A               0x21  /* Control register 2 acceleration */
#define LSM303DLHC_CTRL_REG3_A               0x22  /* Control register 3 acceleration */
#define LSM303DLHC_CTRL_REG4_A               0x23  /* Control register 4 acceleration */
#define LSM303DLHC_CTRL_REG5_A               0x24  /* Control register 5 acceleration */
#define LSM303DLHC_CTRL_REG6_A               0x25  /* Control register 6 acceleration */
#define LSM303DLHC_REFERENCE_A               0x26  /* Reference register acceleration */
#define LSM303DLHC_STATUS_REG_A              0x27  /* Status register acceleration */
#define LSM303DLHC_OUT_X_L_A                 0x28  /* Output Register X acceleration */
#define LSM303DLHC_OUT_X_H_A                 0x29  /* Output Register X acceleration */
#define LSM303DLHC_OUT_Y_L_A                 0x2A  /* Output Register Y acceleration */
#define LSM303DLHC_OUT_Y_H_A                 0x2B  /* Output Register Y acceleration */
#define LSM303DLHC_OUT_Z_L_A                 0x2C  /* Output Register Z acceleration */
#define LSM303DLHC_OUT_Z_H_A                 0x2D  /* Output Register Z acceleration */
#define LSM303DLHC_FIFO_CTRL_REG_A           0x2E  /* Fifo control Register acceleration */
#define LSM303DLHC_FIFO_SRC_REG_A            0x2F  /* Fifo src Register acceleration */

#define LSM303DLHC_INT1_CFG_A                0x30  /* Interrupt 1 configuration Register acceleration */
#define LSM303DLHC_INT1_SOURCE_A             0x31  /* Interrupt 1 source Register acceleration */
#define LSM303DLHC_INT1_THS_A                0x32  /* Interrupt 1 Threshold register acceleration */
#define LSM303DLHC_INT1_DURATION_A           0x33  /* Interrupt 1 DURATION register acceleration */

#define LSM303DLHC_INT2_CFG_A                0x34  /* Interrupt 2 configuration Register acceleration */
#define LSM303DLHC_INT2_SOURCE_A             0x35  /* Interrupt 2 source Register acceleration */
#define LSM303DLHC_INT2_THS_A                0x36  /* Interrupt 2 Threshold register acceleration */
#define LSM303DLHC_INT2_DURATION_A           0x37  /* Interrupt 2 DURATION register acceleration */

#define LSM303DLHC_CLICK_CFG_A               0x38  /* Click configuration Register acceleration */
#define LSM303DLHC_CLICK_SOURCE_A            0x39  /* Click 2 source Register acceleration */
#define LSM303DLHC_CLICK_THS_A               0x3A  /* Click 2 Threshold register acceleration */

#define LSM303DLHC_TIME_LIMIT_A              0x3B  /* Time Limit Register acceleration */
#define LSM303DLHC_TIME_LATENCY_A            0x3C  /* Time Latency Register acceleration */
#define LSM303DLHC_TIME_WINDOW_A             0x3D  /* Time window register acceleration */

#define I_AM_LMS303DLHC                   	 ((uint8_t)0x33)

// <---- ------------ Power Mode Selection ------------ ---->
typedef enum
{
	LSM303DLHC_NORMAL_MODE  	= ((uint8_t)0x00),
	LSM303DLHC_LOWPOWER_MODE    = ((uint8_t)0x08)
}LSM303DLHC_Accel_PwrTypeDef;

// <---- ------------ Output DataRate Selection ------------ ---->
typedef enum
{

	LSM303DLHC_ODR_1_HZ 		= ((uint8_t)0x10),  /*!< Output Data Rate = 1 Hz */
	LSM303DLHC_ODR_10_HZ 		= ((uint8_t)0x20),  /*!< Output Data Rate = 10 Hz */
	LSM303DLHC_ODR_25_HZ 		= ((uint8_t)0x30),  /*!< Output Data Rate = 25 Hz */
	LSM303DLHC_ODR_50_HZ 		= ((uint8_t)0x40),  /*!< Output Data Rate = 50 Hz */
	LSM303DLHC_ODR_100_HZ 		= ((uint8_t)0x50),  /*!< Output Data Rate = 100 Hz */
	LSM303DLHC_ODR_200_HZ 		= ((uint8_t)0x60),  /*!< Output Data Rate = 200 Hz */
	LSM303DLHC_ODR_400_HZ 		= ((uint8_t)0x70),  /*!< Output Data Rate = 400 Hz */
	LSM303DLHC_ODR_1620_HZ_LP 	= ((uint8_t)0x80),  /*!< Output Data Rate = 1620 Hz only in Low Power Mode */
	LSM303DLHC_ODR_1344_HZ 		= ((uint8_t)0x90)   /*!< Output Data Rate = 1344 Hz in Normal mode and 5376 Hz in Low Power Mode */
}LSM303DLHC_Accel_DataRateTypeDef;

// <---- ------------ Axes Selection ------------ ---->
typedef enum
{
	LSM303DLHC_X_ENABLE 		= ((uint8_t)0x01),
	LSM303DLHC_Y_ENABLE 		= ((uint8_t)0x02),
	LSM303DLHC_Z_ENABLE 		= ((uint8_t)0x04),
	LSM303DLHC_AXES_ENABLE  	= ((uint8_t)0x07),
	LSM303DLHC_AXES_DISABLE 	= ((uint8_t)0x00)
}LSM303DLHC_Accel_AxesTypeDef;

// <---- ------------ High Resolution Enable ------------ ---->
typedef enum
{
	LSM303DLHC_HR_ENABLE 		= ((uint8_t)0x08),
	LSM303DLHC_HR_DISABLE		= ((uint8_t)0x00)
}LSM303DLHC_Accel_HREnTypeDef;

// <---- ------------ Full Scale Selection ------------ ---->
typedef enum
{
	LSM303DLHC_FULLSCALE_2G 	= ((uint8_t)0x00),  /*!< ±2 g */
	LSM303DLHC_FULLSCALE_4G 	= ((uint8_t)0x10),  /*!< ±4 g */
	LSM303DLHC_FULLSCALE_8G 	= ((uint8_t)0x20),  /*!< ±8 g */
	LSM303DLHC_FULLSCALE_16G	= ((uint8_t)0x30)  /*!< ±16 g */	
}LSM303DLHC_Accel_ScaleTypeDef;

// <---- ------------ High Pass Filter Status ------------ ---->
typedef enum
{
	LSM303DLHC_HIGHPASSFILTER_DISABLE = ((uint8_t)0x00),
	LSM303DLHC_HIGHPASSFILTER_ENABLE  = ((uint8_t)0x08)
}LSM303DLHC_Accel_FilterStatusTypeDef;

// <---- ------------ High Pass Filter Mode ------------ ---->
typedef enum
{
	LSM303DLHC_HPM_NORMAL_MODE_RES = ((uint8_t)0x00),
	LSM303DLHC_HPM_REF_SIGNAL      = ((uint8_t)0x40),
	LSM303DLHC_HPM_NORMAL_MODE     = ((uint8_t)0x80),
	LSM303DLHC_HPM_AUTORESET_INT   = ((uint8_t)0xC0)
}LSM303DLHC_Accel_FilterModeTypeDef;

// <---- ------------ High Pass CUT-OFF Frequency ------------ ---->
typedef enum
{
	LSM303DLHC_HPFCF_8 		= ((uint8_t)0x00),
	LSM303DLHC_HPFCF_16		= ((uint8_t)0x10),
	LSM303DLHC_HPFCF_32		= ((uint8_t)0x20),
	LSM303DLHC_HPFCF_64		= ((uint8_t)0x30)
}LSM303DLHC_Accel_FilterCutFreqTypeDef;

// <---- ------------ LSM303DL Response ------------ ---->
typedef enum
{
	LSM303DL_RESULT_OK    = ((uint8_t)0x00),
	LSM303DL_RESULT_ERROR = ((uint8_t)0X01)
}LSM303DLHC_Accel_Result;

// <---- ------------ Main LSM303DL Structure ------------ ---->
typedef struct
{
	LSM303DLHC_Accel_PwrTypeDef              Accel_PWR;
	LSM303DLHC_Accel_DataRateTypeDef         Accel_DataRate;
	LSM303DLHC_Accel_AxesTypeDef             Accel_Axes;
	LSM303DLHC_Accel_HREnTypeDef	         Accel_HighResEn;
    LSM303DLHC_Accel_ScaleTypeDef            Accel_Scale;
    LSM303DLHC_Accel_FilterModeTypeDef       Accel_FilterMode;
    LSM303DLHC_Accel_FilterCutFreqTypeDef	 Accel_FilterCutFreq;
}LSM303DLTypeDef;

// <---- ------------ Main LSM303DL Functions ------------ ---->
LSM303DLHC_Accel_Result LSM303DL_Init(I2C_HandleTypeDef* I2Cx , LSM303DLTypeDef* datastruct);
LSM303DLHC_Accel_Result LSM303DL_ReadData(I2C_HandleTypeDef* I2Cx, uint8_t* buffer, uint8_t addr, uint8_t num);
LSM303DLHC_Accel_Result LSM303DL_WriteData(I2C_HandleTypeDef* I2Cx, uint8_t addr, uint8_t data);
#endif /*__HAL_LSM303DL_H__*/
