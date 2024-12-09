/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "L3GD20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
L3GD20TypeDef L3GD20;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t isInitialized = 0x00;
volatile uint8_t Gyro_Buffer[10];
volatile int16_t Gyro_RawData[3];






#define AVERAGE_WINDOW_SIZE                  ((uint32_t) 10u)
#define CALIBRATION_BUFFER_LENGTH            ((uint32_t) 2000u)
#define L3GD20_SENSITIVITY    ((float)0.07)


typedef enum
{
	L3GD20_DATA_NOT_READY,
	L3GD20_DATA_READY,
}L3GD20_DataReadyFlagType;

static L3GD20_DataReadyFlagType dataReadyFlag=L3GD20_DATA_READY;

typedef enum
{
	L3GD20_collect_calibration_samples,
	L3GD20_process_calibration_samples,
	L3GD20_calibrated,
}L3GD20_caliStateType;

static L3GD20_caliStateType currentcalistate=L3GD20_collect_calibration_samples;



typedef enum
{
	L3GD20_fisrt,
	L3GD20_second,
	L3GD20_finaly
}L3GD20_StateType;
static L3GD20_StateType currentState=L3GD20_fisrt;


static float angleRate_x=0;
static float angleRate_y=0;
static float angleRate_z=0;

static int32_t offset_x=0;
static int32_t offset_y=0;
static int32_t offset_z=0;

static float Noise_X = 0;
static float Noise_Y = 0;
static float Noise_Z = 0;

static float Angle_X = 0;
static float Angle_Y = 0;
static float Angle_Z = 0;

static float LastAngleRate_X = 0;
static float LastAngleRate_Y = 0;
static float LastAngleRate_Z = 0;

static int32_t TempNoise_X = 0;
static int32_t TempNoise_Y = 0;
static int32_t TempNoise_Z = 0;


volatile int16_t Raw_x=0;
volatile int16_t Raw_y=0;
volatile int16_t Raw_z=0;




volatile static uint32_t caliCounter = 0;


static int16_t calibrationBuffer_X[CALIBRATION_BUFFER_LENGTH];
static int16_t calibrationBuffer_Y[CALIBRATION_BUFFER_LENGTH];
static int16_t calibrationBuffer_Z[CALIBRATION_BUFFER_LENGTH];

static uint8_t spiTxBuf[2];
static uint8_t spiRxBuf[7];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void Gyro_L3GD20_Init(void);
void Gyro_L3GD20_ReadData(void);

void L3GD20_loop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Gyro_L3GD20_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  Gyro_L3GD20_ReadData();
	  L3GD20_loop();
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// <---- ------------ Gyyroscope Initialize ------------ ---->
void Gyro_L3GD20_Init(void)
{
	L3GD20.Gyro_PWR 			= L3GD20_MODE_ACTIVE;
	L3GD20.Gyro_DataRate 		= L3GD20_OUTPUT_DATARATE_4;
	L3GD20.Gyro_Axes 			= L3GD20_AXES_ENABLE;
	L3GD20.Gyro_Bandwidth 		= L3GD20_BANDWIDTH_4;
	L3GD20.Gyro_Scale 			= L3GD20_FULLSCALE_2000;
	L3GD20.Gyro_FilterEn		= L3GD20_HIGHPASSFILTER_ENABLE;
	L3GD20.Gyro_FilterMode 		= L3GD20_HPM_NORMAL_MODE_RES;
	L3GD20.Gyro_FilterCutFreq 	= L3GD20_HPFCF_0;

	isInitialized = L3GD20_Init(&hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, &L3GD20);
	if(isInitialized != L3GD20_RESULT_OK)
		Error_Handler();
}
// <---- ------------ Gyyroscope Read Data ------------ ---->
void Gyro_L3GD20_ReadData(void)
{
	L3GD20_ReadData(&hspi1, Gyro_Buffer, L3GD20_OUT_X_L_ADDR, 6);

	Gyro_RawData[0] = ((int16_t)(Gyro_Buffer[0] | Gyro_Buffer[1] << 8) * 0.7);
	Gyro_RawData[1] = ((int16_t)(Gyro_Buffer[2] | Gyro_Buffer[3] << 8) * 0.7);
	Gyro_RawData[2] = ((int16_t)(Gyro_Buffer[4] | Gyro_Buffer[5] << 8) * 0.7);
}

void L3GD20_loop(void)
{
//	volatile int16_t Raw_x=0;
//	volatile int16_t Raw_y=0;
//	volatile int16_t Raw_z=0;

	float difftime=0;

	int16_t averageWindow_X[AVERAGE_WINDOW_SIZE] = {0};
	int16_t averageWindow_Y[AVERAGE_WINDOW_SIZE] = {0};
	int16_t averageWindow_Z[AVERAGE_WINDOW_SIZE] = {0};

	uint32_t windowPosition = 0;
	int32_t tempSum_X = 0;
	int32_t tempSum_Y = 0;
	int32_t tempSum_Z = 0;

	switch(currentState)
	{
		//---------------------------------------------------------------------------
		//data
		case(L3GD20_fisrt):
			if(dataReadyFlag==L3GD20_DATA_READY)
			{
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
				spiTxBuf[0]=0x28|0x80;
				HAL_SPI_Transmit(&hspi1,spiTxBuf,1,50);
				HAL_SPI_Receive(&hspi1,&spiRxBuf[1],1,50);
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
				spiTxBuf[0]=0x29|0x80;
				HAL_SPI_Transmit(&hspi1,spiTxBuf,1,50);
				HAL_SPI_Receive(&hspi1,&spiRxBuf[2],1,50);
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
				spiTxBuf[0]=0x2a|0x80;
				HAL_SPI_Transmit(&hspi1,spiTxBuf,1,50);
				HAL_SPI_Receive(&hspi1,&spiRxBuf[3],1,50);
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
				spiTxBuf[0]=0x2b|0x80;
				HAL_SPI_Transmit(&hspi1,spiTxBuf,1,50);
				HAL_SPI_Receive(&hspi1,&spiRxBuf[4],1,50);
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
				spiTxBuf[0]=0x2c|0x80;
				HAL_SPI_Transmit(&hspi1,spiTxBuf,1,50);
				HAL_SPI_Receive(&hspi1,&spiRxBuf[5],1,50);
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
				spiTxBuf[0]=0x2d|0x80;
				HAL_SPI_Transmit(&hspi1,spiTxBuf,1,50);
				HAL_SPI_Receive(&hspi1,&spiRxBuf[6],1,50);
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

				currentState=L3GD20_second;
				dataReadyFlag=L3GD20_DATA_NOT_READY;
			}
			else
			{}
		break;
		//-----------------------------------------------------------------------------------
		//varibla
		case(L3GD20_second):
			Raw_x=(spiRxBuf[2]<<8)|spiRxBuf[1];
			Raw_y=(spiRxBuf[4]<<8)|spiRxBuf[3];
			Raw_z=(spiRxBuf[6]<<8)|spiRxBuf[5];

//			if(currentcalistate==L3GD20_calibrated)
//			{
//				angleRate_x=(float) (Raw_x - (offset_x))*L3GD20_SENSITIVITY;
//				angleRate_y=(float) (Raw_y - (offset_y))*L3GD20_SENSITIVITY;
//				angleRate_z=(float) (Raw_z - (offset_z))*L3GD20_SENSITIVITY;
//				difftime=0.003f;
//
//				if((angleRate_x>Noise_X)||(angleRate_x<-Noise_X))
//				{
//					Angle_X+=((angleRate_x+LastAngleRate_X)*difftime)/(2.0f);
//					LastAngleRate_X=angleRate_x;
//				}
//				else
//				{}
//				if((angleRate_y>Noise_Y)||(angleRate_y<-Noise_Y))
//				{
//					Angle_Y+=((angleRate_y+LastAngleRate_Y)*difftime)/(2.0f);
//					LastAngleRate_Y=angleRate_y;
//				}
//				else
//				{}
//				if((angleRate_z>Noise_Z)||(angleRate_z<-Noise_Z))
//				{
//					Angle_Z+=((angleRate_z+LastAngleRate_Z)*difftime)/(2.0f);
//					LastAngleRate_Z=angleRate_z;
//				}
//				else
//				{}
//			}
//			else
//			{
//				switch(currentcalistate)
//				{
//					//---------------------------------------------------------------------------------------------------------
//					case(L3GD20_collect_calibration_samples):
//						calibrationBuffer_X[caliCounter]=Raw_x;
//						calibrationBuffer_Y[caliCounter]=Raw_y;
//						calibrationBuffer_Z[caliCounter]=Raw_z;
//						caliCounter++;
//
//						if(caliCounter>=CALIBRATION_BUFFER_LENGTH)
//						{
//							caliCounter=0;
//
//							currentcalistate=L3GD20_process_calibration_samples;
//						}
//						else
//						{}
//					break;
//					//----------------------------------------------------------------------------------------------------------
//					case(L3GD20_process_calibration_samples):
//						for(uint32_t idx=0; idx<CALIBRATION_BUFFER_LENGTH;idx++)
//						{
//							tempSum_X=tempSum_X-averageWindow_X[windowPosition]+calibrationBuffer_X[idx];
//							tempSum_Y=tempSum_Y-averageWindow_Y[windowPosition]+calibrationBuffer_Y[idx];
//							tempSum_Z=tempSum_Z-averageWindow_Z[windowPosition]+calibrationBuffer_Z[idx];
//
//							averageWindow_X[windowPosition]=calibrationBuffer_X[idx];
//							averageWindow_Y[windowPosition]=calibrationBuffer_Y[idx];
//							averageWindow_Z[windowPosition]=calibrationBuffer_Z[idx];
//
//							offset_x=tempSum_X/(int32_t)AVERAGE_WINDOW_SIZE;
//							offset_y=tempSum_Y/(int32_t)AVERAGE_WINDOW_SIZE;
//							offset_z=tempSum_Z/(int32_t)AVERAGE_WINDOW_SIZE;
//
//							windowPosition++;
//
//							if(windowPosition>=AVERAGE_WINDOW_SIZE)
//							{
//								windowPosition=0;
//							}
//							else
//							{}
//
//						}
//						for(uint32_t idx=0;idx<CALIBRATION_BUFFER_LENGTH;idx++)
//						{
//							if(((int32_t)calibrationBuffer_X[idx]-offset_x)>TempNoise_X)
//							{
//								TempNoise_X=(int32_t)calibrationBuffer_X[idx]-offset_x;
//							}
//							else if(((int32_t)calibrationBuffer_X[idx]-offset_x)<-TempNoise_X)
//							{
//								TempNoise_X=-((int32_t)calibrationBuffer_X[idx]-offset_x);
//							}
//
//							if(((int32_t)calibrationBuffer_Y[idx]-offset_y)>TempNoise_Y)
//							{
//								TempNoise_Y=(int32_t)calibrationBuffer_Y[idx]-offset_y;
//							}
//							else if(((int32_t)calibrationBuffer_Y[idx]-offset_y)<-TempNoise_Y)
//							{
//								TempNoise_Y=-((int32_t)calibrationBuffer_Y[idx]-offset_y);
//							}
//
//							if(((int32_t)calibrationBuffer_Z[idx]-offset_z)>TempNoise_Z)
//							{
//								TempNoise_Z=(int32_t)calibrationBuffer_Z[idx]-offset_z;
//							}
//							else if(((int32_t)calibrationBuffer_Z[idx]-offset_z)<-TempNoise_Z)
//							{
//								TempNoise_Z=-((int32_t)calibrationBuffer_Z[idx]-offset_z);
//							}
//						}
//
//						Noise_X=(float)TempNoise_X*L3GD20_SENSITIVITY;
//						Noise_Y=(float)TempNoise_Y*L3GD20_SENSITIVITY;
//						Noise_Z=(float)TempNoise_Z*L3GD20_SENSITIVITY;
//
//						currentcalistate=L3GD20_calibrated;
//					break;
//
//					case(L3GD20_calibrated):
//					break;
//
//					default:
//					break;
//				}
//
//		}
		currentState=L3GD20_fisrt;
		dataReadyFlag=L3GD20_DATA_READY;
		break;

		default:
		break;

	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
