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
#include "stdio.h"
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Major					0				// Major version Number
#define Minor					2				// Minor version Number

#define MAX_BLOCK_SIZE          (1024)			//1KB
#define ETX_APP_START_ADDRESS   0x08020000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t BL_Version[2] = {Major, Minor};

uint16_t application_size = 0;
uint16_t application_write_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static int UART_Write_Loop(void);
static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint16_t data_len, bool is_first_block);
static void Firmware_Update(void);
static void goto_application(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	int i = 0;
	for(i = 0; i < len; i++)
	{
		ITM_SendChar(*ptr++);

	}
	return len;
}
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Bootloader v%d:%d Started!!!\n", BL_Version[0], BL_Version[1]);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(2000);

  Firmware_Update();

  // Jump to application
  goto_application();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static int UART_Write_Loop(void)
{
	HAL_StatusTypeDef ex;

	char tx = 'g';
	char rx = '0';
	int ret = 0;
	int count = 0;

	while(1)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		HAL_UART_Transmit(&huart2, (uint8_t *)&tx, 1, HAL_MAX_DELAY);

		ex = HAL_UART_Receive(&huart2, (uint8_t *)&rx, 1, 10);

		if((ex == HAL_OK) && (rx == 'r'))
		{
			//received data
			printf("Firmware Update Started\r\n");
			ret = 1;
			break;
		}

		if(count == 100)
		{
			//received nothing
			printf("No Data Received for Firmware Update\r\n");
			break;
		}
		count++;
		HAL_Delay(20);              //20ms delay
	}

	return ret;
}
//----------------------------------------------------------------------------------------------------
/**
  * @brief Write data to the Application's actual flash location.
  * @param data data to be written
  * @param data_len data length
  * @is_first_block true - if this is first block, false - not first block
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef write_data_to_flash_app(uint8_t *data, uint16_t data_len, bool is_first_block)
{
	HAL_StatusTypeDef ret;

	do
	{
		ret = HAL_FLASH_Unlock();
		if(ret != HAL_OK)
		{
			break;
		}

		//No need to erase every time. Erase only the first time.
//		if(is_first_block)
//		{
//			//Erase the Flash
//			FLASH_EraseInitTypeDef EraseInitStruct;
//			uint32_t SectorError;
//
//			EraseInitStruct.TypeErase   	  	= FLASH_TYPEERASE_SECTORS;
//			EraseInitStruct.Sector		   		= ETX_APP_START_ADDRESS;
//			EraseInitStruct.NbSectors       	= 2;                     //2 Sectors
//
//			printf("Erasing the Flash memory...\r\n");
//			ret = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
//			if(ret != HAL_OK)
//			{
//				break;
//			}
//			application_write_idx = 0;
//		}

		for(int i = 0; i < data_len/2; i++)
		{
			uint16_t halfword_data = data[i * 2] | (data[i * 2 + 1] << 8);

			ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (ETX_APP_START_ADDRESS + application_write_idx), halfword_data);
			if(ret == HAL_OK)
			{
				//update the data count
				application_write_idx += 2;
			}
			else
			{
				printf("Flash Write Error...HALT!!!\r\n");
				break;
			}
		}

		if(ret != HAL_OK)
		{
			break;
		}

		ret = HAL_FLASH_Lock();
		if(ret != HAL_OK)
		{
			break;
		}
	}
	while(false);

	return ret;
}
//----------------------------------------------------------------------------------------------------
/**
  * @brief Check for Firmware Update and update the Firmware
  * @retval None
  */
static void Firmware_Update(void)
{
	HAL_StatusTypeDef ex = HAL_OK;

	uint8_t xx,yy;
	uint8_t x = 'x';
	uint8_t y = 'y';
	uint16_t current_app_size = 0;
	uint16_t i = 0;

	uint8_t block[MAX_BLOCK_SIZE] = { 0 };

	do
	{
		if(UART_Write_Loop() != 0)
		{
			//Sender is ready. Receive the Firmware Size

			// Ask yy
			HAL_UART_Transmit(&huart2, &y, 1, HAL_MAX_DELAY);

			ex = HAL_UART_Receive(&huart2, &yy, 1, 5000);
			if(ex != HAL_OK)
			{
				printf("Get application Size error (yy)...HALT!!!\r\n");
				break;
			}

			// Ask xx
			HAL_UART_Transmit(&huart2, &x, 1, HAL_MAX_DELAY);

			ex = HAL_UART_Receive(&huart2, &xx, 1, 5000);
			if(ex != HAL_OK)
			{
				printf("Get application Size error(XX)...HALT!!!\r\n");
				break;
			}

			application_size = yy | (xx << 8);
			printf("Application Size = %d bytes\r\n", application_size);

			while(1)
			{
				if((i == MAX_BLOCK_SIZE) || (current_app_size >= application_size))
				{
					printf("Received Block[%d]\r\n", current_app_size/MAX_BLOCK_SIZE);

					//write to flash
					ex = write_data_to_flash_app(block, MAX_BLOCK_SIZE, (current_app_size <= MAX_BLOCK_SIZE));
					if(ex != HAL_OK)
					{
						break;
					}

					//clear the memory
					memset(block, 0,MAX_BLOCK_SIZE);
					i = 0;
				}

			if(current_app_size >= application_size)
			{
				//received all data. exit
				ex = HAL_OK;
				break;
			}

			// Ask yy
			HAL_UART_Transmit(&huart2, &y, 1, HAL_MAX_DELAY);

			ex = HAL_UART_Receive(&huart2, &yy, 1, 5000);
			if(ex != HAL_OK)
			{
				printf("Get application data[index:%d] error (yy)...HALT!!!\r\n", i);
				break;
			}

			// Ask xx
			HAL_UART_Transmit(&huart2, &x, 1, HAL_MAX_DELAY);

			ex = HAL_UART_Receive(&huart2, &xx, 1, 5000);
			if(ex != HAL_OK)
			{
				printf("Get application data[index:%d] error(XX)...HALT!!!\r\n", i);
				break;
			}

			//--- Save xxyy in block[i]
			block[i++] = yy;
			block[i++] = xx;
			current_app_size += 2;
			}
		}
	}
	while(false);

	if(ex != HAL_OK)
	{
		while(1);
	}
}
//----------------------------------------------------------------------------------------------------
static void goto_application(void)
{
	printf("Gonna Jump to application\n");

	void(*app_reset_handler) (void) = (void*) (*(volatile uint32_t*)(ETX_APP_START_ADDRESS + 4));

	if(app_reset_handler == (void*)0xFFFFFFFF)
	{
		printf("Invalid Application... HALT!!!\r\n");
		while(1);
	}

	__set_MSP(*(volatile uint32_t*) ETX_APP_START_ADDRESS);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	app_reset_handler();
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
