/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "nRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//void reg_write(uint8_t address, uint8_t value)
//{
//	uint8_t W_address = address + 0x20;	// Add 32 to register address to write
//	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_RESET);	// Select slave
//	HAL_SPI_Transmit(&hspi1, &W_address, 1, 1);
//	HAL_SPI_Transmit(&hspi1, &value, 1, 1);
//	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);
//};
//
//uint8_t reg_read(uint8_t address)
//{
//	uint8_t data;
// 	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_RESET);	//Select slave
//	HAL_SPI_Transmit(&hspi1, &address, 1, 1);
//	HAL_SPI_Receive(&hspi1, &data, 1, 1);
//	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);
//	return data;
//};
//
//void read_Pipe_address(uint8_t pipe)
//{
//	uint8_t pipe_buffer[5];
//	uint8_t Pipe_address = pipe + 0x0A;
// 	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_RESET);	//Select slave
// 	HAL_SPI_Transmit(&hspi1, &pipe, 1, 1);
// 	HAL_SPI_Receive(&hspi1, pipe_buffer, 5, 10);
//	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);
//	CDC_Transmit_FS(pipe_buffer, sizeof(pipe_buffer));
//};

void print_n24_registers()
{
	uint8_t reg_value;
	for(int reg = 0; reg < 10; reg++)
	{
		reg_value = reg_read(reg);
		HAL_Delay(10);
		CDC_Transmit_FS(&reg_value, sizeof(reg_value));
		CDC_Transmit_FS((uint8_t*)("\n"), sizeof("\n"));
	}

	for(int pipe = 0; pipe < 7; pipe++)
	{
		read_Pipe_address(pipe);
		HAL_Delay(10);
		CDC_Transmit_FS((uint8_t*)("\n"), sizeof("\n"));
	}

	for(int reg = 17; reg < 24; reg++)
	{
		reg_value = reg_read(reg);
		HAL_Delay(10);
		CDC_Transmit_FS(&reg_value, sizeof(reg_value));
		CDC_Transmit_FS((uint8_t*)("\n"), sizeof("\n"));
	}

}

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(ce_GPIO_Port, ce_Pin, GPIO_PIN_RESET);
  	HAL_Delay(20);
  	HAL_GPIO_WritePin(csn_GPIO_Port, csn_Pin, GPIO_PIN_SET);

  	// RESET all register
//  	reg_write(0x00, 0x08);
//	reg_write(0x04, 0x03);
//	reg_write(0x06, 0x0E);



	// CONFIG TX
  	reg_write(0x00, 0x0A); // PTX, Power ON, CRC, 1 byte
	reg_write(0x00, 0x0A); // PTX, Power ON, CRC, 1 byte
	reg_write(0x04, 0xFF); // 15 Retry, 4ms wait
	reg_write(0x06, 0x08); // -18dBm, 2Mbps



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint8_t addr_1 = reg_read(0x00);
	  uint8_t addr_2 = reg_read(0x04);
	  uint8_t addr_3 = reg_read(0x06);
	  //print_n24_registers();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
