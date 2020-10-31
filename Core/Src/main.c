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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "usbd_cdc.h"
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  	// TX init
	struct nRF24_Handle TX_n24_H;
	TX_n24_H.hspi = &hspi1;
	TX_n24_H.CSN_GPIO_Port = csn_GPIO_Port;
	TX_n24_H.CSN_Pin = csn_Pin;
	TX_n24_H.CE_GPIO_Port = ce_GPIO_Port;
	TX_n24_H.CE_Pin = ce_Pin;
	TX_n24_H.IQR_GPIO_Port = irq_GPIO_Port;
	TX_n24_H.IQR_Pin = irq_Pin;

	nRF24_QS(TX_n24_H, 1);
	nRF24_FlushRX(TX_n24_H);
	nRF24_FlushTX(TX_n24_H);

	// RX init
	struct nRF24_Handle RX_n24_H;
	RX_n24_H.hspi = &hspi2;
	RX_n24_H.CSN_GPIO_Port = csn_RX_GPIO_Port;
	RX_n24_H.CSN_Pin = csn_RX_Pin;
	RX_n24_H.CE_GPIO_Port = ce_RX_GPIO_Port;
	RX_n24_H.CE_Pin = ce_RX_Pin;
	RX_n24_H.IQR_GPIO_Port = irq_RX_GPIO_Port;
	RX_n24_H.IQR_Pin = irq_RX_Pin;

	nRF24_QS(RX_n24_H, 0);
	nRF24_FlushRX(RX_n24_H);
	nRF24_FlushTX(RX_n24_H);
	uint8_t rx_data = 0x00;
	uint8_t Data = 0x14;

	  // Channel Address management
	uint8_t pipe_addr[5] = {0x78, 0x78, 0x78, 0x78, 0x78};		// Set channel address as 0x7878787878
	nRF24_SetDataPipeADDR(TX_n24_H, 0x0A, pipe_addr);	//PTX RX pipe 0
	nRF24_SetDataPipeADDR(TX_n24_H, 0x10, pipe_addr);	//PTX TX_ADD
	nRF24_SetDataPipeADDR(RX_n24_H, 0x0B, pipe_addr);	//PRX RX pipe 1

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // TX part (SPI 1)

	  nRF24_RegWrite(TX_n24_H, 0x07, 0x70);		// Clear interrupt
	  Data = Data % 255;
	  Data++;
	  uint8_t W_TX_Payload = 0xA0;

	  nRF24_TX_WritePayload(TX_n24_H, &Data, sizeof(Data), W_TX_Payload);
	  nRF24_TX_SendPayload(TX_n24_H, 0);

	  uint8_t RX_Empty = (nRF24_RegRead(RX_n24_H, 0x17));	// Data in FIFO
	  nRF24_FlushTX(TX_n24_H);
	  // RX part (SPI 2)
	  nRF24_RX_ReadPayload(RX_n24_H, &rx_data, sizeof(rx_data));

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
