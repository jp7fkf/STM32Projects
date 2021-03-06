
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void LCD_Init();
int LCD_WriteData(uint8_t data);
void HelloWorld();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10);
  LCD_Init();
  HAL_Delay(10);

  char 	aRxBuffer[2];
  char 	aTxBuffer[2];
  uint8_t aSPITxBuffer[3];
  uint8_t aSPIRxBuffer[3];
  char  rxbuf[2];
  char  txbuf1[] = "Sensor Failed! \r\n";
  char  txbuf2[] = "Sensor Wakeup Successful! \r\n";
  char  txbuf3[] = "Data Received! \r\n";
  char 	str[32];
  HelloWorld();
  char oper = 0;
  char stat = 0;
  uint32_t pres = 0;
  int temp = 0;

  HAL_Delay(60);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  if(oper == 0)
	  {
		  aSPITxBuffer[0] = (0x07<<2) | 0x00; // resister address and read/write bit

		  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,0);
		  HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)aSPITxBuffer,(uint8_t*)aSPIRxBuffer,2,2000);
		  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,1);

		  HAL_UART_Transmit(&huart1, (uint8_t*)aSPIRxBuffer, sizeof(aSPIRxBuffer), 0xFFFF);
		  HAL_UART_Transmit(&huart1, "Data Received!\r\n", sizeof("Data Received!\r\n"), 0xFFFF);
		  HAL_Delay(10);

		  if(aSPIRxBuffer[1] & 0x01)
		  {
			  if(stat > 6)
				  HAL_UART_Transmit(&huart1, (uint8_t *)txbuf1, sizeof(txbuf1), 0xFFFF);
		  }else{
			  aSPITxBuffer[0] = (0x1F<<2) | 0x00; // resister address and read/write bit

			  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,0);
			  HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)aSPITxBuffer,(uint8_t*)aSPIRxBuffer,2,2000);
			  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,1);

			  HAL_UART_Transmit(&huart1, "Data Received2!\r\n", sizeof("Data Received2!\r\n"), 0xFFFF);
			  HAL_UART_Transmit(&huart1, (uint8_t*)aSPIRxBuffer, sizeof(aSPIRxBuffer), 0xFFFF);
			  if(aSPIRxBuffer[1] & 0x01)
			  {
				  HAL_UART_Transmit(&huart1, (uint8_t *)txbuf2, sizeof(txbuf2), 0xFFFF);
				  aSPITxBuffer[0] = (0x03<<2) | 0x02; // resister address and read/write bit
				  aSPITxBuffer[1] = 0x0A; // data
				  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,0);
				  HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)aSPITxBuffer,(uint8_t*)aSPIRxBuffer,2,2000);
				  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,1);
				  oper = 1;
			  }else{
				  HAL_UART_Transmit(&huart1, "not LSB=1!\r\n", sizeof("not LSB=1!\r\n"), 0xFFFF);
				  HAL_UART_Transmit(&huart1, (uint8_t *)txbuf1, sizeof(txbuf1), 0xFFFF);
			  }
		  }
	  }else{
		  aSPITxBuffer[0] = (0x1F<<2) | 0x00; // resister address and read/write bit
		  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,0);
		  HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)aSPITxBuffer,(uint8_t*)aSPIRxBuffer,2,2000);
		  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,1);
		  pres = aSPITxBuffer[1] & 0x03;
		  aSPITxBuffer[0] = (0x20<<2) | 0x00; // resister address and read/write bit
		  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,0);
		  HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)aSPITxBuffer,(uint8_t*)aSPIRxBuffer,3,2000);
		  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,1);

		  pres = (pres << 8) | aSPIRxBuffer[1];
		  pres = (pres << 8) | aSPIRxBuffer[2];
		  pres /= 4;

		  aSPITxBuffer[0] = (0x21<<2) | 0x00; // resister address and read/write bit
		  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,0);
		  HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)aSPITxBuffer,(uint8_t*)aSPIRxBuffer,3,2000);
		  HAL_GPIO_WritePin(GPIOA,SPI_GPIO_Pin,1);

		  temp = ((aSPIRxBuffer[1] & 0x20) << 18) | ((aSPIRxBuffer[1] & 0x1F) << 8) | aSPIRxBuffer[3];

		  sprintf(str, "TEMP: %6.2d Deg.\r\n", temp/20);
		  HAL_UART_Transmit(&huart1, (uint8_t *)str, sizeof(str), 0xFFFF);
		  sprintf(str, "PRES: %6.2d hPa\r\n", pres/100);
		  HAL_UART_Transmit(&huart1, (uint8_t *)str, sizeof(str), 0xFFFF);
	  }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00101D2D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_GPIO_GPIO_Port, SPI_GPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_GPIO_Pin */
  GPIO_InitStruct.Pin = SPI_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_GPIO_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int LCD_WriteInstraction(uint8_t data)
{
    uint8_t buf[] = { 0x00, data };
    int status = HAL_I2C_Master_Transmit(&hi2c1, 0x7c, buf, 2, 1000);
    HAL_Delay(1);
    return status == HAL_OK;
}

// LCD の初期化
void LCD_Init()
{
    LCD_WriteInstraction(0x38);     // Function set
    LCD_WriteInstraction(0x39);     // Function set
    LCD_WriteInstraction(0x14);     // Internal OSC frequency
    LCD_WriteInstraction(0x70);     // Contrast set
    LCD_WriteInstraction(0x56);     // Power/ICON/Contrast set
    LCD_WriteInstraction(0x6c);     // Follower control
    HAL_Delay(200);
    LCD_WriteInstraction(0x38);     // Function set
    LCD_WriteInstraction(0x0c);     // Display ON/OFF control
    LCD_WriteInstraction(0x01);     // Clear Display
    HAL_Delay(1);
}

// DDRAM への書き込み
int LCD_WriteData(uint8_t data)
{
    uint8_t buf[] = { 0x40, data };
    int status = HAL_I2C_Master_Transmit(&hi2c1, 0x7c, buf, 2, 1000);
    HAL_Delay(1);
    return status == HAL_OK;
}

// 文字列の出力
void Puts(const char *p)
{
    for ( ; *p ; p++)
    {
        LCD_WriteData(*p);
    }
}

// Hello, world! の表示
void HelloWorld()
{
	LCD_WriteInstraction(0x80 | 0x00);
    Puts("HAL");
    LCD_WriteInstraction(0x80 | 0x40);  // 2 行目の先頭に移動
    Puts("world!");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
