
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
#include "lcd.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MED_VALUE 10
#define FWD_SLOPE 0.034602076
#define FWD_INTERCEPT 43.979238754
#define REF_SLOPE 0.034843206
#define REF_INTERCEPT 45.12195122

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int median_value(int *c);
double mean_value(double *c);

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
  MX_DMA_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(50);
  LcdInit();
  HAL_Delay(10);

  LcdPuts((char *)"POWER+SWR  METER");
  HAL_Delay(10);
  LcdXy(0,1);
  HAL_Delay(10);
  LcdPuts((char *)"     JP7FKF     ");
  HAL_Delay(1000);
  LcdCls();

  int i = 0;
//  int fwd_buf[MED_VALUE], ref_buf[MED_VALUE];
  int fwd,ref;
  double fwd_watt, ref_watt;
  double fwd_sqrt, ref_sqrt;
  double swr;
  char str[10];

  enum{ ADC_BUFFER_LENGTH = 1024 };
  uint16_t g_ADCBuffer[ADC_BUFFER_LENGTH];

  memset(g_ADCBuffer, 0, sizeof(g_ADCBuffer));
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)g_ADCBuffer, (uint32_t)ADC_BUFFER_LENGTH);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  fwd = g_ADCBuffer[0];
	  ref = g_ADCBuffer[1];

//	  fwd_buf[i] = (int)fwd;
//	  fwd = median_value(fwd_buf);
//	  ref_buf[i] = (int)ref;
//	  ref = median_value(ref_buf);

//	  fwd_buf[i] = (double)fwd_watt;
//	  fwd_watt = mean_value(fwd_buf);
//	  ref_buf[i] = (double)ref_watt;
//	  ref_watt = mean_value(ref_buf);

	  i++;
	  if(i>=MED_VALUE)
		  i=0;

	fwd_watt = fwd * FWD_SLOPE - FWD_INTERCEPT; //volt -> dBm
	ref_watt = ref * REF_SLOPE - REF_INTERCEPT;

	fwd_watt = pow(10, fwd_watt/10);  //dBm -> mW
	ref_watt = pow(10, ref_watt/10);

	fwd_sqrt = fwd_watt;
	ref_sqrt = ref_watt;

	if(fwd_watt + ref_watt < 0.001)
		swr = -1;
	else
		swr = 1;

	if(fwd_watt < 1)
	{
		// mW to uW
		fwd_watt *= 1000;
		sprintf(str, "%6.2fuW",fwd_watt);
	}else if(fwd_watt > 1000){
		// mW to W
		fwd_watt /= 1000;
		sprintf(str, "%7.1fW",fwd_watt);
	}else{
		sprintf(str, "%6.2fmW",fwd_watt);
	}
	LcdXy(0,0);
	LcdPuts(str);
	LcdXy(8,0);
	LcdPuts("/");

	if(ref_watt < 1)
	{
		// mW to uW
		ref_watt *= 1000;
		sprintf(str, "%5.1fuW",ref_watt);
	}else if(ref_watt > 1000){
		// mW to W
		ref_watt /= 1000;
		sprintf(str, "%6.1fW",ref_watt);
	}else{
		sprintf(str, "%5.1fmW",ref_watt);
	}
	LcdPuts(str);


	if(swr == 1)
	{
		//calc swr
		fwd_sqrt = sqrt(fwd_sqrt);
		ref_sqrt = sqrt(ref_sqrt);

		swr = (fwd_sqrt + ref_sqrt)/(fwd_sqrt - ref_sqrt);
//		if(swr > 9.9)
//			swr = 9.99;
		sprintf(str, "SWR: %5.2f",swr);
	}else{
		sprintf(str, "Standby");
	}
//	sprintf(str,"%4d / %4d",fwd, ref);
	LcdXy(0,1);
	LcdPuts(str);
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_Pin|RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E_Pin|DB5_Pin|DB6_Pin|DB7_Pin 
                          |DB8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin RW_Pin */
  GPIO_InitStruct.Pin = RS_Pin|RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : E_Pin DB5_Pin DB6_Pin DB7_Pin 
                           DB8_Pin */
  GPIO_InitStruct.Pin = E_Pin|DB5_Pin|DB6_Pin|DB7_Pin 
                          |DB8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int median_value(int *c)
{
    int i, j;
    int med[MED_VALUE];
    double buf;

    for(i=0; i<MED_VALUE; i++)
    {
    	med[i] = c[i];
    }

    for (j=0; j<MED_VALUE-1; j++) {
        for (i=0; i<MED_VALUE-1; i++) {
            if (med[i+1] < med[i]) {
                buf = med[i+1];
                med[i+1] = med[i];
                med[i] = buf;
            }
        }
    }
    return (int)med[MED_VALUE/2];
}

double mean_value(double *c)
{
    int i;
    double sum = 0;

    for(i=0; i<MED_VALUE; i++)
    {
    	sum += c[i];
    }

    return sum/MED_VALUE;
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
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
