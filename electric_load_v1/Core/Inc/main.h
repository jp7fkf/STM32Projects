/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define shunt1_Pin GPIO_PIN_0
#define shunt1_GPIO_Port GPIOA
#define shunt2_Pin GPIO_PIN_1
#define shunt2_GPIO_Port GPIOA
#define shunt3_Pin GPIO_PIN_2
#define shunt3_GPIO_Port GPIOA
#define shunt4_Pin GPIO_PIN_3
#define shunt4_GPIO_Port GPIOA
#define load_voltage_Pin GPIO_PIN_4
#define load_voltage_GPIO_Port GPIOA
#define thermo_sensor_Pin GPIO_PIN_5
#define thermo_sensor_GPIO_Port GPIOA
#define enable_led_Pin GPIO_PIN_6
#define enable_led_GPIO_Port GPIOA
#define enable_sw_Pin GPIO_PIN_7
#define enable_sw_GPIO_Port GPIOA
#define io_button1_Pin GPIO_PIN_0
#define io_button1_GPIO_Port GPIOB
#define io_button2_Pin GPIO_PIN_1
#define io_button2_GPIO_Port GPIOB
#define lcd_data5_Pin GPIO_PIN_11
#define lcd_data5_GPIO_Port GPIOA
#define lcd_data6_Pin GPIO_PIN_12
#define lcd_data6_GPIO_Port GPIOA
#define lcd_data7_Pin GPIO_PIN_13
#define lcd_data7_GPIO_Port GPIOA
#define lcd_data8_Pin GPIO_PIN_14
#define lcd_data8_GPIO_Port GPIOA
#define lcd_rs_Pin GPIO_PIN_3
#define lcd_rs_GPIO_Port GPIOB
#define lcd_ena_Pin GPIO_PIN_4
#define lcd_ena_GPIO_Port GPIOB
#define lcd_rw_Pin GPIO_PIN_5
#define lcd_rw_GPIO_Port GPIOB
#define renc_a_Pin GPIO_PIN_6
#define renc_a_GPIO_Port GPIOB
#define renc_b_Pin GPIO_PIN_7
#define renc_b_GPIO_Port GPIOB
#define beeper_Pin GPIO_PIN_8
#define beeper_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
