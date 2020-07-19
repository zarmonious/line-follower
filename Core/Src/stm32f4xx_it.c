/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define turn_right TIM3->CCR2
#define turn_left TIM3->CCR1
#define MAX 5000 // 600
#define MAX_SET 140 // 140
#define MIN 0 // 80
#define MIN_ADD 0 // 60
#define DUTY_SCALE_MAX 0.5
#define BLACK 2460
#define SPEED 250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
  static float previous_error = 0.;
  static float integral = 0.;
  static float measured_value = 0.;
  float error = 0.;
  float derivative = 0.;

  float y;
  const float dt = 0.000067;
  const float P = 0.8; // 0.6
  const float I = 0.8; // 0.8
  const float D = 0.0010; // 0.0006

  static uint8_t start = 0;
  static uint8_t last_pos = 1; 		// 0 left, 1 right


  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_FLAG_UPDATE) != RESET)
    {
    	if( opticSensors[0] >= BLACK && opticSensors[1] >= BLACK)
    	{
    		on_line++;
    	}
    	else
    	{
    		not_on_line++;
    	}

    	if (opticSensors[0] + opticSensors[1] != 0)
    	{
    		measured_value = (((float)opticSensors[0]-(float)opticSensors[1])/((float)opticSensors[0]+(float)opticSensors[1])*10000. -120.);
    	}
    	error = 0. - measured_value;					// P
    	integral = integral + error * dt;				// I
    	derivative = (error - previous_error) / dt; 	// D

    	y = P * error + I * integral + D * derivative;

    	previous_error = error;

    	if(start == 1)			// starting point
    	{
			if (last_pos == 0)
			{
			   turn_left = 100;
        	   turn_right = 0;
			}
			else
			{
			   turn_left = 0;
        	   turn_right = 140;
			}

    		if( opticSensors[0] >= 2460 || opticSensors[1] >= 2400)
    		{
    			start = 0;
    		}
    	}
    	else
    	{
    		if( y < 0)
    		{
    			last_pos = 1;
    			turn_left = 0;

    			if (-y >= MIN && -y <= MAX)
    			{
					turn_right = -y;
    			}
    			else if (-y < MIN)
    			{
    				turn_right = -y + MIN_ADD;
    			}
    			else
    			{
					turn_right = MAX_SET;
    			}
    		}
    		else if (y > 0)
    		{
				last_pos = 0;
				turn_right = 0;

				if (y >= MIN && y <= MAX)
    			{
					turn_left = y;
    			}
    			else if (y < MIN)
				{
					turn_right = y + MIN_ADD;
				}
    			else
    			{
    				turn_left = MAX_SET;
    			}
    		}
    		else
    		{
    			turn_left = 0;
    			turn_right = 0;
    		}

    		if( opticSensors[0] < 2460 && opticSensors[1] < 2400) 		// search black line
    		{
    			start = 1;
    		}

    	}

    	__HAL_TIM_CLEAR_IT(&htim1, TIM_FLAG_UPDATE);
    }
  }
  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
 static uint32_t n = 0; 			// Position in TIM8 period (0...TIM8->ARR).
 uint32_t stepPeriod; 				// 0...29
 uint32_t stepPhase;				// 0...9
 uint32_t phase;
 static float dutyScale = 0.3;

 if (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) != RESET)
 {
	 if (__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_UPDATE) != RESET)
	 {
		 stepPeriod = (uint32_t) (((float) n) / (((float) TIM8->ARR) / ((float) STEPS)));
		 phase = (uint32_t) ((float) stepPeriod / (float) SECTIONS);
		 stepPhase = stepPeriod % SECTIONS;

		 if (phase == 0)
		 {
			 TIM1->CCR1 = sectionValues[stepPhase] * dutyScale;					// Asceding array access
			 TIM1->CCR2 = 0;													// Stay zero
			 TIM1->CCR3 = sectionValues[SECTIONS - stepPhase - 1] * dutyScale;  // Desceding array access
		 }
		 else if (phase == 1)
		 {
			 TIM1->CCR1 = sectionValues[SECTIONS - stepPhase - 1] * dutyScale;
			 TIM1->CCR2 = sectionValues[stepPhase] * dutyScale;
			 TIM1->CCR3 = 0;
		 }
		 else if (phase == 2)
		 {
			 TIM1->CCR1 = 0;
			 TIM1->CCR2 = sectionValues[SECTIONS - stepPhase - 1] * dutyScale;
			 TIM1->CCR3 = sectionValues[stepPhase] * dutyScale;
		 }

		 n++;
		 if (n > TIM8->ARR)
		 {
			 n = 0;

			 if (TIM8->ARR > SPEED) // Max speed
			 {
				 TIM8->ARR = TIM8->ARR - 10;
			 }

			 if (dutyScale < DUTY_SCALE_MAX)
			 {
				 dutyScale = dutyScale * 1.025;
				 if (dutyScale > DUTY_SCALE_MAX)
				 {
					 dutyScale = DUTY_SCALE_MAX;
				 }
			 }
		 }

		__HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
	 }
 }
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	static uint32_t inputCapNew = 0;
	uint32_t inputCapOld = 0;
	uint32_t difference = 0;

	  if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC1) != RESET)
	  {
		if (__HAL_TIM_GET_IT_SOURCE(&htim5, TIM_FLAG_CC1) != RESET)
		{
			inputCapOld = inputCapNew;
			inputCapNew = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

			if (inputCapNew > inputCapOld)
			{
				difference = inputCapNew - inputCapOld;
			}
			else if (inputCapNew < inputCapOld)
			{
				difference = 59999 - inputCapOld + inputCapNew;
			}

			revolutions = 9000.0/(float) difference;

		__HAL_TIM_CLEAR_IT(&htim5, TIM_FLAG_CC1);
	    }
	  }
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
