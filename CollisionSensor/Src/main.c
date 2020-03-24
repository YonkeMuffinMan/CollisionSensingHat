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

// LED Pins on GPIOC
#define RED_LED 6
#define BLUE_LED 7
#define ORANGE_LED 8
#define GREEN_LED 9

// Motor Pins
#define MOTOR1_B 4 // PB4, TIM3 channel 1


void SystemClock_Config(void);
void configGPIOB_AF1(uint8_t x);
void configGPIOC_AF0(uint8_t x);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	// Enable GPIOB clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM 3 clock
	
	// Configure pins
	configGPIOB_AF1(MOTOR1_B);
	
	uint32_t arr = 10000;
	
	// Configure TIM3 to trigger UEV at 800 Hz, every 1250 us
	TIM3->PSC = 0;	// 8MHz timer clock -> 125ns counter
	TIM3->ARR = arr;	// Reset at 10000 for 1250 us
	
	// Use PWM
	// Set CC1S to output
	TIM3->CCMR1 &= ~TIM_CCMR1_CC1S_Msk;
	// OC1M to PWM Mode 1
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk);
	TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos);
	// Enable output compare preload
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE_Msk;
	// Set output enable for channel 1
	TIM3->CCER |= TIM_CCER_CC1E_Msk;
	// Set CCR to 75% of ARR value
	TIM3->CCR1 = arr;
	
	// Enable/start TIM3
	TIM3->CR1 &= ~(1 << 4);	// upcounter
	TIM3->CR1 |= TIM_CR1_CEN_Msk;	// Counter enabled

  while (1)
  {

  }
}

/*
 * Configure GPIOB pin
 * Pass in the pin number, x, of the pin on PBx
 * Configures pin to alternate function mode, push-pull output,
 * low-speed, no pull-up/down resistors, and AF1
 */
void configGPIOB_AF1(uint8_t x) {
	// Set to Alternate function mode
	GPIOB->MODER &= ~(1 << (2*x));
	GPIOB->MODER |= (1 << ((2*x)+1));
	// Set to Push-pull
	GPIOB->OTYPER &= ~(1 << x);
	// Set to Low speed
	GPIOB->OSPEEDR &= ~((1 << (2*x)) | (1 << ((2*x)+1)));
	// Set to no pull-up/down
	GPIOB->PUPDR &= ~((1 << (2*x)) | (1 << ((2*x)+1)));
	// Set alternate functon to AF1, TIM3_CH
	GPIOB->AFR[0] &= ~(0xF << (4*x));
	GPIOB->AFR[0] |= (0x1 << (4*x));
}

/*
 * Configure GPIOC pin
 * Pass in the pin number, x, of the pin on PCx
 * Configures pin to alternate function mode, push-pull output,
 * low-speed, no pull-up/down resistors, and AF0
 */
void configGPIOC_AF0(uint8_t x) {
	// Set to Alternate function mode
	GPIOC->MODER &= ~(1 << (2*x));
	GPIOC->MODER |= (1 << ((2*x)+1));
	// Set to Push-pull
	GPIOC->OTYPER &= ~(1 << x);
	// Set to Low speed
	GPIOC->OSPEEDR &= ~((1 << (2*x)) | (1 << ((2*x)+1)));
	// Set to no pull-up/down
	GPIOC->PUPDR &= ~((1 << (2*x)) | (1 << ((2*x)+1)));
	// Set alternate functon to AF0, TIM3_CH
	GPIOC->AFR[0] &= ~(0xF << (4*x));
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
