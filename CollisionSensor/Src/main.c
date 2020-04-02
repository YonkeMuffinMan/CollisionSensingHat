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
#include "motor.h"
#include "ultrasonicSensorUart.h"

/*
 * USART3 Pins:
 *	TX: PC4, PB10, PC10
 *	RX: PC5, PB11, PC11
 *	CHOSEN: TX PB10, RX PB11
 *	        AF4      AF4    
 */
#define TX_B 10
#define RX_B 11

// Motor Pins
#define MOTOR1_B 4 // PB4, TIM3 channel 1

// Distance thresholds for LEDs in milimeteres
#define RED_LED_THRESHOLD 0
#define ORANGE_LED_THRESHOLD 300 // 1 feet
#define BLUE_LED_THRESHOLD 950 // 3 feet
#define GREEN_LED_THRESHOLD 1900 // 6 feet
#define NO_LED_THRESHOLD 3500 // 12 feet

// LED Pins on GPIOC
#define RED_LED 6
#define BLUE_LED 7
#define ORANGE_LED 8
#define GREEN_LED 9

void SystemClock_Config(void);

void configGPIOC_AF0(uint8_t x);
void configGPIOC_output(uint8_t pin);

void setWarnings(void);
void setLEDs(uint16_t distance);
void setVibrationIntensity(uint16_t distance);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;	// Enable GPIOC clock
	
	// initialize LEDs
	configGPIOC_output(RED_LED);
	configGPIOC_output(GREEN_LED);
	configGPIOC_output(BLUE_LED);
	configGPIOC_output(ORANGE_LED);
	
	// Set up motor on GPIOB and TIM3
	// pin_number, pwm_prescalar, pwm_arr
	MOTOR motor = { MOTOR1_B, 0, 10000 };
	setupMotor(motor);
	startMotor();
	
	// uart_tx, uart_rx, uart_baud_rate
	SENSOR sensor = { TX_B, RX_B, 9600 };
	setupSensor(sensor);

  while (1)
  {
		HAL_Delay(100);
		getReading();
		setWarnings();
  }
}

/*
 * Generic GPIOC configuration function
 * Pass in the pin number, x, of the GPIO on PCx
 * Configures pin to general-pupose output mode, push-pull output,
 * low-speed, and no pull-up/down resistors
 */
void configGPIOC_output(uint8_t pin) {
	uint32_t shift2x = 2*pin;
	uint32_t shift2xp1 = shift2x+1;
	
	// Set General Pupose Output
	GPIOC->MODER |= (1 << shift2x);
	GPIOC->MODER &= ~(1 << shift2xp1);
	// Set to Push-pull
	GPIOC->OTYPER &= ~(1 << pin);
	// Set to Low speed
	GPIOC->OSPEEDR &= ~((1 << shift2x) | (1 << shift2xp1));
	// Set to no pull-up/down
	GPIOC->PUPDR &= ~((1 << shift2x) | (1 << shift2xp1));
}

void setWarnings() {
	while (newValue == 0);
	
	uint16_t distance = distanceValue; // in millimeters	
	setLEDs(distance);
	setVibrationIntensity(distance);
}

void setLEDs(uint16_t distance) {
	// turn on LEDs
	GPIOC->BSRR = (((distance >= GREEN_LED_THRESHOLD) & (distance < NO_LED_THRESHOLD)) << GREEN_LED) |
									(((distance >= BLUE_LED_THRESHOLD) & (distance < GREEN_LED_THRESHOLD)) << BLUE_LED) |
									(((distance >= ORANGE_LED_THRESHOLD) & (distance < BLUE_LED_THRESHOLD)) << ORANGE_LED) |
									(((distance >= RED_LED_THRESHOLD) & (distance < ORANGE_LED_THRESHOLD)) << RED_LED);
		// turn off LEDs
	GPIOC->BRR = (((distance < GREEN_LED_THRESHOLD) | (distance > NO_LED_THRESHOLD) << GREEN_LED) |
								 ((distance < BLUE_LED_THRESHOLD) | (distance > GREEN_LED_THRESHOLD)) << BLUE_LED) |
								 (((distance < ORANGE_LED_THRESHOLD) | (distance > BLUE_LED_THRESHOLD)) << ORANGE_LED) |
								 ((distance > ORANGE_LED_THRESHOLD) << RED_LED);
}

void setVibrationIntensity(uint16_t distance) {
	uint8_t motorSpeed = ((distance < ORANGE_LED_THRESHOLD) << 3) |
											 ((distance < BLUE_LED_THRESHOLD) << 2) |
											 ((distance < GREEN_LED_THRESHOLD) << 1) |
											 ((distance < NO_LED_THRESHOLD));
	
	switch (motorSpeed) {
		case 0xF:
			setDutyCycle(1); break;
		case 0x7:
			setDutyCycle(0.75); break;
		case 0x3:
			setDutyCycle(0.50); break;
		case 0x1:
		default:
			setDutyCycle(0);
	}
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
