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
#include "lcd.h"

/*
 * USART3 Pins:
 *  TX: PC4, PB10, PC10
 *  RX: PC5, PB11, PC11
 *  CHOSEN: TX PB10, RX PB11
 *          AF4      AF4    
 */
#define TX_B 10
#define RX_B 11

// SPI Pins for LCD
#define SCK_B 13	// system clock
#define MOSI_B 15 // send data
#define DC_B 5		// mode select
#define RST_B 6		// reset
#define SCE_B 7		// chip select

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

void configGPIOC_output(uint8_t pin);
void timerSetup(void);

void setWarnings(void);
void setLEDs(uint16_t distance);
void displayTemperature(void);

/*
 * Setup the motr, sensor, LEDs, LCD screen, and the 100ms timer interrupt
 * All processing is done through the timer interrrupt
 */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  // Enable GPIOC clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
  
  // initialize LEDs
  configGPIOC_output(RED_LED);
  configGPIOC_output(GREEN_LED);
  configGPIOC_output(BLUE_LED);
  configGPIOC_output(ORANGE_LED);
  
  // Set up motor on GPIOB and TIM3
  MOTOR motor = { MOTOR1_B, 0, 10000, {ORANGE_LED_THRESHOLD, BLUE_LED_THRESHOLD, GREEN_LED_THRESHOLD, NO_LED_THRESHOLD} }; // pin_number, pwm_prescalar, pwm_arr, thresholds (high to low)
  MOTOR_Setup(&motor);
  MOTOR_Start();
  
	// Set up UART Ultrasonic Distance sensor
  SENSOR sensor = { TX_B, RX_B, 9600 }; // uart_tx, uart_rx, uart_baud_rate
  SENSOR_Setup(&sensor);
	
	// Set up LCD screen
	LCD screen = { SCK_B, MOSI_B, SCE_B, DC_B, RST_B };
	LCD_Setup(&screen);
	LCD_DistanceSetup();
	
	// setup and start the 100ms timer
	timerSetup();
	
  while (1)
  {
		
  }
}

/*
 * Setup the 100ms timer to get readings and set the proper warnings
 */
void timerSetup() {
	// Configure TIM2 to trigger UEV at 10 Hz, every 100 ms
	TIM2->PSC = (8000-1);	// 1kHz timer clock -> 1ms counter
	TIM2->ARR = 100;
	
	// Configure TIM2 to interrupt on UEV
	TIM2->CR1 &= ~(1 << 1);	// UDIS bit to 0 means UEV enabled
	TIM2->DIER |= 1;	// Update interrupt enabled
	
	// Enable/start TIM2
	TIM2->CR1 &= ~(1 << 4);	// upcounter
	TIM2->CR1 |= 1;	// Counter enabled
	
	// Configure TIM2 interrupt handler and enable in NVIC
	// Set its priority lower than the Ultrasonic Distance UART's
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 3);
}

/*
 * TIM2 Interrupt Handler: Get Ultrasonic distance readings and set the warnings
 */
void TIM2_IRQHandler(void) {
	SENSOR_GetReading();
	setWarnings();
	HAL_Delay(10);
	SENSOR_GetTempReading();
	displayTemperature();
	
	TIM2->SR &= ~(1);	// clear update interrupt flag
}

/*
 * Wait for new temperature value, then display it
 */
void displayTemperature() {
  while (sensorValues.new_temp_value == 0);
	uint8_t temp = sensorValues.temperature - 45;
	uint16_t far = ((temp * 9)/5) + 32;
	LCD_PrintTempMeasurement(far, "F", 1, temp, "C", 1);
}

/*
 * Wait for a new distance value, then set the warnings
 */
void setWarnings() {
  while (sensorValues.new_value == 0);
  uint16_t distance = sensorValues.distance; // in millimeters  
  setLEDs(distance);
  //MOTOR_SetVibrationIntensity(distance);
	LCD_PrintMeasurement(distance, "mm", 2);
}

/*
 * Turn on and off the LEDs based on the distance thresholds
 */
void setLEDs(uint16_t distance) {
  // turn on LEDs
  GPIOC->BSRR = (((distance >= GREEN_LED_THRESHOLD) & (distance < NO_LED_THRESHOLD)) << GREEN_LED) |
                  (((distance >= BLUE_LED_THRESHOLD) & (distance < GREEN_LED_THRESHOLD)) << BLUE_LED) |
                  (((distance >= ORANGE_LED_THRESHOLD) & (distance < BLUE_LED_THRESHOLD)) << ORANGE_LED) |
                  (((distance >= RED_LED_THRESHOLD) & (distance < ORANGE_LED_THRESHOLD)) << RED_LED);
    // turn off LEDs
  GPIOC->BRR = (((distance < GREEN_LED_THRESHOLD) | (distance >= NO_LED_THRESHOLD)) << GREEN_LED) |
                 (((distance < BLUE_LED_THRESHOLD) | (distance >= GREEN_LED_THRESHOLD)) << BLUE_LED) |
                 (((distance < ORANGE_LED_THRESHOLD) | (distance >= BLUE_LED_THRESHOLD)) << ORANGE_LED) |
                 ((distance >= ORANGE_LED_THRESHOLD) << RED_LED);
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
