/*
 * File: ultrasonicSensorUart.c
 * Purpose: Defines all functions pertaining to the setup and communication
 *          with the US-100 Ultrasonic Distance Sensor. All communication
 *          is via USART3 using GPIOB pins.
 */
#include "ultrasonicSensorUart.h"

// initialize the data recieved to 0
volatile SENSOR_Values sensorValues = { 0, 0, 0, 0, 0 };
volatile uint8_t rangeMeasurement = 1;

/*
 * Setups the USART3 subsystem and GPIO pins
 */
void SENSOR_Setup(SENSOR *sensor) {
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; //Enable USART3 clock
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock
  
  configPinB_AF4(sensor->uart_tx);
  configPinB_AF4(sensor->uart_rx);
  
  SENSOR_SetBaudRate(sensor->uart_baud_rate);
  // enable transmitter and reciever hardware
  USART3->CR1 |= USART_CR1_RE_Msk | USART_CR1_TE_Msk;
  // enable Recieved Register Not Empty interrupt
  USART3->CR1 |= USART_CR1_RXNEIE_Msk;
  // enable peripheral
  USART3->CR1 |= USART_CR1_UE_Msk;
  
	// enable the interrupt and set it to highest priority
  NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 0);
}

/*
 * Set baud rate of USART3 to rate using the HCLK
 */
void SENSOR_SetBaudRate(uint32_t rate) {
  uint32_t fclk = HAL_RCC_GetHCLKFreq();
  uint16_t usartdiv = fclk / rate;
  USART3->BRR = usartdiv;
}

/*
 * Send a request for a reading to the sensor
 */
void SENSOR_GetReading() {
  // wait until transmit data register is empty. bit 7 will be set when empty
  while ((USART3->ISR & (1 << 7)) == 0) {}
  
	rangeMeasurement = 1;
  // Transmit data register is now empty, write new char to send
  USART3->TDR = 0x55;
}

/*
 * Send a request for a temperature reading to the sensor
 */
void SENSOR_GetTempReading(void) {
	// wait until transmit data register is empty. bit 7 will be set when empty
  while ((USART3->ISR & (1 << 7)) == 0) {}
  
	rangeMeasurement = 0;
  // Transmit data register is now empty, write new char to send
  USART3->TDR = 0x50;
}

/*
 * USART3 or 4 interrupt request handler
 * Wait for data to be received, then process it
 */
void USART3_4_IRQHandler(void) {
	// wait for distance data to be received
  if (((USART3->ISR & USART_ISR_RXNE_Msk) >> USART_ISR_RXNE_Pos) != 1) return;
	
	if (rangeMeasurement) SENSOR_RecvDistance();
	else SENSOR_RecvTemperature();
}

/*
 * Get the 16 bit distance value
 */
void SENSOR_RecvDistance() {
	// if two values have already been recieved, reset everything
  if (sensorValues.recieved == 2) {
    sensorValues.recieved = 0; 
    sensorValues.distance = 0;
    sensorValues.new_value = 0;
  }
	// read the value and parse it out
  uint8_t val = USART3->RDR;
  switch (sensorValues.recieved++) {
    case 0:
      sensorValues.distance |= val << 8; break;
    case 1:
      sensorValues.distance |= val; break;
  };
  
	// if two 8 bit values were received, the 16 bit number is ready
  if (sensorValues.recieved == 2) sensorValues.new_value = 1;
}

/*
 * Get the 8 bit temperature value
 */
void SENSOR_RecvTemperature() {
	if (sensorValues.temp_recieved == 1) {
    sensorValues.temp_recieved = 0; 
    sensorValues.temperature = 0;
    sensorValues.new_temp_value = 0;
  }
	
	sensorValues.temperature = USART3->RDR;
	sensorValues.temp_recieved++;
	sensorValues.new_temp_value = 1;
}

/*
 * GPIOB Pin configuration function
 * Pass in the pin number, x
 * Configures pin to alternate function mode, push-pull output,
 * low-speed, no pull-up/down resistors, and AF4
 */
void configPinB_AF4(uint8_t x) {
  // Set to Alternate function mode, 10
  GPIOB->MODER &= ~(1 << (2*x));
  GPIOB->MODER |= (1 << ((2*x)+1));
  // Set to Push-pull
  GPIOB->OTYPER &= ~(1 << x);
  // Set to Low speed
  GPIOB->OSPEEDR &= ~((1 << (2*x)) | (1 << ((2*x)+1)));
  // Set to no pull-up/down
  GPIOB->PUPDR &= ~((1 << (2*x)) | (1 << ((2*x)+1)));
  // Set alternate functon to AF4, USART3 0100
  if (x < 8) {  // use AFR low register
    GPIOB->AFR[0] &= ~(0xF << (4*x));
    GPIOB->AFR[0] |= (0x4 << (4*x));
  }
  else {  // use AFR high register
    GPIOB->AFR[1] &= ~(0xF << (4*(x-8)));
    GPIOB->AFR[1] |= (0x4 << (4*(x-8)));
  }
}
