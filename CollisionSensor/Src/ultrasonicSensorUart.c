#include "ultrasonicSensorUart.h"

volatile uint8_t valuesRecieved = 0;
volatile uint16_t distanceValue = 0;
volatile uint8_t newValue = 0;

void setupSensor(SENSOR sensor) {
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; //Enable USART3 clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	// Enable GPIOB clock
	
	configPinB_AF4(sensor.uart_tx);
	configPinB_AF4(sensor.uart_rx);
	
	setUSART3BaudRate(sensor.uart_baud_rate);
	// enable transmitter and reciever hardware
	USART3->CR1 |= USART_CR1_RE_Msk | USART_CR1_TE_Msk;
	// enable Recieved Register Not Empty interrupt
	USART3->CR1 |= USART_CR1_RXNEIE_Msk;
	// enable peripheral
	USART3->CR1 |= USART_CR1_UE_Msk;
	
	NVIC_EnableIRQ(USART3_4_IRQn);
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
	if (x < 8) {	// use AFR low register
		GPIOB->AFR[0] &= ~(0xF << (4*x));
		GPIOB->AFR[0] |= (0x4 << (4*x));
	}
	else {	// use AFR high register
		GPIOB->AFR[1] &= ~(0xF << (4*(x-8)));
		GPIOB->AFR[1] |= (0x4 << (4*(x-8)));
	}
}

/*
 * Set baud rate of USART3 to rate using the HCLK
 */
void setUSART3BaudRate(uint32_t rate) {
	uint32_t fclk = HAL_RCC_GetHCLKFreq();
	uint16_t usartdiv = fclk / rate;
	USART3->BRR = usartdiv;
}

/*
 * Send char on USART3
 */
void getReading() {
	// wait until transmit data register is empty. bit 7 will be set when empty
	while ((USART3->ISR & (1 << 7)) == 0) {}
	
	// Transmit data register is now empty, write new char to send
	USART3->TDR = 0x55;
}

/*
 * USART3 or 4 interrupt request handler
 */
void USART3_4_IRQHandler(void) {
	if (((USART3->ISR & USART_ISR_RXNE_Msk) >> USART_ISR_RXNE_Pos) != 1) return;
	if (valuesRecieved == 2) {
		valuesRecieved = 0; 
		distanceValue = 0;
		newValue = 0;
	}
	uint8_t val = USART3->RDR;
	switch (valuesRecieved++) {
		case 0:
			distanceValue |= val << 8; break;
		case 1:
			distanceValue |= val; break;
	};
	
	if (valuesRecieved == 2) newValue = 1;
}
