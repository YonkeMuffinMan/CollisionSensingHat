#include "motor.h"

uint32_t arr;

void setupMotor(MOTOR motor) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	// Enable GPIOB clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIM 3 clock
	
	// Configure pins
	configGPIOB_AF1(motor.pin_number);
	
	arr = motor.pwm_arr;
	
	// Configure TIM3 to trigger UEV at 800 Hz, every 1250 us
	TIM3->PSC = motor.pwm_prescalar;	// 8MHz timer clock -> 125ns counter
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
	// Set CCR to 0
	TIM3->CCR1 = 0;
	
	// Enable/start TIM3
	TIM3->CR1 &= ~(1 << 4);	// upcounter
}

void startMotor() {
	TIM3->CR1 |= TIM_CR1_CEN_Msk;	// Counter enabled
}

void setDutyCycle(float prcnt) {
	TIM3->CCR1 = arr * prcnt;
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
