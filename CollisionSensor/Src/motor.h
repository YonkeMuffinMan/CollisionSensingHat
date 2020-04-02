#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f0xx_hal.h"

typedef struct motor {
	uint8_t pin_number;
	uint16_t pwm_prescalar;
	uint16_t pwm_arr;
} MOTOR;

void setupMotor(MOTOR motor);
void startMotor(void);
void configGPIOB_AF1(uint8_t x);

void setDutyCycle(float prcnt);

#endif /* __MOTOR_H */
