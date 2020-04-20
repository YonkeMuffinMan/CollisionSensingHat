/*
 * File: motor.h
 * Purpose: Declares all functions and structs pertaining to the setup and
 *          controlling of the vibrating motor disc. The vibration intensity
 *          is controlled using PWM on a GPIOB pin and TIM3.
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f0xx_hal.h"

// PWM pin, prescalar, auto-reload value, and the four warning thresholds
typedef struct motor {
  uint8_t pin_number;
  uint16_t pwm_prescalar;
  uint16_t pwm_arr;
	uint32_t thresholds[4];   // thresholds vibration changes at high vibration intensity to lowest (off)
} MOTOR;

// Motor setup and startup functions
void MOTOR_Setup(MOTOR *motor);
void MOTOR_Start(void);

// Motor speen manipulation functions
void MOTOR_SetDutyCycle(float prcnt);
void MOTOR_SetVibrationIntensity(uint16_t distance);

void configGPIOB_AF1(uint8_t x);

#endif /* __MOTOR_H */
