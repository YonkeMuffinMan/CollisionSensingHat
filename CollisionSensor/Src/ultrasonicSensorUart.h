/*
 * File: ultrasonicSensorUart.h
 * Purpose: Declares all functions and structs pertaining to the setup and
 *          communication with the US-100 Ultrasonic Distance Sensor. All
 *          communication is via USART3 using GPIOB pins.
 */
#ifndef __ULTRASONIC_UART_H
#define __ULTRASONIC_UART_H

#include "stm32f0xx_hal.h"

// Holds the UART information
typedef struct {
  uint8_t uart_tx;
  uint8_t uart_rx;
  uint32_t uart_baud_rate;
} SENSOR;

// Holds the data recieved information
typedef struct {
	uint8_t recieved;
	uint16_t distance;
	uint8_t new_value;
} SENSOR_Values;

// Define a volatile extern so SENSOR_GetReading can change the values and main can see them
extern volatile SENSOR_Values sensorValues;

void SENSOR_Setup(SENSOR *sensor);

void SENSOR_SetBaudRate(uint32_t x);
void SENSOR_GetReading(void);

void configPinB_AF4(uint8_t x);

#endif /* __ULTRASONIC_UARTUART_H */
