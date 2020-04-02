#ifndef __ULTRASONIC_UART_H
#define __ULTRASONIC_UART_H

#include "stm32f0xx_hal.h"

typedef struct {
	uint8_t uart_tx;
	uint8_t uart_rx;
	uint32_t uart_baud_rate;
} SENSOR;

extern volatile uint8_t valuesRecieved;
extern volatile uint16_t distanceValue;
extern volatile uint8_t newValue;

void setupSensor(SENSOR sensor);
void configPinB_AF4(uint8_t x);

void setUSART3BaudRate(uint32_t x);
void getReading(void);

#endif /* __ULTRASONIC_UARTUART_H */
