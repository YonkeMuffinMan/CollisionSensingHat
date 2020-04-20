/*
 * File: lcd.c
 * Purpose: Defines all functions pertaining to the setup and communication
 *          with the Nokia 5110 LCD Screen. All communication is via SPI2
 *          using GPIOB pins.
 */
#include "lcd.h"

LCD *thisScreen;

/*
 * Setups up the needed SPI2 and general IO pins and the SPI2 subsystem
 */
void LCD_Setup(LCD *screen) {
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; //Enable SPI2 clock
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock
  
	thisScreen = screen;
	
	// configure SPI2 Pins
  configPinB_AF0(thisScreen->data_in);
	configPinB_AF0(thisScreen->sclk);
	// configure general IO pins
	configGPIOB_output(thisScreen->chip_select);
	configGPIOB_output(thisScreen->mode_select);
	configGPIOB_output(thisScreen->reset);
	
	// send a reset pulse to reset LCD screen 
	GPIOB->BRR = (1 << thisScreen->reset);
	HAL_Delay(100);
	GPIOB->BSRR = (1 << thisScreen->chip_select) | (1 << thisScreen->reset);
  
	// Configure SPI
	SPI2->CR1 &= ~SPI_CR1_BR_Msk;
	SPI2->CR1 |= 0x3 << SPI_CR1_BR_Pos; // Fpclk / 16
	SPI2->CR1 &= ~SPI_CR1_CPHA_Msk; // first clock transition is first data capture edge
	SPI2->CR1 &= ~SPI_CR1_CPOL_Msk; // clock 0 when idle
	SPI2->CR1 |= SPI_CR1_BIDIOE_Msk; // Output enabled
	SPI2->CR1 &= ~SPI_CR1_LSBFIRST_Msk; // MSB transmitted first
	SPI2->CR1 |= SPI_CR1_MSTR_Msk; // Master configuration
	
	SPI2->CR2 |= 0x7 << SPI_CR2_DS_Pos; // 8 bit data messages
	SPI2->CR2 |= SPI_CR2_SSOE_Msk;
	
	SPI2->CR1 |= SPI_CR1_SPE_Msk; // SPI enbale
	
	// Send the setup commands and clear the display
	LCD_Startup();
	LCD_ClearDisplay();
}


/*
 * Sends a byte to the LCD screen via SPI2
 */
void LCD_SendByte(char c) {
	// wait until the transmit buffer is empty
	while((SPI2->SR & SPI_SR_TXE_Msk) != SPI_SR_TXE_Msk);
	
	// set the chip select line low (its active low)
	GPIOB->BRR = (1 << thisScreen->chip_select);
	
	*(uint8_t *)&(SPI2->DR) = c; // Make sure to do only an 8bit write, otherwise SPI assumes datapacking

	// wait until the transmission is over
	while((SPI2->SR & SPI_SR_BSY_Msk) == SPI_SR_BSY_Msk);
	
	// deslect the chip
	GPIOB->BSRR = (1 << thisScreen->chip_select);
}

/*
 * Send a command byte to the LCD
 */
void LCD_SendCommand(char c) {
	// set the D/C line low to indicate a command is being set
	GPIOB->BRR = (1 << thisScreen->mode_select);
	
	LCD_SendByte(c);
}

/*
 * Send a data byte to the LCD. This will set a column of 8 bits on the LCD
 */
void LCD_SendData(char c) {
	// set the D/C line high to indicate a data byte is being set
	GPIOB->BSRR = (1 << thisScreen->mode_select);
	
	LCD_SendByte(c);
}

/*
 * Send a sequence of startup commands
 */
void LCD_Startup() {
	LCD_SendCommand(COMMAND_EXTENDED_INSTRUCTION); // Lets LCD know commands from extended instr set coming
	LCD_SendCommand(0xbf); // Set the contrast
	LCD_SendCommand(0x04); // Set the temperature coefficient
	LCD_SendCommand(0x14); // Set the LCD Bias mode
	LCD_SendCommand(0x20); // Return back to basic instructions
	LCD_SendCommand(0x0c); // Set LCD Display to normal mode
}

/*
 * clear the display by going through each column/row and clearing the 8bits
 */
void LCD_ClearDisplay() {
	// reset cursor to top left corner
	LCD_Reset();
	
	for (int i = 0; i < (84*48/8); i++) {
		LCD_SendData(0x00);
	}
}

/*
 * clear row y starting at column x
 */
void LCD_ClearRow(uint8_t y, uint8_t x) {
	LCD_SetY(y);
	LCD_SetX(x);
	
	for (int i = 0; i < (84-x); i++) {
		LCD_SendData(0x00);
	}
}

/*
 * Turn on inverse mode - black background and white text
 */
void LCD_InverseDisplay() {
	LCD_SendCommand(COMMAND_DISPLAY_INVERSE);
}

/*
 * set display to normal mode - white background and black text
 */
void LCD_NormalDisplay() {
	LCD_SendCommand(COMMAND_DISPLAY_NORMAL);
}

/*
 * reset cursor to left hand side
 */
void LCD_ResetX() {
	LCD_SetX(0x0);
}

/*
 * reset cursor to top row
 */
void LCD_ResetY() {
	LCD_SetY(0x0);
}

/*
 * reset to top left corner
 */
void LCD_Reset() {
	LCD_ResetX();
	LCD_ResetY();
}

/*
 * set the x column to between 0 <= x <= 83
 */
void LCD_SetX(uint8_t x) {
	if (x > 83) return;
	LCD_SendCommand(0x80 | x);
}

/*
 * set the y row to between 0 <= y <= 5
 */
void LCD_SetY(uint8_t y) {
	if (y > 5) return;
	LCD_SendCommand(0x40 | y);
}

/*
 * Print a character on the screen
 */
void LCD_PrintCharacter(char c) {
	char index = c - ' '; // the array starts at the space character
	
	// if character is next to edge, add a blank column
	if (ascii_to_lcd[index][0] != 0x00) {
		LCD_SendData(0x00);
	}
	// send the 5 columns that make up the character
	for (int i = 0; i < 5; i++) {
		LCD_SendData(ascii_to_lcd[c-' '][i]);
	}
	// if character is next to edge, add a blank column
	if (ascii_to_lcd[index][4] != 0x00) {
		LCD_SendData(0x00);
	}
}

/*
 * Print a string on the screen
 */
void LCD_PrintString(char* str, int sz) {
	// print each character of the string
	for (int i = 0; i < sz; i++) {
		LCD_PrintCharacter(str[i]);
	}
}

/*
 * Print all the defined characters
 */
void LCD_PrintAll() {
	for (int i = ' '; i < (' ' + (sizeof(ascii_to_lcd)/sizeof(ascii_to_lcd[0]))); i++)
		LCD_PrintCharacter(i);
}

/*
 * center text in the row. the row must be set before calling this function
 */
void LCD_PrintStringCentered(char* str, uint8_t sz) {
	uint8_t numCol = sz*5; // each character takes up about 5 columns
	
	// empty columns are added before and after characters that use all 5 column, so take them into account
	for (int i = 0; i < sz; i ++) {
		uint8_t *c = ascii_to_lcd[str[i]-' '];
		if (c[0] != 0x00) numCol++;
		if (c[4] != 0x00) numCol++;
	}
	
	LCD_SetX((84-numCol)/2);
	LCD_PrintString(str, sz);
}

/*
 * Display "DISTANCE:" on the second row of the LCD screen, centered
 */
void LCD_DistanceSetup() {
	LCD_ClearDisplay(); // clear display
	LCD_SetY(2);
	LCD_PrintStringCentered("DISTANCE:", 9);
}

/*
 * Print the distance measurement centered on the third row
 */
void LCD_PrintMeasurement(uint16_t dist, char* units, uint8_t units_sz) {
	char distStr[32]; // distance measurements are only 16 bits, so a little extra for units
	
	// clear the previous distance measurement
	LCD_ClearRow(3, 0);
	LCD_SetY(3);
	
	// check if the distance is out of range of sensor, which is about 4500mm
	if (dist > 4500) {
		LCD_PrintStringCentered("OUT OF RANGE", 12);
		return;
	}
	
	// convert the measurment to a string
	uint8_t sz = uintToStr(distStr, dist);
	
	// add the units to the string
	for (int i = 0, j = sz; i < units_sz; i++, j++) {
		distStr[j] = units[i];
	}
	
	LCD_PrintStringCentered(distStr, sz+units_sz);
}

/*
 * convert an unsigned int to a string and return the number of characters
 */ 
uint8_t uintToStr(char *buf, uint16_t dist) {
	uint8_t charsWritten = 0;
	// get the character for each digit
	do {
		buf[charsWritten++] = '0' + (dist % 10);
		dist /= 10;
	} while (dist > 0);
	
	// need to reverse the order in the array
	for (int i = 0, j = charsWritten-1; i < j; i++, j--) {
		char temp = buf[i];
		buf[i] = buf[j];
		buf[j] = temp;
	}
	
	return charsWritten;
}

/*
 * Generic GPIOB configuration function
 * Pass in the pin number, x, of the GPIO on PBx
 * Configures pin to general-pupose output mode, push-pull output,
 * low-speed, and no pull-up/down resistors
 */
void configGPIOB_output(uint8_t pin) {
  uint32_t shift2x = 2*pin;
  uint32_t shift2xp1 = shift2x+1;
  
  // Set General Pupose Output
  GPIOB->MODER |= (1 << shift2x);
  GPIOB->MODER &= ~(1 << shift2xp1);
  // Set to Push-pull
  GPIOB->OTYPER &= ~(1 << pin);
  // Set to Low speed
  GPIOB->OSPEEDR &= ~((1 << shift2x) | (1 << shift2xp1));
  // Set to no pull-up/down
  GPIOB->PUPDR &= ~((1 << shift2x) | (1 << shift2xp1));
}

/*
 * GPIOB Pin configuration function
 * Pass in the pin number, x
 * Configures pin to alternate function mode, push-pull output,
 * low-speed, no pull-up/down resistors, and AF0
 */
void configPinB_AF0(uint8_t x) {
  // Set to Alternate function mode, 10
  GPIOB->MODER &= ~(1 << (2*x));
  GPIOB->MODER |= (1 << ((2*x)+1));
  // Set to Push-pull
  GPIOB->OTYPER &= ~(1 << x);
  // Set to Low speed
  GPIOB->OSPEEDR |= ((1 << (2*x)) | (1 << ((2*x)+1)));
  // Set to no pull-up/down
  GPIOB->PUPDR &= ~((1 << (2*x)) | (1 << ((2*x)+1)));
  // Set alternate functon to AF0, SPI2
  if (x < 8) {  // use AFR low register
    GPIOB->AFR[0] &= ~(0xF << (4*x));
  }
  else {  // use AFR high register
    GPIOB->AFR[1] &= ~(0xF << (4*(x-8)));
  }
}
