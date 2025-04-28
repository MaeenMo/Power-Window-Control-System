#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"

// must match your backpack’s address
#define LCD_I2C_ADDR 0x27

// initialize the display (4-bit, 2-line, 5×8 font)
bool LCD_I2C_Init(void);

// create a custom 5×8 glyph at CGRAM slot [0..7]
void LCD_I2C_CreateChar(uint8_t slot, const uint8_t data[8]);

// basic commands
void LCD_I2C_Clear(void);
void LCD_I2C_SetCursor(uint8_t row, uint8_t col);

// write text or a single custom character
void LCD_I2C_Print(const char *s);
void LCD_I2C_WriteChar(uint8_t c);

#endif // LCD_I2C_H