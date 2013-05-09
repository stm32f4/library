#ifndef __5110_H
#define __5110_H

#ifdef __HY32
#include "5110 - Hy32.h"
#else
#include "5110 - discovery.h"
#endif
#include "stm32f4xx.h"

#define	LCD_COMMAND	0
#define LCD_DATA 1

void LCD5110_init(void);

void LCD5110_Write_Char(unsigned char c);

void LCD5110_Write_Char_inv(unsigned char c);

void LCD5110_Clear(void);

void LCD5110_Set_XY(unsigned char X, unsigned char Y);

void LCD5110_Set_Pos(unsigned char X, unsigned char Y);

void LCD5110_Write_String(char *s);

void LCD5110_Write_Dec(uint16_t buffer, uint8_t size);

void LCD5110_Write_Dec32(uint32_t buffer, uint8_t size);

void LCD5110_Led(unsigned char c);

void LCD5110_Write_Byte(unsigned char val);

#endif
