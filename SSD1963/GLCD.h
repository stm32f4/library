/****************************************Copyright (c)**************************************************                         
 **
 **                                 http://www.powermcu.com
 **
 **--------------File Info-------------------------------------------------------------------------------
 ** File name:			GLCD.h
 ** Descriptions:		SSD1963
 **
 **------------------------------------------------------------------------------------------------------
 ** Created by:			AVRman
 ** Created date:		2011-2-23
 ** Version:			1.0
 ** Descriptions:		The original version
 **
 **------------------------------------------------------------------------------------------------------
 ** Modified by:
 ** Modified date:
 ** Version:
 ** Descriptions:
 ********************************************************************************************************/

#ifndef __GLCD_H 
#define __GLCD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "SSD1963_CMD.h"
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define USE_16BIT_PMP

/*********************************************************************
 * Overview: Horizontal and vertical display resolution
 *                  (from the glass datasheet).
 *********************************************************************/
#define DISP_HOR_RESOLUTION				320
#define DISP_VER_RESOLUTION				240

/*********************************************************************
 * Overview: Horizontal synchronization timing in pixels
 *                  (from the glass datasheet).
 *********************************************************************/
#define DISP_HOR_PULSE_WIDTH		20    /* 20 */
#define DISP_HOR_BACK_PORCH			48	  /* 48	*/
#define DISP_HOR_FRONT_PORCH		0	  /* 20 */

/*********************************************************************
 * Overview: Vertical synchronization timing in lines
 *                  (from the glass datasheet).
 *********************************************************************/
#define DISP_VER_PULSE_WIDTH		2	  /* 2 */
#define DISP_VER_BACK_PORCH			16	  /* 16 */
//#define DISP_VER_FRONT_PORCH		4	  /* 4 */

/*********************************************************************
 * Definition for SPI interface for HIMAX 8238-A relevant to hardware
 * layout; Hardware dependent!
 *********************************************************************/
#define GPIO3 3
#define GPIO2 2
#define GPIO1 1
#define GPIO0 0
#define LCD_RESET (1<<GPIO3)	   /* LCD Reset signal (Reset for display panel, NOT ssd1963) */
#define LCD_SPENA (1<<GPIO0)	   /* SPI EN signal */
#define LCD_SPCLK (1<<GPIO1)	   /* SPI CLK */
#define LCD_SPDAT (1<<GPIO2)	   /* SPI DATA */

/* LCD color */
#define White          0xFFFF
#define Black          0x0000
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0
#define RGB565CONVERT(red, green, blue)		(uint16_t) ((((red) >> 3) << 11) | (((green) >> 2) << 5) | ((blue) >> 3))

/* Private function prototypes -----------------------------------------------*/
void LCD_Init(void);
void LCD_Clear(uint16_t Color);
void LCD_SetBacklight(uint8_t intensity);
uint16_t LCD_GetPoint(uint16_t Xpos, uint16_t Ypos);
void LCD_SetPoint(uint16_t Xpos, uint16_t Ypos, uint16_t point);
void PutChar(uint16_t Xpos, uint16_t Ypos, uint8_t c, uint16_t charColor,
		uint16_t bkColor);
void LCD_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
		uint16_t color);
void PutChinese(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color,
		uint16_t bkColor);
void GUI_Text(uint16_t Xpos, uint16_t Ypos, char *str, uint16_t Color,
		uint16_t bkColor);
void GUI_Chinese(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color,
		uint16_t bkColor);
void LCD_DrawPicture(uint16_t StartX, uint16_t StartY, uint16_t EndX,
		uint16_t EndY, uint16_t *pic);
uint8_t LCD_GetScanLine();
void LCD_FillArea(uint16_t start_x, uint16_t start_y, uint16_t end_x,
		uint16_t end_y, uint16_t color);
#endif

/*********************************************************************************************************
 END FILE
 *********************************************************************************************************/

