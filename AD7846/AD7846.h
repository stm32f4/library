#ifndef __TOUCH_7846_H
#define __TOUCH_7846_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

#define LCD_Width				240
#define LCD_Height 				320
#define RESCALE_FACTOR 			1000000
#define USE_SPI3
//#define USE_SPI2
#define CS_PORT					GPIOA
#define CS_PIN					GPIO_Pin_15
#define IRQ_PORT				GPIOC
#define IRQ_PIN					GPIO_Pin_5
#define INT_PORT				EXTI_PortSourceGPIOC
#define INT_PIN_SOURCE			EXTI_PinSource5
#define INT_LINE				EXTI_Line5

#define DELAY_ON	30
#define DELAY_OFF	30


typedef struct {
	u16 X0;
	u16 Y0;
	u16 X;
	u16 Y;
} Pen_Holder;

typedef enum tp_event TP_EVENT;
enum tp_event {
	on, off, none
};

extern Pen_Holder Pen_Point;
extern volatile int touch_done;
extern volatile s32 A2;
extern volatile s32 B2;
extern volatile s32 C2;
extern volatile s32 D2;
extern volatile s32 E2;
extern volatile s32 F2;

#define T_CS()   GPIO_ResetBits(CS_PORT, CS_PIN);
#define T_DCS()  GPIO_SetBits(CS_PORT, CS_PIN);

#define CMD_RDX 0X90  //0B10010000
#define CMD_RDY	0XD0  //0B11010000
unsigned char SPI_WriteByte(u8 num);
void SpiDelay(unsigned int DelayCnt);
u16 TPReadX(void);
u16 TPReadY(void);
u8 read_once(void);
u8 Read_Ads7846(void);
u8 Read_Ads7846_filter(void);

void EXTI9_5_IRQHandler(void);
void NVIC_TOUCHConfiguration(void);
void TC_InitSPI(void);
void LCD_ShowNum(uint8_t x, uint16_t y, uint16_t data);
void TC_set_interrupt(int state);
void Convert_Pos(void);
void TS_Calibrate(u16 width, u16 height);
void TC_SetTouchCallBack(void (*f)(TP_EVENT));

#endif
