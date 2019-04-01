/**
 ********************************************************************************
 *
 * Pin configuration for 8 bits mode
 *
 *        TPCS  : PB12 (1.36)
 *        SCK   : PB13 (1.37)
 *        MISO  : PB14 (1.38)
 *        MOSI  : PB15 (1.39)
 *        TPINT : PD6  (2.30)
 */
#include "AD7846.h"
#include "stm32f4xx_it.h"
#include "stdio.h"
#include "stdlib.h"
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

void TC_set_interrupt(int state);

Pen_Holder Pen_Point;
volatile int touch_done = 0;
volatile s32 A2 = 166265;
volatile s32 B2 = 417;
volatile s32 C2 = -14737797;
volatile s32 D2 = -1340;
volatile s32 E2 = -129035;
volatile s32 F2 = 252764014;
unsigned int xxx;
unsigned int yyy;
unsigned char flag = 0;
void (*TpTouchCallBack)(TP_EVENT) = 0;

TP_EVENT volatile event = none;
u8 volatile available = 0;

unsigned char SPI_WriteByte(u8 num) {
	unsigned char Data = 0;
#ifdef USE_SPI3
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPI3, num);
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET)
		;
	Data = SPI_I2S_ReceiveData(SPI3);
#endif

#ifdef USE_SPI2
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
	;
	SPI_I2S_SendData(SPI2, num);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	;
	Data = SPI_I2S_ReceiveData(SPI2);
#endif

	return Data;
}

void SpiDelay(unsigned int DelayCnt) {
	unsigned int i;
	for (i = 0; i < DelayCnt; i++)
		;
}

u16 TPReadX(void) {
	u16 x = 0;
	T_CS()
	;
	SpiDelay(10);
	SPI_WriteByte(CMD_RDX);
	SpiDelay(10);
	x = SPI_WriteByte(0x0);
	x <<= 8;
	x += SPI_WriteByte(0x0);
	T_DCS()
	;
	//SpiDelay(10);
	x = x >> 4;
	x = x & 0xfff;
	x = 2047 - x;
	return x;
}

u16 TPReadY(void) {
	u16 y = 0;
	T_CS()
	;
	SpiDelay(10);
	SPI_WriteByte(CMD_RDY);
	SpiDelay(10);
	y = SPI_WriteByte(0x0);
	y <<= 8;
	y += SPI_WriteByte(0x0);
	T_DCS()
	;
	y = y >> 4;
	y = y & 0xfff;
	return (y);
}

u8 read_once(void) {
	Pen_Point.X = TPReadY();
	Pen_Point.Y = TPReadX();
	return 1;
}

void TC_set_interrupt(int state) {

	if (state == 0) {
		EXTI_ClearITPendingBit(INT_LINE);
		EXTI->IMR &= ~INT_LINE;
		EXTI->EMR &= ~INT_LINE;
	} else {
		EXTI->IMR |= INT_LINE;
		EXTI->EMR |= INT_LINE;
	}
}

void TC_InitSPI() {
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	// Enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

#ifdef USE_SPI2
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2); //sclk	10	 13
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);//miso	11	 14
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);//mosi	12	 15

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_I2S_DeInit(SPI2);
#endif
#ifdef USE_SPI3
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3); //PB3 SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3); //PB5 MISO

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3); //PC12 MOSI

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	SPI_I2S_DeInit(SPI3);
#endif

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //SPI_CPOL_Low 	 SPI_CPOL_High
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //SPI_NSS_Hard	 //SPI_NSS_Soft
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; //16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

#ifdef USE_SPI2
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
#endif
#ifdef USE_SPI3
	SPI_Init(SPI3, &SPI_InitStructure);
	SPI_Cmd(SPI3, ENABLE);
#endif

	//CS
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = CS_PIN; // 3
	GPIO_Init(CS_PORT, &GPIO_InitStruct); // d
	T_DCS()
	;
	//T_PEN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = IRQ_PIN;
	GPIO_Init(IRQ_PORT, &GPIO_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(INT_PORT, INT_PIN_SOURCE);
	EXTI_InitStructure.EXTI_Line = INT_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

u8 Read_Ads7846(void) {
	u8 t, t1, count = 0;
	u16 databuffer[2][10];
	u16 temp = 0;
	while (count < 10) {
		{
			if (read_once()) {
				databuffer[0][count] = Pen_Point.X;
				databuffer[1][count] = Pen_Point.Y;
				count++;
			}
		}
	}

	if (count == 10) {
		do {
			t1 = 0;
			for (t = 0; t < count - 1; t++) {
				if (databuffer[0][t] > databuffer[0][t + 1]) {
					temp = databuffer[0][t + 1];
					databuffer[0][t + 1] = databuffer[0][t];
					databuffer[0][t] = temp;
					t1 = 1;
				}
			}
		} while (t1);
		do {
			t1 = 0;
			for (t = 0; t < count - 1; t++) {
				if (databuffer[1][t] > databuffer[1][t + 1]) {
					temp = databuffer[1][t + 1];
					databuffer[1][t + 1] = databuffer[1][t];
					databuffer[1][t] = temp;
					t1 = 1;
				}
			}
		} while (t1);

		Pen_Point.X = (databuffer[0][3] + databuffer[0][4] + databuffer[0][5]) / 3;
		Pen_Point.Y = (databuffer[1][3] + databuffer[1][4] + databuffer[1][5]) / 3;
		xxx = Pen_Point.X;
		yyy = Pen_Point.Y;
		flag = 1;
		if (xxx > 10 && yyy > 10) {
			touch_done = 1;
		}
		return 1;
	}
	flag = 0;
	return 0;
}

u8 Read_Ads7846_filter(void) {
	const u8 samples = 10;
	const u8 keep = 4;
	u8 t;
	u8 count = 0;
	u8 valid = 0;
	u16 databuffer[2][samples];
	s16 distance[2][samples];
	u16 averageX = 0;
	u16 averageY = 0;
	u16 temp = 0;

	// Acquire samples
	while (count < samples) {
		{
			if (read_once()) {
				databuffer[0][count] = Pen_Point.X;
				databuffer[1][count] = Pen_Point.Y;
				count++;
			}
		}
	}

	// Calculate samples average
	for (count = 0; count < samples; count++) {
		if (databuffer[0][count] > 0 && databuffer[1][count] > 0) {
			averageX += databuffer[0][count];
			averageY += databuffer[1][count];
			valid++;
		}
	}
	averageX /= valid;
	averageY /= valid;

	// Calculate distance from average for each sample
	valid = 0;
	for (count = 0; count < samples; count++) {
		if (databuffer[0][count] > 0 && databuffer[1][count] > 0) {
			distance[0][valid] = databuffer[0][count] - averageX;
			distance[1][valid] = databuffer[1][count] - averageY;
			valid++;
		}
	}

	// Sort distances
	for (count = valid; count > 0; count--) {
		for (t = 0; t < count - 1; t++) {
			if (abs(distance[0][t]) > abs(distance[0][t + 1])) {
				temp = distance[0][t + 1];
				distance[0][t + 1] = distance[0][t];
				distance[0][t] = temp;
			}
			if (abs(distance[1][t]) > abs(distance[1][t + 1])) {
				temp = distance[1][t + 1];
				distance[1][t + 1] = distance[1][t];
				distance[1][t] = temp;
			}
		}
	}

	// Keep only some values
	databuffer[0][0] = 0;
	databuffer[1][0] = 0;
	for (count = 0; count < keep && count < valid; count++) {
		databuffer[0][0] += distance[0][count] + averageX;
		databuffer[1][0] += distance[1][count] + averageY;
	}
	databuffer[0][0] /= count;
	databuffer[1][0] /= count;

	if (valid < keep) {
		Pen_Point.X = 0;
		Pen_Point.Y = 0;
	} else {
		Pen_Point.X = databuffer[0][0];
		Pen_Point.Y = databuffer[1][0];
	}

	xxx = Pen_Point.X;
	yyy = Pen_Point.Y;

	flag = 1;
	return 1;
}

void EXTI9_5_IRQHandler(void) {
	if (EXTI_GetITStatus(INT_LINE) != RESET) {
		SetTrace();
		Delay_ms(1);
		ResetTrace();
		u8 INT_Line_Value = GPIO_ReadInputDataBit(IRQ_PORT, IRQ_PIN);
		if (INT_Line_Value == 0) {
			Delay_ms(50);
			if (GPIO_ReadInputDataBit(IRQ_PORT, IRQ_PIN) == 0) {
				Read_Ads7846_filter();
				Convert_Pos();
				available = 1;
				event = on;
			}
		} else if ((INT_Line_Value == 1) && (available == 1)) {
			Delay_ms(50);
			if (GPIO_ReadInputDataBit(IRQ_PORT, IRQ_PIN) == 1) {
				event = off;
			}
		}
		EXTI_ClearITPendingBit(INT_LINE);
		if (TpTouchCallBack && (available == 1)) {
			TpTouchCallBack(event);
			event = none;
		}
	}
}

void Convert_Pos(void) {
	Pen_Point.X0 = (Pen_Point.X * A2 + Pen_Point.Y * B2 + C2) / RESCALE_FACTOR;
	Pen_Point.Y0 = (Pen_Point.X * D2 + Pen_Point.Y * E2 + F2) / RESCALE_FACTOR;
}

void GetSample(u16 refX, u16 refY, u32 * _posX, u32 * _posY) {
	// Draw cross
	LCD_DrawCross(refX, refY, 1);

	// Get the sample after 100ms
	while (1) {
		Pen_Point.X = 0;
		Read_Ads7846();
		if (Pen_Point.X > 10 && Pen_Point.Y > 10) {
			Delay_ms(100);
			Read_Ads7846();
			break;
		}
	}
	*_posX = Pen_Point.X;
	*_posY = Pen_Point.Y;

	// Erase cross
	LCD_DrawCross(refX, refY, 0);

	u8 cnt = 10;
	// Wait for no touch for at least 100ms
	while (cnt > 0) {
		Delay_ms(10);
		Read_Ads7846();
		if (Pen_Point.X < 10 && Pen_Point.Y < 10) {
			cnt--;
		} else {
			cnt = 10;
		}
	}
}
void TS_Calibrate(u16 width, u16 height) {

	TC_set_interrupt(0);

	uint32_t coordinate_X1a = 0, coordinate_X2a = 0, coordinate_X3a = 0, coordinate_X4a = 0, coordinate_X5a = 0;
	uint32_t coordinate_Y1a = 0, coordinate_Y2a = 0, coordinate_Y3a = 0, coordinate_Y4a = 0, coordinate_Y5a = 0;
	uint32_t coordinate_X1b = 0, coordinate_X2b = 0, coordinate_X3b = 0, coordinate_X4b = 0, coordinate_X5b = 0;
	uint32_t coordinate_Y1b = 0, coordinate_Y2b = 0, coordinate_Y3b = 0, coordinate_Y4b = 0, coordinate_Y5b = 0;
	uint32_t coordinate_X1 = 0, coordinate_X2 = 0, coordinate_X3 = 0, coordinate_X4 = 0, coordinate_X5 = 0;
	uint32_t coordinate_Y1 = 0, coordinate_Y2 = 0, coordinate_Y3 = 0, coordinate_Y4 = 0, coordinate_Y5 = 0;
	uint16_t Xd1 = (width / 2), Xd2 = 1 * (width / 8), Xd3 = 7 * (width / 8), Xd4 = 7 * (width / 8), Xd5 = 1 * (width / 8);
	uint16_t Yd1 = (height / 2), Yd2 = 1 * (height / 8), Yd3 = 1 * (height / 8), Yd4 = 7 * (height / 8), Yd5 = 7 * (height / 8);
	double A = 0.0, B = 0.0, C = 0.0, D = 0.0, E = 0.0, F = 0.0;
	double d = 0.0, dx1 = 0.0, dx2 = 0.0, dx3 = 0.0, dy1 = 0.0, dy2 = 0.0, dy3 = 0.0;
	uint32_t X2_1 = 0, X2_2 = 0, X2_3 = 0, X2_4 = 0, X2_5 = 0;
	uint32_t Y2_1 = 0, Y2_2 = 0, Y2_3 = 0, Y2_4 = 0, Y2_5 = 0;
	uint32_t XxY_1 = 0, XxY_2 = 0, XxY_3 = 0, XxY_4 = 0, XxY_5 = 0;
	uint32_t XxXd_1 = 0, XxXd_2 = 0, XxXd_3 = 0, XxXd_4 = 0, XxXd_5 = 0;
	uint32_t YxXd_1 = 0, YxXd_2 = 0, YxXd_3 = 0, YxXd_4 = 0, YxXd_5 = 0;
	uint32_t XxYd_1 = 0, XxYd_2 = 0, XxYd_3 = 0, XxYd_4 = 0, XxYd_5 = 0;
	uint32_t YxYd_1 = 0, YxYd_2 = 0, YxYd_3 = 0, YxYd_4 = 0, YxYd_5 = 0;
	uint32_t alfa = 0, beta = 0, chi = 0, Kx = 0, Ky = 0, Lx = 0, Ly = 0;
	uint16_t epsilon = 0, fi = 0, Mx = 0, My = 0;

	// First set of samples
	GetSample(width / 2, height / 2, &coordinate_X1a, &coordinate_Y1a);
	GetSample(width / 8, height / 8, &coordinate_X2a, &coordinate_Y2a);
	GetSample((7 * width) / 8, height / 8, &coordinate_X3a, &coordinate_Y3a);
	GetSample((7 * width) / 8, (7 * height) / 8, &coordinate_X4a, &coordinate_Y4a);
	GetSample(width / 8, (7 * height) / 8, &coordinate_X5a, &coordinate_Y5a);

	// Second set of samples
	GetSample(width / 2, height / 2, &coordinate_X1b, &coordinate_Y1b);
	GetSample(width / 8, height / 8, &coordinate_X2b, &coordinate_Y2b);
	GetSample((7 * width) / 8, height / 8, &coordinate_X3b, &coordinate_Y3b);
	GetSample((7 * width) / 8, (7 * height) / 8, &coordinate_X4b, &coordinate_Y4b);
	GetSample(width / 8, (7 * height) / 8, &coordinate_X5b, &coordinate_Y5b);

	/* Average between X and Y coupled Touchscreen values */
	coordinate_X1 = (coordinate_X1a + coordinate_X1b) / 2;
	coordinate_X2 = (coordinate_X2a + coordinate_X2b) / 2;
	coordinate_X3 = (coordinate_X3a + coordinate_X3b) / 2;
	coordinate_X4 = (coordinate_X4a + coordinate_X4b) / 2;
	coordinate_X5 = (coordinate_X5a + coordinate_X5b) / 2;

	coordinate_Y1 = (coordinate_Y1a + coordinate_Y1b) / 2;
	coordinate_Y2 = (coordinate_Y2a + coordinate_Y2b) / 2;
	coordinate_Y3 = (coordinate_Y3a + coordinate_Y3b) / 2;
	coordinate_Y4 = (coordinate_Y4a + coordinate_Y4b) / 2;
	coordinate_Y5 = (coordinate_Y5a + coordinate_Y5b) / 2;

	X2_1 = (coordinate_X1 * coordinate_X1);
	X2_2 = (coordinate_X2 * coordinate_X2);
	X2_3 = (coordinate_X3 * coordinate_X3);
	X2_4 = (coordinate_X4 * coordinate_X4);
	X2_5 = (coordinate_X5 * coordinate_X5);

	Y2_1 = (coordinate_Y1 * coordinate_Y1);
	Y2_2 = (coordinate_Y2 * coordinate_Y2);
	Y2_3 = (coordinate_Y3 * coordinate_Y3);
	Y2_4 = (coordinate_Y4 * coordinate_Y4);
	Y2_5 = (coordinate_Y5 * coordinate_Y5);

	XxY_1 = (coordinate_X1 * coordinate_Y1);
	XxY_2 = (coordinate_X2 * coordinate_Y2);
	XxY_3 = (coordinate_X3 * coordinate_Y3);
	XxY_4 = (coordinate_X4 * coordinate_Y4);
	XxY_5 = (coordinate_X5 * coordinate_Y5);

	XxXd_1 = (coordinate_X1 * Xd1);
	XxXd_2 = (coordinate_X2 * Xd2);
	XxXd_3 = (coordinate_X3 * Xd3);
	XxXd_4 = (coordinate_X4 * Xd4);
	XxXd_5 = (coordinate_X5 * Xd5);

	YxXd_1 = (coordinate_Y1 * Xd1);
	YxXd_2 = (coordinate_Y2 * Xd2);
	YxXd_3 = (coordinate_Y3 * Xd3);
	YxXd_4 = (coordinate_Y4 * Xd4);
	YxXd_5 = (coordinate_Y5 * Xd5);

	XxYd_1 = (coordinate_X1 * Yd1);
	XxYd_2 = (coordinate_X2 * Yd2);
	XxYd_3 = (coordinate_X3 * Yd3);
	XxYd_4 = (coordinate_X4 * Yd4);
	XxYd_5 = (coordinate_X5 * Yd5);

	YxYd_1 = (coordinate_Y1 * Yd1);
	YxYd_2 = (coordinate_Y2 * Yd2);
	YxYd_3 = (coordinate_Y3 * Yd3);
	YxYd_4 = (coordinate_Y4 * Yd4);
	YxYd_5 = (coordinate_Y5 * Yd5);

	alfa = X2_1 + X2_2 + X2_3 + X2_4 + X2_5;
	beta = Y2_1 + Y2_2 + Y2_3 + Y2_4 + Y2_5;
	chi = XxY_1 + XxY_2 + XxY_3 + XxY_4 + XxY_5;
	epsilon = coordinate_X1 + coordinate_X2 + coordinate_X3 + coordinate_X4 + coordinate_X5;
	fi = coordinate_Y1 + coordinate_Y2 + coordinate_Y3 + coordinate_Y4 + coordinate_Y5;
	Kx = XxXd_1 + XxXd_2 + XxXd_3 + XxXd_4 + XxXd_5;
	Ky = XxYd_1 + XxYd_2 + XxYd_3 + XxYd_4 + XxYd_5;
	Lx = YxXd_1 + YxXd_2 + YxXd_3 + YxXd_4 + YxXd_5;
	Ly = YxYd_1 + YxYd_2 + YxYd_3 + YxYd_4 + YxYd_5;
	Mx = Xd1 + Xd2 + Xd3 + Xd4 + Xd5;
	My = Yd1 + Yd2 + Yd3 + Yd4 + Yd5;

	d = 5 * (((double) alfa * beta) - ((double) chi * chi)) + 2 * ((double) chi * epsilon * fi) - ((double) alfa * fi * fi)
			- ((double) beta * epsilon * epsilon);
	dx1 = 5 * (((double) Kx * beta) - ((double) Lx * chi)) + ((double) fi * (((double) Lx * epsilon) - ((double) Kx * fi)))
			+ ((double) Mx * (((double) chi * fi) - ((double) beta * epsilon)));
	dx2 = 5 * (((double) Lx * alfa) - ((double) Kx * chi)) + ((double) epsilon * (((double) Kx * fi) - ((double) Lx * epsilon)))
			+ ((double) Mx * (((double) chi * epsilon) - ((double) alfa * fi)));
	dx3 = ((double) Kx * (((double) chi * fi) - ((double) beta * epsilon)))
			+ ((double) Lx * (((double) chi * epsilon) - ((double) alfa * fi)))
			+ ((double) Mx * (((double) alfa * beta) - ((double) chi * chi)));
	dy1 = 5 * (((double) Ky * beta) - ((double) Ly * chi)) + ((double) fi * (((double) Ly * epsilon) - ((double) Ky * fi)))
			+ ((double) My * (((double) chi * fi) - ((double) beta * epsilon)));
	dy2 = 5 * (((double) Ly * alfa) - ((double) Ky * chi)) + ((double) epsilon * (((double) Ky * fi) - ((double) Ly * epsilon)))
			+ ((double) My * (((double) chi * epsilon) - ((double) alfa * fi)));
	dy3 = ((double) Ky * (((double) chi * fi) - ((double) beta * epsilon)))
			+ ((double) Ly * (((double) chi * epsilon) - ((double) alfa * fi)))
			+ ((double) My * (((double) alfa * beta) - ((double) chi * chi)));

	A = dx1 / d;
	B = dx2 / d;
	C = dx3 / d;
	D = dy1 / d;
	E = dy2 / d;
	F = dy3 / d;

	/* To avoid computation with "double" variables A, B, C, D, E, F, we use the s32 variables
	 A2, B2, C2, D2, E2, F2, multiplied for a Scale Factor equal to 100000 to retain the precision*/
	A2 = (int32_t) (A * RESCALE_FACTOR);
	B2 = (int32_t) (B * RESCALE_FACTOR);
	C2 = (int32_t) (C * RESCALE_FACTOR);
	D2 = (int32_t) (D * RESCALE_FACTOR);
	E2 = (int32_t) (E * RESCALE_FACTOR);
	F2 = (int32_t) (F * RESCALE_FACTOR);

	TC_set_interrupt(1);
}

void TC_SetTouchCallBack(void (*f)(TP_EVENT)) {
	TpTouchCallBack = f;
}

