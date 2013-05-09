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
#include "ILI9325.h"
#include "stdio.h"
#include "stdlib.h"
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

void TC_set_interrupt(int state);

Pen_Holder Pen_Point;
volatile int touch_done = 0;
volatile s32 A2=128611, B2=792, C2=-14202480, D2=-933, E2=174095, F2=-16121157;
unsigned int xxx;
unsigned int yyy;

unsigned char flag = 0;

unsigned char SPI_WriteByte(u8 num) {
	unsigned char Data = 0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
	SPI_I2S_SendData(SPI2, num);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;
	Data = SPI_I2S_ReceiveData(SPI2);

	return Data;
}

void SpiDelay(unsigned int DelayCnt) {
	unsigned int i;
	for (i = 0; i < DelayCnt; i++)
		;
}

u16 TPReadX(void) {
	u16 x = 0;
	T_CS();
	SpiDelay(10);
	SPI_WriteByte(0x90);
	SpiDelay(10);
	x = SPI_WriteByte(0x0);
	x <<= 8;
	x += SPI_WriteByte(0x0);
	T_DCS();
	//SpiDelay(10);
	x = x >> 4;
	x = x & 0xfff;
	x = 2047 - x;
	return x;
}

u16 TPReadY(void) {
	u16 y = 0;
	T_CS();
	SpiDelay(10);
	SPI_WriteByte(0xd0);
	SpiDelay(10);
	y = SPI_WriteByte(0x0);
	y <<= 8;
	y += SPI_WriteByte(0x0);
	T_DCS();
	y = y >> 4;
	y = y & 0xfff;
	return (y);
}

u8 read_once(void) {
	Pen_Point.X = TPReadY();
	Pen_Point.Y = TPReadX();
	return 1;
}

void NVIC_TOUCHConfiguration(void) {
	/*
	 NVIC_InitTypeDef NVIC_InitStructure;

	 #ifdef  VECT_TAB_RAM
	 NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	 #else
	 NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	 #endif

	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 //	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;  //使用外部中断10~15
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure); 	*/
}

void TC_set_interrupt(int state) {

	if (state == 0) {
		EXTI->IMR &= ~EXTI_Line6;
		EXTI->EMR &= ~EXTI_Line6;
	} else {
		EXTI->IMR |= EXTI_Line6;
		EXTI->EMR |= EXTI_Line6;
	}
}

/**
 * Initialize SPI (SPI2) for the touch Panel
 *
 */
void TC_Init(void) {
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	// Enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2); //sclk	10	 13
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2); //miso	11	 14
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2); //mosi	12	 15

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //SPI_CPOL_Low 	 SPI_CPOL_High
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //SPI_NSS_Hard	 //SPI_NSS_Soft
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
	//CS
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12; // 3
	GPIO_Init(GPIOB, &GPIO_InitStruct); // d
	T_DCS();
	//T_PEN
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
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

		Pen_Point.X = (databuffer[0][3] + databuffer[0][4] + databuffer[0][5])
				/ 3;
		Pen_Point.Y = (databuffer[1][3] + databuffer[1][4] + databuffer[1][5])
				/ 3;
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
	u8 t, t1, count = 0;
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
		averageX += databuffer[0][count];
		averageY += databuffer[1][count];
	}
	averageX /= samples;
	averageY /= samples;

	// Calculate distance from average for each sample
	for (count = 0; count < samples; count++) {
		distance[0][count] = databuffer[0][count] - averageX;
		distance[1][count] = databuffer[1][count] - averageY;
	}

	// Sort distances
	for (count = samples; count > 0; count--) {
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
	for (count = 0; count < keep; count++) {
		databuffer[0][0] += distance[0][count] + averageX;
		databuffer[1][0] += distance[1][count] + averageY;
	}
	databuffer[0][0] /= keep;
	databuffer[1][0] /= keep;

	Pen_Point.X = databuffer[0][0];
	Pen_Point.Y = databuffer[1][0];

	xxx = Pen_Point.X;
	yyy = Pen_Point.Y;

	if (xxx > 10 && yyy > 10) {
		touch_done = 1;
	}

	flag = 1;
	return 1;
}

void EXTI9_5_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line6);
		trackTouch(TrackTouch_Pen);
	}
}

void Convert_Pos(void) {
	Pen_Point.X0 = (Pen_Point.X * A2 + Pen_Point.Y * B2 + C2) / RESCALE_FACTOR;
	Pen_Point.Y0 = (Pen_Point.X * D2 + Pen_Point.Y * E2 + F2) / RESCALE_FACTOR;
}

void GetSample(u16 refX, u16 refY, u32 * _posX, u32 * _posY) {
	LCD_Clear(0, 0, 0);
	LCD_SetTextColor(0xFF, 0xFF, 0xFF);
	LCD_Cross(refX, refY, 5);
	while (1) {
		Pen_Point.X = 0;
		Read_Ads7846();
		if (Pen_Point.X > 10) {
			Delay_ms(100);
			Read_Ads7846();
			break;
		}
	}
	*_posX = Pen_Point.X;
	*_posY = Pen_Point.Y;
	LCD_SetTextColor(0xFF, 0, 0);
	LCD_Cross(refX, refY, 5);
	while (Pen_Point.X > 10) {
		Read_Ads7846();
	}
	Delay_ms(500); /* This is to catch only one touch event */
}
void TS_Calibrate(void) {

	TC_set_interrupt(0);

	uint32_t coordinate_X1a = 0, coordinate_X2a = 0, coordinate_X3a = 0,
			coordinate_X4a = 0, coordinate_X5a = 0;
	uint32_t coordinate_Y1a = 0, coordinate_Y2a = 0, coordinate_Y3a = 0,
			coordinate_Y4a = 0, coordinate_Y5a = 0;
	uint32_t coordinate_X1b = 0, coordinate_X2b = 0, coordinate_X3b = 0,
			coordinate_X4b = 0, coordinate_X5b = 0;
	uint32_t coordinate_Y1b = 0, coordinate_Y2b = 0, coordinate_Y3b = 0,
			coordinate_Y4b = 0, coordinate_Y5b = 0;
	uint32_t coordinate_X1 = 0, coordinate_X2 = 0, coordinate_X3 = 0,
			coordinate_X4 = 0, coordinate_X5 = 0;
	uint32_t coordinate_Y1 = 0, coordinate_Y2 = 0, coordinate_Y3 = 0,
			coordinate_Y4 = 0, coordinate_Y5 = 0;
	uint16_t Xd1 = (LCD_Width / 2), Xd2 = 1 * (LCD_Width / 8), Xd3 = 7
			* (LCD_Width / 8), Xd4 = 7 * (LCD_Width / 8), Xd5 = 1
			* (LCD_Width / 8);
	uint16_t Yd1 = (LCD_Height / 2), Yd2 = 1 * (LCD_Height / 8), Yd3 = 1
			* (LCD_Height / 8), Yd4 = 7 * (LCD_Height / 8), Yd5 = 7
			* (LCD_Height / 8);
	double A = 0.0, B = 0.0, C = 0.0, D = 0.0, E = 0.0, F = 0.0;
	double d = 0.0, dx1 = 0.0, dx2 = 0.0, dx3 = 0.0, dy1 = 0.0, dy2 = 0.0, dy3 =
			0.0;
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
	GetSample(LCD_Width / 2, LCD_Height / 2, &coordinate_X1a, &coordinate_Y1a);
	GetSample(LCD_Width / 8, LCD_Height / 8, &coordinate_X2a, &coordinate_Y2a);
	GetSample((7 * LCD_Width) / 8, LCD_Height / 8, &coordinate_X3a,
			&coordinate_Y3a);
	GetSample((7 * LCD_Width) / 8, (7 * LCD_Height) / 8, &coordinate_X4a,
			&coordinate_Y4a);
	GetSample(LCD_Width / 8, (7 * LCD_Height) / 8, &coordinate_X5a,
			&coordinate_Y5a);

	// Second set of samples
	GetSample(LCD_Width / 2, LCD_Height / 2, &coordinate_X1b, &coordinate_Y1b);
	GetSample(LCD_Width / 8, LCD_Height / 8, &coordinate_X2b, &coordinate_Y2b);
	GetSample((7 * LCD_Width) / 8, LCD_Height / 8, &coordinate_X3b,
			&coordinate_Y3b);
	GetSample((7 * LCD_Width) / 8, (7 * LCD_Height) / 8, &coordinate_X4b,
			&coordinate_Y4b);
	GetSample(LCD_Width / 8, (7 * LCD_Height) / 8, &coordinate_X5b,
			&coordinate_Y5b);

	LCD_Clear(0, 0, 0);

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
	epsilon = coordinate_X1 + coordinate_X2 + coordinate_X3 + coordinate_X4
			+ coordinate_X5;
	fi = coordinate_Y1 + coordinate_Y2 + coordinate_Y3 + coordinate_Y4
			+ coordinate_Y5;
	Kx = XxXd_1 + XxXd_2 + XxXd_3 + XxXd_4 + XxXd_5;
	Ky = XxYd_1 + XxYd_2 + XxYd_3 + XxYd_4 + XxYd_5;
	Lx = YxXd_1 + YxXd_2 + YxXd_3 + YxXd_4 + YxXd_5;
	Ly = YxYd_1 + YxYd_2 + YxYd_3 + YxYd_4 + YxYd_5;
	Mx = Xd1 + Xd2 + Xd3 + Xd4 + Xd5;
	My = Yd1 + Yd2 + Yd3 + Yd4 + Yd5;

	d = 5 * (((double) alfa * beta) - ((double) chi * chi))
			+ 2 * ((double) chi * epsilon * fi) - ((double) alfa * fi * fi)
			- ((double) beta * epsilon * epsilon);
	dx1 = 5 * (((double) Kx * beta) - ((double) Lx * chi))
			+ ((double) fi * (((double) Lx * epsilon) - ((double) Kx * fi)))
			+ ((double) Mx * (((double) chi * fi) - ((double) beta * epsilon)));
	dx2 =
			5 * (((double) Lx * alfa) - ((double) Kx * chi))
					+ ((double) epsilon
							* (((double) Kx * fi) - ((double) Lx * epsilon)))
					+ ((double) Mx
							* (((double) chi * epsilon) - ((double) alfa * fi)));
	dx3 = ((double) Kx * (((double) chi * fi) - ((double) beta * epsilon)))
			+ ((double) Lx * (((double) chi * epsilon) - ((double) alfa * fi)))
			+ ((double) Mx * (((double) alfa * beta) - ((double) chi * chi)));
	dy1 = 5 * (((double) Ky * beta) - ((double) Ly * chi))
			+ ((double) fi * (((double) Ly * epsilon) - ((double) Ky * fi)))
			+ ((double) My * (((double) chi * fi) - ((double) beta * epsilon)));
	dy2 =
			5 * (((double) Ly * alfa) - ((double) Ky * chi))
					+ ((double) epsilon
							* (((double) Ky * fi) - ((double) Ly * epsilon)))
					+ ((double) My
							* (((double) chi * epsilon) - ((double) alfa * fi)));
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
