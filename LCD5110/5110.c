#include "main.h"
#include "5110.h"
#include "font.h"
#include "stm32f4xx_conf.h"
#include "ascii.h"

//Define the LCD Operation function
void LCD5110_LCD_Write_Byte(unsigned char dat, unsigned char LCD5110_MOde);

//Define the hardware operation function
void LCD5110_GPIO_Config(void);
void LCD5110_SCK(unsigned char temp);
void LCD5110_DIN(unsigned char temp);
void LCD5110_CS(unsigned char temp);
void LCD5110_RST(unsigned char temp);
void LCD5110_DC(unsigned char temp);

//Data

/**
 * Initialize LCD module
 *
 * Input parameters : none
 * Return value		: none
 */
void LCD5110_init() {

	//Configure pins
	LCD5110_GPIO_Config();

	// Set pin initial state
	LCD5110_Led(0); //Turn back light off
	LCD5110_RST(0); //Set LCD reset = 0;
	LCD5110_DC(1); //Mode = command;
	LCD5110_DIN(1); //Set In at high level;
	LCD5110_SCK(1); //Set CLK high;
	LCD5110_CS(1); //Unselect chip;

	//Keep reset pin low for 10 ms
	Delay_ms(10);
	//Release Reset Pin
	LCD5110_RST(1); //LCD_RST = 1;

	//Configure LCD module
	LCD5110_LCD_Write_Byte(0x21, LCD_COMMAND); //Extended instruction set selected
	LCD5110_LCD_Write_Byte(0xB7, LCD_COMMAND); //Set LCD voltage (defined by experimentation...)
	LCD5110_LCD_Write_Byte(0x14, LCD_COMMAND); //Set Bias for 1/48
	LCD5110_LCD_Write_Byte(0x06, LCD_COMMAND); //Set temperature control (TC2)
	LCD5110_LCD_Write_Byte(0x20, LCD_COMMAND); //Revert to standard instruction set
	LCD5110_Clear(); //Clear display (still off)
	LCD5110_LCD_Write_Byte(0x0c, LCD_COMMAND); //Set display on in "normal" mode (not inversed)

}
/**
 * Write byte to the module.
 *
 * @param dat  	data to write
 * @param mode  0 if command, 1 if data
 *
 * @retval		None
 */
void LCD5110_LCD_Write_Byte(unsigned char dat, unsigned char mode) {
	unsigned char i;
	LCD5110_CS(0); //SPI_CS = 0;

	if (0 == mode)
		LCD5110_DC(0); //LCD_DC = 0;
	else
		LCD5110_DC(1); //LCD_DC = 1;

	for (i = 0; i < 8; i++) {
		LCD5110_DIN(dat & 0x80); //SPI_MO = dat & 0x80;
		dat = dat << 1;
		LCD5110_SCK(0); //SPI_SCK = 0;
		LCD5110_SCK(1); //SPI_SCK = 1;
	}

	LCD5110_CS(1); //SPI_CS = 1;

}

/**
 * Write character to LCD at current position
 *
 * @param c: char to write
 * @retval None
 */
void LCD5110_Write_Char(unsigned char c) {
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for (line = 0; line < 6; line++) {
		ch = font6_8[c][line];
		LCD5110_LCD_Write_Byte(ch, LCD_DATA);

	}
}

/**
 * Write character to LCD in inverse video at current location
 *
 * @param c: char to write
 * @retval None
 */
void LCD5110_Write_Char_inv(unsigned char c) {
	unsigned char line;
	unsigned char ch = 0;

	c = c - 32;

	for (line = 0; line < 6; line++) {
		ch = ~font6_8[c][line];
		LCD5110_LCD_Write_Byte(ch, LCD_DATA);

	}
}

/**
 * Write string to LCD at current position. String must be null terminated.
 *
 * @param s: string pointer
 * @retval None
 */
void LCD5110_Write_String(char *s) {
	unsigned char ch;
	while (*s != '\0') {
		ch = *s;
		LCD5110_Write_Char(ch);
		s++;
	}
}

void LCD5110_Rectangle(uint8_t X, uint8_t Y, uint8_t width, uint8_t height) {

}

/**
 * Clear display. Write 0 in all memory location.
 *
 * @param None
 * @retval None
 */
void LCD5110_Clear() {
	unsigned char i, j;
	for (i = 0; i < 6; i++)
		for (j = 0; j < 84; j++)
			LCD5110_LCD_Write_Byte(0, LCD_DATA);
}

/**
 * Set memory current location for characters (set coordinates).
 * Applies only for Fonts with a 6 pixels width.
 *
 * @param X: Column (range from 0 to 13)
 * @param Y: Row (range from 0 to 5)
 * @retval None
 *
 */
void LCD5110_Set_XY(unsigned char X, unsigned char Y) {
	unsigned char x;
	x = 6 * X;

	LCD5110_LCD_Write_Byte(0x40 | Y, LCD_COMMAND);
	LCD5110_LCD_Write_Byte(0x80 | x, LCD_COMMAND);
}

void LCD5110_Set_Pos(unsigned char X, unsigned char Y) {
	LCD5110_LCD_Write_Byte(0x40 | Y, LCD_COMMAND);
	LCD5110_LCD_Write_Byte(0x80 | X, LCD_COMMAND);
}

/**
 * Write integer to LCD
 *
 * @param b: integer to write
 * @retval None
 */
void LCD5110_Write_Dec(uint16_t value, uint8_t size) {

	char digit[5];
	uint8_t i;
	getDecimalFromShort(digit, value, 5);

	for (i = 5 - size; i < 5; i++) {
		LCD5110_Write_Char(digit[i]);
	}
}

/**
 * Write long to LCD
 *
 * @param b: Long to write
 * @retval None
 */
void LCD5110_Write_Dec32(unsigned long b, uint8_t size) {

	unsigned char digit[9];
	int i;
	uint8_t zero = 1;

	digit[0] = b / 100000000;
	b = b - digit[0] * 100000000;
	digit[1] = b / 10000000;
	b = b - digit[1] * 10000000;
	digit[2] = b / 1000000;
	b = b - digit[2] * 1000000;
	digit[3] = b / 100000;
	b = b - digit[3] * 100000;
	digit[4] = b / 10000;
	b = b - digit[4] * 10000;
	digit[5] = b / 1000;
	b = b - digit[5] * 1000;
	digit[6] = b / 100;
	b = b - digit[6] * 100;
	digit[7] = b / 10;
	digit[8] = b - digit[7] * 10;

	for (i = 9 - size; i < 9; i++) {
		if ((zero == 1) && (digit[i] == 0)) {
			digit[i] = 32;
		} else {
			zero = 0;
			digit[i] += 48;
		}
		LCD5110_Write_Char(digit[i]);
	}
}
void LCD5110_Write_Byte(u8 val) {
	LCD5110_LCD_Write_Byte(val, LCD_DATA);
}

/**
 * Set pin configuration. Doesn't use SPI controller. Just regular pins.
 *
 *	PF11 : Reset
 *	PF9  : CE
 *	PF7  : DC
 *	PF5  : MISO
 *	PF3  : CLK
 *	PF1  : LED control
 *
 * @param None
 * @retval None
 */
void LCD5110_GPIO_Config() {
	GPIO_InitTypeDef GPIO_Def;

	//Declare pins to configure
	GPIO_Def.GPIO_Pin = RESET_PIN | CE_PIN | DC_PIN | MISO_PIN | CLK_PIN
			| LED_PIN;
	GPIO_Def.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Def.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Def.GPIO_OType = GPIO_OType_PP;
	GPIO_Def.GPIO_PuPd = GPIO_PuPd_NOPULL;

	//Start clock to the selected port
	RCC_AHB1PeriphClockCmd(GPIO_CLK, ENABLE);

	//Init Port
	GPIO_Init(GPIO_NAME, &GPIO_Def);

}

/**
 * Manage CS pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_CS(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIO_NAME, CE_PIN);
	else
		GPIO_SetBits(GPIO_NAME, CE_PIN);
}

/**
 * Manage Reset pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_RST(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIO_NAME, RESET_PIN);
	else
		GPIO_SetBits(GPIO_NAME, RESET_PIN);
}

/**
 * Manage DC pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_DC(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIO_NAME, DC_PIN);
	else
		GPIO_SetBits(GPIO_NAME, DC_PIN);
}

/**
 * Manage DIN pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_DIN(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIO_NAME, MISO_PIN);
	else
		GPIO_SetBits(GPIO_NAME, MISO_PIN);
}

/**
 * Manage CLK pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_SCK(unsigned char state) {
	if (state == 0)
		GPIO_ResetBits(GPIO_NAME, CLK_PIN);
	else
		GPIO_SetBits(GPIO_NAME, CLK_PIN);
}

/**
 * Manage LED pin
 *
 * @param state: pin state (0 or 1)
 * @retval None
 */
void LCD5110_Led(unsigned char state) {
	if (state == 0)
		GPIO_SetBits(GPIO_NAME, LED_PIN);
	else
		GPIO_ResetBits(GPIO_NAME, LED_PIN);
}
