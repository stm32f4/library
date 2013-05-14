#define	LCD_COMMAND	0
#define LCD_DATA 1

void LCD5110_init(void);

void LCD5110_Write_Char(unsigned char c);

void LCD5110_Write_Char_inv(unsigned char c);

void LCD5110_Clear(void);

void LCD5110_Set_XY(unsigned char X, unsigned char Y);

void LCD5110_Set_Pos(unsigned char X, unsigned char Y);

void LCD5110_Write_String(char *s);

void LCD5110_Write_Dec(unsigned int buffer);

void LCD5110_Led(unsigned char c);

void LCD5110_write_byte(unsigned char val);
