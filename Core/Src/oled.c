#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "stm32f1xx_hal.h"

 /**************************************************************************
Author：Minibalance
Our aliexpress：https://minibalance.aliexpress.com
**************************************************************************/
uint8_t OLED_GRAM[128][8];


void OLED_Refresh_Gram(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //Setting page address (0~7)
		OLED_WR_Byte (0x00,OLED_CMD);      //Set display location - low address
		OLED_WR_Byte (0x10,OLED_CMD);      //Set display location - column high address
		for(n=0;n<128;n++)
			OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}

// write a byte to OLED.
//dat: to write data / commands
//cmd: data / command flag 0, indicating command; 1, representing data;
void OLED_WR_Byte(uint8_t byte,uint8_t cmd)
{
	uint8_t i;
	if (cmd)
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // command PA13=1
	else
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // data PA13=0
	for (i=0; i<8; i++)
	{
	    HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // SCLK PB5=0
	    if (byte & 0x80)	// Serializar byte
	    	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // SDA PB4=1
	    else
	    	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // SDA PB4=0
	    HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // SCLK PB5=1
	    byte<<=1;
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);  // command PA13=1
}


// turn on the OLED display.
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC command
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//Close OLED display
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDCcommand
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
// clear screen function, clear the screen, the whole screen is black! And not lit!!!
void OLED_Clear(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
		for(n=0;n<128;n++)
			OLED_GRAM[n][i]=0X00;
	OLED_Refresh_Gram();//update display
}
void OLED_Clear_2(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
		for(n=0;n<128;n++)
			OLED_GRAM[n][i]=0X00;
	//OLED_Refresh_Gram();//update display
}
// draw a point
//x:0~127
//y:0~63
//t:1 fill 0, empty.
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t pos,bx,temp=0;
	if(x>127||y>63)return;//It's out of range.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;
}

// display a character at the specified location, including some characters.
//x:0~127
//y:0~63
//mode:0, reverse display; 1, normal display.
//size: choose font 16/12
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' ';// get the offset value.
    for(t=0;t<size;t++)
    {
		if(size==12)temp=oled_asc2_1206[chr][t];  //Call 1206 font
		else temp=oled_asc2_1608[chr][t];		 //Call 1608 font
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}
    }
}
//m^n function
uint32_t oled_pow(uint8_t m, uint8_t n)
{
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}
// show 2 numbers.
//x, Y: starting coordinates
//len: number of digits
//size: font size
//mode: mode 0, fill mode; 1, overlay mode.
//num: value (0~4294967295);
void OLED_ShowNumber(uint8_t x,uint8_t y, uint32_t num,uint8_t len,uint8_t size)
{
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1;

		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1);
	}
}
// display string
//x, y: starting coordinates
//*p: string start address
// 16 font.
void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
    while(*p!='\0')
    {
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);
        x+=8;
        p++;
    }
}
//初始化oled
void OLED_Init(void)
{
/*
 	RCC->APB2ENR|=1<<3;    // enable PORTB clock
	GPIOB->CRL&=0XFF000FFF;
	GPIOB->CRL|=0X00222000;//PB3 4 5 push-pull

 	RCC->APB2ENR|=1<<2;    //Enable PORTA clock
	GPIOA->CRH&=0X0FFFFFFF;
	GPIOA->CRH|=0X20000000;//PA15 push-pull
*/

	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // RESET PB3=0
	HAL_Delay(100);
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // RESET PB3=1

	OLED_WR_Byte(0xAE,OLED_CMD); //Close display
	OLED_WR_Byte(0xD5, OLED_CMD);
	//Set clock frequency factor, oscillation frequency
	OLED_WR_Byte(80, OLED_CMD);
	//[3:0], frequency divisor; [7:4], oscillation frequency.
	OLED_WR_Byte(0xA8,OLED_CMD); //Set the number of drivers
	OLED_WR_Byte(0X3F,OLED_CMD); //Default 0X3F (1/64)
	OLED_WR_Byte(0xD3,OLED_CMD); //Set display offset
	OLED_WR_Byte(0X00,OLED_CMD); //The default is 0

	OLED_WR_Byte(0x40,OLED_CMD); //Set the start line [5:0] and the number of rows.

	OLED_WR_Byte(0x8D,OLED_CMD); //Charge pump settings
	OLED_WR_Byte(0x14,OLED_CMD); //Bit2, on / off
	OLED_WR_Byte(0x20,OLED_CMD); //Set memory address mode
	OLED_WR_Byte(0x02, OLED_CMD); //[1:0], 00, column address mode;
	// 01, row address mode; 10, page address mode; default 10;
	OLED_WR_Byte(0xA1,OLED_CMD); //Segment redefinition settings, bit0:0,0->0; 1,0->127;
	OLED_WR_Byte(0xC0, OLED_CMD);
	//Set COM scan direction; bit3:0, normal mode;
	// 1, redefine mode COM[N-1]->COM0; N: drive path.
	OLED_WR_Byte(0xDA,OLED_CMD); //Setting up COM hardware pin configuration
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]To configure

	OLED_WR_Byte(0x81,OLED_CMD); //Contrast settings
	OLED_WR_Byte(0xEF, OLED_CMD);
	//1~255; default 0X7F (brightness setting, larger and brighter)
	OLED_WR_Byte(0xD9,OLED_CMD); //Set up pre charge cycle
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //Set VCOMH voltage multiplying rate
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4, OLED_CMD);
	//The global display is turned on; bit0:1, turn on; 0, turn off;
	//(white screen / black screen).
	OLED_WR_Byte(0xA6, OLED_CMD);
	//Set the display mode; bit0:1, reverse display; 0, normal display.
	OLED_WR_Byte(0xAF,OLED_CMD); //Open display
	OLED_Clear();
}



