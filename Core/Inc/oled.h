#ifndef __OLED_H
#define __OLED_H

 /**************************************************************************
Author：Minibalance
Our aliexpress：https://minibalance.aliexpress.com
**************************************************************************/
//-----------------OLED port definition----------------
#include "stm32f1xx_hal.h"
#define OLED_CMD  0	//Write command
#define OLED_DATA 1	//Write data
//OLED control function
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Clear_2(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p);
#endif

