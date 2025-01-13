//////////////////////////////////////////////////////////////////////////////////	 
//->->->ֻ->ѧϰʹ�ã�δ->->->->�ɣ->->->->->->->κ->�;
//->->Ӳ->->->Ƭ->STM32F103RCT6,->->ԭ->MiniSTM32->->->,->Ƶ72MHZ->->->12MHZ
//QDtech-TFTҺ->->-> for STM32 IOģ->
//xiao->@ShenZhen QDtech co.,LTD
//->˾->վ:www.qdtft.com
//�Ա->�վ->http://qdtech.taobao.com
//wiki->->->վ->http://www.lcdwiki.com
//->˾�ṩ->->֧�֣->κμ->->->⻶ӭ->ʱ->->ѧϰ
//�̻�(->->) :+86 0755-23594567 
//�ֻ�:15989313508->�빤-> 
//->->:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//->->֧->QQ:3002773612  3002778157
//->->->->QQȺ:324828016
//->->->->:2018/08/09
//�汾->V1.0
//->Ȩ->�У->->�ؾ->�
//Copyright(C) ->->->ȫ->->�Ӽ->->->޹�˾ 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================->Դ->->================================================//
//     LCDģ->                STM32->Ƭ->
//      VCC          ->        DC5V/3.3V        //->Դ
//      GND          ->          GND            //->Դ->
//=======================================Һ->->->->�߽->�==========================================//
//
//     LCDģ                   STM32g071    
//     DB0~DB15      ->        PB0~PB15         //
//=======================================Һ->->->->�߽->�==========================================//
//     LCDcontrol			  STM32g071 
//     LCD_WR        ->          PA1           //
//     LCD_RD        ->          PA0           //
//     LCD_RS        ->          PA4           //
//     LCD_RST       ->          PB11          //
//     LCD_CS        ->          PB1           //
//=========================================->->->->->->=========================================//
//->�ģ�鲻->->->->�ܻ->ߴ->д->->->ܣ->->ǲ->�Ҫ->->->�ܣ->->�Ҫ->�д->->->->�
//	   LCDģ->                STM32->Ƭ-> 
//      T_IRQ        ->          PC1        //->->->->->�ж->ź�
//      T_DO         ->          PC2          //->->->SPI->�߶->ź�
//      T_DIN        ->          PC3          //->->->SPI->->д�ź�
//      T_CS         ->          PC13         //->->->Ƭѡ->->�ź�
//      T_CLK        ->          PC0          //->->->SPI->->ʱ->�ź�
**************************************************************************************************/	
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/		
#include "lcd.h"
#include "stdlib.h"
#include "delay.h"	 
#include "ili9341.h"
	   
//->->LCD->Ҫ->->
//Ĭ->Ϊ->->
_lcd_dev lcddev;

//->->->ɫ,->->->ɫ
uint16_t POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
uint16_t DeviceCode;	 

void LCD_write(uint16_t VAL)
{
	LCD_CS_CLR;  
	//DATAOUT(VAL);
	uint16_t temp = LCD_BIT_MASK_A & VAL;
	uint8_t count = 0;
	uint8_t data = 0;

	HAL_GPIO_WritePin(LCD_GPIOA_TYPE, LCD_MASK_A, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_GPIOB_TYPE, LCD_MASK_B, GPIO_PIN_RESET);
	temp = LCD_BIT_MASK_C & VAL;
	for(count = 0; count < sizeof(LCD_PIN); count++ )
	{
		if((temp >> count) & 0x1)
		{
			data |= 1 << LCD_PIN;
		}
		LCD_PIN++;
	}
	HAL_GPIO_WritePin(LCD_GPIOC_TYPE, LCD_MASK_C, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_GPIOA_TYPE, LCD_MASK_A, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_GPIOB_TYPE, LCD_MASK_B, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_GPIOC_TYPE, LCD_MASK_C, GPIO_PIN_SET);
	
	GPIOA->BSRR &= (~LCD_MASK_A | (LCD_MASK_A & VAL));
	GPIOB->BSRR &= (~LCD_MASK_B | (LCD_MASK_B & VAL));
	GPIOC->BSRR &= (~LCD_MASK_C | (LCD_MASK_C & VAL));
	LCD_WR_CLR; 
	LCD_WR_SET; 
	LCD_CS_SET;
}

uint16_t LCD_read(void)
{
	uint16_t data;
	LCD_CS_CLR;
	LCD_RD_CLR;
	delay_us(1);//->ʱ1us	
	data = DATAIN;
	LCD_RD_SET;
	LCD_CS_SET;
	return data;
}

/*****************************************************************************
 * @name       :void LCD_WR_REG(uint16_t data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(uint16_t data)
{ 
   LCD_RS_CLR;     
	 #if LCD_USE8BIT_MODEL
	 //LCD_write(data<<8);
	 LCD_write(data);
	 #else
	 LCD_write(data);
	 #endif
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(uint16_t data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(uint16_t data)
{
	 LCD_RS_SET;
	 #if LCD_USE8BIT_MODEL
	 //LCD_write(data<<8);
	 LCD_write(data);
	 #else
	 LCD_write(data);
	 #endif
}

/*****************************************************************************
 * @name       :uint16_t LCD_RD_DATA(void)
 * @date       :2018-11-13 
 * @function   :Read an 16-bit value from the LCD screen
 * @parameters :None
 * @retvalue   :read value
******************************************************************************/
uint16_t LCD_RD_DATA(void)
{
	LCD_RS_SET; 
	#if LCD_USE8BIT_MODEL
	return (LCD_read()>>8);
	#else
	return LCD_read();
	#endif
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   

/*****************************************************************************
 * @name       :uint16_t LCD_ReadReg(uint16_t LCD_Reg)
 * @date       :2018-11-13 
 * @function   :read value from specially registers
 * @parameters :LCD_Reg:Register address
 * @retvalue   :read value
******************************************************************************/
void LCD_ReadReg(uint16_t LCD_Reg,uint8_t *Rval,int n)
{
	LCD_WR_REG(LCD_Reg); 
	//GPIOB->CRL=0X88888888; //PB0-7  ->->->->
	//GPIOB->CRH=0X88888888; //PB8-15 ->->->->
	GPIOB->ODR=0XFFFF;     //ȫ->->->�
	while(n--)
	{		
		*(Rval++) = LCD_RD_DATA();
	}

	//GPIOB->CRL=0X33333333; 		//PB0-7  ->->->�
	//GPIOB->CRH=0X33333333; 		//PB8-15 ->->->�
	GPIOB->ODR=0XFFFF;    		//ȫ->->->�  
}

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 

/*****************************************************************************
 * @name       :void LCD_ReadRAM_Prepare(void)
 * @date       :2018-11-13 
 * @function   :Read GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.rramcmd);
}

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(uint16_t Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(uint16_t Data)
{	
   LCD_RS_SET; 
	 #if LCD_USE8BIT_MODEL
		LCD_CS_CLR;
		DATAOUT(Data);
		LCD_WR_CLR; 
		LCD_WR_SET;
		DATAOUT(Data<<8);
		LCD_WR_CLR; 
		LCD_WR_SET;
		LCD_CS_SET;
 //  LCD_write(Data&0xFF00);
//	 LCD_write(Data<<8);
	 #else
	 LCD_write(Data);
	 #endif
}

uint16_t Color_To_565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

/*****************************************************************************
 * @name       :uint16_t Lcd_ReadData_16Bit(void)
 * @date       :2018-11-13 
 * @function   :Read an 16-bit value from the LCD screen
 * @parameters :None
 * @retvalue   :read value
******************************************************************************/	
uint16_t Lcd_ReadData_16Bit(void)
{
	uint16_t r,g,b;
	LCD_RS_SET;
	LCD_CS_CLR;
	
	//dummy data
	LCD_RD_CLR;
	delay_us(1);//->ʱ1us	
	r = DATAIN;
	LCD_RD_SET;
	
	//8bit:red data
	//16bit:red and green data
	LCD_RD_CLR;
	delay_us(1);//->ʱ1us	
	r = DATAIN;
	LCD_RD_SET;
	
	//8bit:green data
	//16bit:blue data
	LCD_RD_CLR;
	delay_us(1);//->ʱ1us	
	g = DATAIN;
	LCD_RD_SET;
	
	#if LCD_USE8BIT_MODEL	
	//blue data
	LCD_RD_CLR;
	delay_us(1);//->ʱ1us	
	b = DATAIN;
	LCD_RD_SET;
	r >>= 8;
	g >>= 8;
	b >>= 8;
	#else
	b = g>>8;
	r = r>>8;
	g = r&0xFF; 
	#endif
	LCD_CS_SET;
	return Color_To_565(r, g, b);
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(uint16_t x,uint16_t y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);//->�ù->λ-> 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

/*****************************************************************************
 * @name       :uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
 * @date       :2018-11-13 
 * @function   :Read a pixel color value at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :the read color value
******************************************************************************/	
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
	uint16_t color;
	if(x>=lcddev.width||y>=lcddev.height)
	{
		return 0;	//->->�˷�Χ,ֱ�ӷ->�	
	}
	LCD_SetCursor(x,y);//->�ù->λ-> 
	LCD_ReadRAM_Prepare();
	//change to Input Mode
	//GPIOB->CRL=0X88888888; //PB0-7 
	//GPIOB->CRH=0X88888888; //PB8-15 ->->->->
	GPIOB->ODR=0XFFFF;     //ȫ->->->�
	
	color = Lcd_ReadData_16Bit();
	//change to Output Mode
	//GPIOB->CRL=0X33333333; 		//PB0-7  ->->->�
	//GPIOB->CRH=0X33333333; 		//PB8-15 ->->->�
	GPIOB->ODR=0XFFFF;    		//ȫ->->->�  
	return color;
}

/*****************************************************************************
 * @name       :void LCD_Clear(uint16_t Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clear(uint16_t Color)
{
  unsigned int i;//,m;  
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	for(i=0;i<lcddev.height*lcddev.width;i++)
	{
 //   for(m=0;m<lcddev.width;m++)
  //  {	
			Lcd_WriteData_16Bit(Color);
	//	}
	}
} 

/*****************************************************************************
 * @name       :void LCD_GPIOInit(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_GPIOInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	  /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	
	GPIO_InitStructure.Pin = (LCD_RS_PIN | LCD_WR_PIN | LCD_RD_PIN | LCD_DATA_0 | LCD_DATA_2 | LCD_DATA_7); //GPIOC10,1,0,8,9,4
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(LCD_GPIOA_TYPE, &GPIO_InitStructure);	
	GPIOA->BSRR = GPIO_InitStructure.Pin;

	GPIO_InitStructure.Pin = (LCD_RST_PIN | LCD_CS_PIN | LCD_DATA_3 | LCD_DATA_4 | LCD_DATA_5 | LCD_DATA_6);
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;	//
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(LCD_GPIOB_TYPE, &GPIO_InitStructure);
	GPIOB->BSRR = GPIO_InitStructure.Pin;

	GPIO_InitStructure.Pin = (LCD_DATA_1);
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;	//
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(LCD_GPIOC_TYPE, &GPIO_InitStructure);
	GPIOC->BSRR = GPIO_InitStructure.Pin;
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	LCD_RST_CLR;
	delay_ms(100);	
	LCD_RST_SET;
	delay_ms(50);
}

/*****************************************************************************
 * @name       :void LCD_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  
	LCD_GPIOInit();//LCD GPIO->ʼ->	
 	LCD_RESET(); //LCD ->λ
//*************2.8inch ILI9341->ʼ->**********//	
	LCD_WR_REG(0xCF);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0xC9); //C1 
	LCD_WR_DATA(0X30); 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA(0x64); 
	LCD_WR_DATA(0x03); 
	LCD_WR_DATA(0X12); 
	LCD_WR_DATA(0X81); 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA(0x85); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x7A); 
	LCD_WR_REG(0xCB);  
	LCD_WR_DATA(0x39); 
	LCD_WR_DATA(0x2C); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x34); 
	LCD_WR_DATA(0x02); 
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA(0x20); 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0xC0);    //Power control 
	LCD_WR_DATA(0x1B);   //VRH[5:0] 
	LCD_WR_REG(0xC1);    //Power control 
	LCD_WR_DATA(0x00);   //SAP[2:0];BT[3:0] 01 
	LCD_WR_REG(0xC5);    //VCM control 
	LCD_WR_DATA(0x30); 	 //3F
	LCD_WR_DATA(0x30); 	 //3C
	LCD_WR_REG(0xC7);    //VCM control2 
	LCD_WR_DATA(0XB7); 
	LCD_WR_REG(0x36);    // Memory Access Control 
	LCD_WR_DATA(0x08); 
	LCD_WR_REG(0x3A);   
	LCD_WR_DATA(0x55); 
	LCD_WR_REG(0xB1);   
	LCD_WR_DATA(0x00);   
	LCD_WR_DATA(0x1A); 
	LCD_WR_REG(0xB6);    // Display Function Control 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0xA2); 
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
	LCD_WR_DATA(0x00); 
	LCD_WR_REG(0x26);    //Gamma curve selected 
	LCD_WR_DATA(0x01); 
	LCD_WR_REG(0xE0);    //Set Gamma 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x2A); 
	LCD_WR_DATA(0x28); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x0E); 
	LCD_WR_DATA(0x08); 
	LCD_WR_DATA(0x54); 
	LCD_WR_DATA(0XA9); 
	LCD_WR_DATA(0x43); 
	LCD_WR_DATA(0x0A); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x00); 		 
	LCD_WR_REG(0XE1);    //Set Gamma 
	LCD_WR_DATA(0x00); 
	LCD_WR_DATA(0x15); 
	LCD_WR_DATA(0x17); 
	LCD_WR_DATA(0x07); 
	LCD_WR_DATA(0x11); 
	LCD_WR_DATA(0x06); 
	LCD_WR_DATA(0x2B); 
	LCD_WR_DATA(0x56); 
	LCD_WR_DATA(0x3C); 
	LCD_WR_DATA(0x05); 
	LCD_WR_DATA(0x10); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_DATA(0x3F); 
	LCD_WR_DATA(0x3F); 
	LCD_WR_DATA(0x0F); 
	LCD_WR_REG(0x2B); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3f);
	LCD_WR_REG(0x2A); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xef);	 
	LCD_WR_REG(0x11); //Exit Sleep
	delay_ms(120);
	LCD_WR_REG(0x29); //display on	

  LCD_direction(USE_HORIZONTAL);//->->LCD->ʾ->->
//	LCD_LED=1;//->->->->	 
	LCD_Clear(WHITE);//->ȫ->->ɫ
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//->ʼд->GRAM			
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(uint8_t direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(uint8_t direction)
{ 
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
			lcddev.rramcmd=0x2E;
	switch(direction){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3));
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<5)|(1<<6));
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<4)|(1<<6));
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<5)|(1<<4));
		break;	
		default:break;
	}		
}	 

/*****************************************************************************
 * @name       :uint16_t LCD_Read_ID(void)
 * @date       :2018-11-13 
 * @function   :Read ID
 * @parameters :None
 * @retvalue   :ID value
******************************************************************************/ 
uint16_t LCD_Read_ID(void)
{
	uint8_t val[4] = {0};
	LCD_ReadReg(0xD3,val,4);
	return (val[2]<<8)|val[3];
}
