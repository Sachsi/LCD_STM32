//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32F103RCT6,����ԭ��MiniSTM32������,��Ƶ72MHZ������12MHZ
//QDtech-TFTҺ������ for STM32 IOģ��
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567 
//�ֻ�:15989313508���빤�� 
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/08/09
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================��Դ����================================================//
//     LCDģ��                STM32��Ƭ��
//      VCC          ��        DC5V/3.3V        //��Դ
//      GND          ��          GND            //��Դ��
//=======================================Һ���������߽���==========================================//
//��ģ��Ĭ��������������ΪSPI����
//     LCDģ��                STM32��Ƭ��    
//     DB0~DB15      ��        PB0~PB15         //Һ����16λ���������ź�
//=======================================Һ���������߽���==========================================//
//     LCDģ�� 				  STM32��Ƭ�� 
//     LCD_WR        ��          PC7           //Һ����д���ݿ����ź�
//     LCD_RD        ��          PC6           //Һ���������ݿ����ź�
//     LCD_RS        ��          PC8           //Һ��������/��������ź�
//     LCD_RST       ��          PC10          //Һ������λ�����ź�
//     LCD_CS        ��          PC9           //Һ����Ƭѡ�����ź�
//=========================================������������=========================================//
//���ģ�鲻���������ܻ��ߴ��д������ܣ����ǲ���Ҫ�������ܣ�����Ҫ���д���������
//	   LCDģ��                STM32��Ƭ�� 
//      T_IRQ        ��          PC1        //�����������ж��ź�
//      T_DO         ��          PC2          //������SPI���߶��ź�
//      T_DIN        ��          PC3          //������SPI����д�ź�
//      T_CS         ��          PC13         //������Ƭѡ�����ź�
//      T_CLK        ��          PC0          //������SPI����ʱ���ź�
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
#ifndef __LCD_H
#define __LCD_H		
//#include "main.h"
//#include "sys.h"	 
#include "stm32g0xx_hal.h"
#include "stdlib.h"
#include <stdint.h>

//LCD��Ҫ������
typedef struct  
{										    
	uint16_t 	width;			//LCD ����
	uint16_t 	height;			//LCD �߶�
	uint16_t 	id;				  //LCD ID
	uint8_t  	dir;			  //���������������ƣ�0��������1��������	
	uint16_t	wramcmd;		//��ʼдgramָ��
	uint16_t  	rramcmd;   //��ʼ��gramָ��
	uint16_t  	setxcmd;		//����x����ָ��
	uint16_t  	setycmd;		//����y����ָ��	 
}_lcd_dev; 	

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����
/////////////////////////////////////�û�������///////////////////////////////////	 
#define USE_HORIZONTAL  	0 	//����Һ����˳ʱ����ת���� 	0-0����ת��1-90����ת��2-180����ת��3-270����ת
#define LCD_USE8BIT_MODEL   1	//�������������Ƿ�ʹ��8λģʽ 0,ʹ��16λģʽ.1,ʹ��8λģʽ

//////////////////////////////////////////////////////////////////////////////////	  
//����LCD�ĳߴ�
#define LCD_W 240
#define LCD_H 320

//TFTLCD������Ҫ���õĺ���		   
extern uint16_t  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern uint16_t  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ

////////////////////////////////////////////////////////////////////
//-----------------LCD-Data and Control Pin----------- 
#define LCD_GPIOC_TYPE  GPIOC  //GPIOC Port
//#define LED      	4       //
#define LCD_DATA_1 	GPIO_PIN_7       //Pin7 PortC
#define LCD_MASK_C (1 << 1)
#define LCD_GPIOA_TYPE  GPIOA  	//GPIOA Port
#define LCD_RS   	4       	//�Ĵ���/����ѡ������ PC8 
#define LCD_WR   	1      	//
#define LCD_RD   	0       //
#define LCD_RS_PIN	GPIO_PIN_4
#define LCD_WR_PIN	GPIO_PIN_1
#define LCD_RD_PIN	GPIO_PIN_0
#define LCD_DATA_0	GPIO_PIN_9       //
#define LCD_DATA_2  GPIO_PIN_10      //
#define LCD_DATA_7  GPIO_PIN_8       //
#define LCD_MASK_A (1 << 0 | 1 << 2 | 1 << 7)

#define LCD_GPIOB_TYPE  GPIOB  	//GPIOB Port
#define LCD_RST  11       	//
#define LCD_CS   1       	//  
#define LCD_RST_PIN GPIO_PIN_11
#define LCD_CS_PIN	GPIO_PIN_1
#define LCD_DATA_3	GPIO_PIN_3      //
#define LCD_DATA_4  GPIO_PIN_5     	//
#define LCD_DATA_5  GPIO_PIN_4       //
#define LCD_DATA_6  GPIO_PIN_14      //
#define LCD_MASK_B (LCD_DATA_3 | LCD_DATA_4 | LCD_DATA_5 | LCD_DATA_6)
//QDtechȫϵ��ģ������������ܿ��Ʊ��������û�Ҳ���Խ�PWM���ڱ�������
//#define	LCD_LED PBout(LED) //LCD����    		 PC4
//���ʹ�ùٷ��⺯���������еײ㣬�ٶȽ����½���14֡ÿ�룬���������˾�Ƽ�����
//����IO����ֱ�Ӳ����Ĵ���������IO������ˢ�����ʿ��Դﵽ28֡ÿ�룡 

//GPIO��λ�����ߣ�
#define	LCD_CS_SET  GPIOB->BSRR=1<<LCD_CS    //Ƭѡ�˿�  	
#define	LCD_RS_SET	GPIOA->BSRR=1<<LCD_RS    //����/����    
#define	LCD_RST_SET	GPIOB->BSRR=1<<LCD_RST   //��λ			  
#define	LCD_WR_SET	GPIOA->BSRR=1<<LCD_WR    //д 	
#define LCD_RD_SET  GPIOA->BSRR=1<<LCD_RD    //��		  

//GPIO��λ�����ͣ�							    
#define	LCD_CS_CLR  GPIOB->BRR=1<<LCD_CS     //Ƭѡ�˿�  	
#define	LCD_RS_CLR	GPIOA->BRR=1<<LCD_RS     //����/����  	 
#define	LCD_RST_CLR	GPIOB->BRR=1<<LCD_RST    //��λ			  
#define	LCD_WR_CLR	GPIOA->BRR=1<<LCD_WR     //д
#define LCD_RD_CLR  GPIOA->BRR=1<<LCD_RD     //��	  		  

//PB0~15,��Ϊ������
//ע�⣺���ʹ��8λģʽ�������ߣ���Һ���������ݸ�8λ�ǽӵ�MCU�ĸ�8λ������
//�����������8λģʽ��ʾ������ΪҺ����DB10-DB17��Ӧ������Ƭ��GPIOB_Pin8-GPIOB_Pin15
//�����������16λģʽ��DB0-DB7�ֱ��GPIOB_Pin0-GPIOB_Pin7,DB10-DB17��Ӧ������Ƭ��GPIOB_Pin8-GPIOB_Pin15
#define DATAOUT(x) GPIOB->ODR=x; //�������
#define DATAIN     GPIOB->IDR;   //��������

//������ɫ
#define WHITE       0xFFFF
#define BLACK      	0x0000	  
#define BLUE       	0x001F  
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40 //��ɫ
#define BRRED 			0XFC07 //�غ�ɫ
#define GRAY  			0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	0X841F //ǳ��ɫ
#define LIGHTGRAY     0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			 		0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE      	0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE          0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)
	    															  
void LCD_Init(void);
void LCD_write(uint16_t VAL);
uint16_t LCD_read(void);
void LCD_Clear(uint16_t Color);	 
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
void LCD_DrawPoint(uint16_t x,uint16_t y);//����
uint16_t  LCD_ReadPoint(uint16_t x,uint16_t y); //����	   
void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd);
uint16_t LCD_RD_DATA(void);//��ȡLCD����								    
void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue);
void LCD_WR_REG(uint16_t data);
void LCD_WR_DATA(uint16_t data);
void LCD_ReadReg(uint16_t LCD_Reg,uint8_t *Rval,int n);
void LCD_WriteRAM_Prepare(void);
void LCD_ReadRAM_Prepare(void);   
void Lcd_WriteData_16Bit(uint16_t Data);
uint16_t Lcd_ReadData_16Bit(void);
void LCD_direction(uint8_t direction );
uint16_t Color_To_565(uint8_t r, uint8_t g, uint8_t b);
uint16_t LCD_Read_ID(void);

//�����Ȼ�����ٶȲ����죬����ʹ������ĺ궨��,����ٶ�.
//ע��Ҫȥ��lcd.c��void LCD_WR_DATA(uint16_t data)��������Ŷ
/*
#if LCD_USE8BIT_MODEL==1//ʹ��8λ������������ģʽ
	#define LCD_WR_DATA(data){\
	LCD_RS_SET;\
	LCD_CS_CLR;\
	DATAOUT(data);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	DATAOUT(data<<8);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	LCD_CS_SET;\
	}
	#else//ʹ��16λ������������ģʽ
	#define LCD_WR_DATA(data){\
	LCD_RS_SET;\
	LCD_CS_CLR;\
	DATAOUT(data);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	LCD_CS_SET;\
	} 	
#endif
*/
				  		 
#endif  
	 
	 



