/**
 * @file ILI9341_Controller.cpp
 * @author Tobias Sachse
 * @brief 
 * @version 0.1
 * @date 2025-01-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "ILI9341_Controller.hpp"

ILI9341::ILI9341(uint16_t h, uint16_t w ){
 LCD_Size.heigth = h;
 LCD_Size.width = w;
}
ILI9341::~ILI9341(){

}

void ILI9341::WriteBits(uint16_t value)
{
    m_Buffer = value;
}
uint16_t ILI9341::getBuffer(void){
    return m_Buffer;
}

size ILI9341::getSize(void){
    return LCD_Size;
}

uint16_t ILI9341::GetGPIOPinMask(uint16_t GPIOPinMask, uint8_t value)
{
    uint16_t temp = GPIOPinMask & value;

	uint16_t data = 0;
	for(uint8_t i = 0; i < 8; i++ )
    {
		if((temp >> i) & 0x1)
		{
            switch (static_cast<LCD_BIT>(i))
            {
            case LCD_BIT::LCD_D0:
                data |= 1 << static_cast<int>(LCD_PIN::LCD_D0);
                break;
            case LCD_BIT::LCD_D1:
                data |= 1 << static_cast<int>(LCD_PIN::LCD_D1);
                break;
            case LCD_BIT::LCD_D2:
                data |= 1 << static_cast<int>(LCD_PIN::LCD_D2);
                break;
            case LCD_BIT::LCD_D3:
                data |= 1 << static_cast<int>(LCD_PIN::LCD_D3);
                break;
            case LCD_BIT::LCD_D4:
                data |= 1 << static_cast<int>(LCD_PIN::LCD_D4);
                break;
            case LCD_BIT::LCD_D5:
                data |= 1 << static_cast<int>(LCD_PIN::LCD_D5);
                break;
            case LCD_BIT::LCD_D6:
                data |= 1 << static_cast<int>(LCD_PIN::LCD_D5);
                break;
            case LCD_BIT::LCD_D7:
                data |= 1 << static_cast<int>(LCD_PIN::LCD_D6);
                break;
            default:
                break;
            }
        }
    }
    return data;
}