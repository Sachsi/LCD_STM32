/**
 * @file ILI9341_Controller.hpp
 * @au0thor Tobias Sachse
 * @brief 
 * @version 0.1
 * @date 2025-01-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef _ILI9341_CONTROLLER_H
#define _ILI9341_CONTROLLER_H
#include <stdint.h>
#include <map>
#include <array>
#include <list>
//-----------------LCD-Data and Control Pin----------- 
enum class LCD_PIN{
	LCD_D0 = 9,
	LCD_D1 = 7,
	LCD_D2 = 10,
	LCD_D3 = 3,
	LCD_D4 = 5,
	LCD_D5 = 4,
	LCD_D6 = 14,
	LCD_D7 = 8
};

enum class LCD_BIT{
    LCD_D0,
    LCD_D1,
    LCD_D2,
    LCD_D3,
    LCD_D4,
    LCD_D5,
    LCD_D6,
    LCD_D7,
    LCD_RS,
    LCD_WR,
    LCD_RD,
    LCD_RST,
    LCD_CS
};



struct size
{
    uint16_t width;
    uint16_t heigth;
};


class ILI9341
{
private:
    size LCD_Size{};
    uint16_t m_Buffer;
    std::list<int> m_PinList{};

public:
    ILI9341(uint16_t h, uint16_t w);
    ~ILI9341();
    
    void WriteBits(uint16_t value);
    uint16_t getBuffer(void);
    size getSize(void);
    
    uint16_t GetGPIOPinMask(uint16_t GPIOPinMask, uint8_t value);

};

#endif