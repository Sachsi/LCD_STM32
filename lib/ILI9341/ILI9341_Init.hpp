/**
 * @file ILI9341_Init.hpp
 * @author Tobias Sachse
 * @brief 
 * @version 0.1
 * @date 2025-01-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __ILI9341_INIT_H
#define __ILI9341_INIT_H
#include <stdint.h>

class ILI9341_Init
{
private:
    struct size
    {
        uint16_t width;
        uint16_t heigth;
    };
    size m_LCD_Size{};
    uint16_t m_ID;
    uint8_t  m_dir;
    uint16_t m_wram_cmd;
    uint16_t m_rram_cmd;
    uint16_t m_setx_cmd;
    uint16_t m_sety_cmd;
public:
    ILI9341_Init() = default;
    void setLCD_Size(uint16_t x, uint16_t y);
    size getSize(void);
    ~ILI9341_Init();
};

#endif