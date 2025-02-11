/**
 * @file ILI9341_Init.cpp
 * @author Tobias Sachse
 * @brief 
 * @version 0.1
 * @date 2025-01-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "ILI9341_Init.hpp"

void ILI9341_Init::setLCD_Size(uint16_t x, uint16_t y){
    m_LCD_Size.heigth = y;
    m_LCD_Size.width = x;
}

ILI9341_Init::~ILI9341_Init(){

}

ILI9341_Init::size  ILI9341_Init::getSize(void){
    return m_LCD_Size;
}