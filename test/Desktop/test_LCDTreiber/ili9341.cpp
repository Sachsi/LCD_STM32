#include <ILI9341_Controller.hpp>
#include "ili9341.hpp"
#include <unity.h>


/* sometimes you may want to get at local data in a module.
 * for example: If you plan to pass by reference, this could be useful
 * however, it should often be avoided */
//extern int Counter;

void setUp(void)
{
  /* This is run before EACH TEST */
  
  //Counter = 0x5a5a;
}

void tearDown(void)
{
}
void test_Message(void)
{
    TEST_IGNORE_MESSAGE("This Test is ignored");
}

void test_size_of_LCD(void)
{
    ILI9341 LCD(120, 320);
    auto a = LCD.getSize();
    TEST_ASSERT_EQUAL_UINT16(120, a.heigth);
    TEST_ASSERT_EQUAL_UINT16(320, a.width);    
}

void test_write_8Bit_to_LCD(void)
{
    ILI9341 LCD(120,320);
    LCD.WriteBits(0x66);
    TEST_ASSERT_EQUAL_UINT16(0x66, LCD.getBuffer());
}

void test_GettheCorrectPinMask_ForDataValue(void)
{
    #define LCD_BIT_MASK_C (1 << 1)
    #define LCD_BIT_MASK_A (1 << 2 | 1 << 0 | 1 << 3)

    ILI9341 LCD(120,320);
    uint16_t data = LCD.GetGPIOPinMask(LCD_BIT_MASK_C, 0x55);
    TEST_ASSERT_EQUAL_UINT16(0x00, data);
    data = LCD.GetGPIOPinMask(LCD_BIT_MASK_C, 0x66);
    TEST_ASSERT_EQUAL_UINT16(0x80, data);
    data = LCD.GetGPIOPinMask(LCD_BIT_MASK_A, 0x55);
    TEST_ASSERT_EQUAL_UINT16(0x0600, data);
    data = LCD.GetGPIOPinMask(LCD_BIT_MASK_A, 0x0D);
    TEST_ASSERT_EQUAL_UINT16(0x0608, data);
}

/*=======MAIN=====*/
int runUnityTest(void)
{
  UNITY_BEGIN();
  RUN_TEST(test_Message);
  RUN_TEST(test_size_of_LCD);
  RUN_TEST(test_write_8Bit_to_LCD);
  RUN_TEST(test_GettheCorrectPinMask_ForDataValue);
  return (UnityEnd());
}

