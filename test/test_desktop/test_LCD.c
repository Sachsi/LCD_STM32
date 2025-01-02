/**
 * @file test_LCD.c
 * @author Tobias Sachse
 * @brief 
 * @version 0.1
 * @date 2024-12-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <unity.h>
#include "test_LCD.h"



void test_ignored(){
    TEST_IGNORE_MESSAGE("This Message was ignored!");
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_ignored);
    UNITY_END();
}