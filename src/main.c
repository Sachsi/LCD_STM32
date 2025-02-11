/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lvgl.h"
#include "lv_version.h"
#include "drivers\display\ili9341\lv_ili9341.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   320
#define TFT_VER_RES   240
#define TFT_ROTATION  LV_DISPLAY_ROTATION_0
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
#define BUF_SIZE_N (TFT_HOR_RES * 10 * BYTES_PER_PIXEL)
#define BUF_SIZE (DRAW_BUF_SIZE / 4)
uint32_t draw_buf1[BUF_SIZE_N];
//uint32_t draw_buf2[BUF_SIZE_N];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
void my_lcd_send_cmd(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, const uint8_t *param, size_t param_size);
void my_lcd_send_color(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, uint8_t *param, size_t param_size);
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
uint16_t ConvertToLCDformate(uint16_t bit_Mask, uint16_t color_full);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* LVGL calls it when a rendered image needs to copied to the display*/
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{

    //Set the drawing region
    //set_draw_window(area->x1, area->y1, area->x2, area->y2);

    int height = area->y2 - area->y1 + 1;
    int width = area->x2 - area->x1 + 1;

    //We will do the SPI write manually here for speed
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    //CS low to begin data
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

    //Write colour to each pixel
    for (int i = 0; i < width * height; i++) {
        uint16_t color_full = (*px_map << 11) | (*px_map << 5) | (*px_map);
        //parallel_write(color_full);

        HAL_GPIO_WritePin(GPIOA, LCD_MASK_A, GPIO_PIN_RESET);
        color_full = ConvertToLCDformate(LCD_BIT_MASK_A, color_full);
        HAL_GPIO_WritePin(GPIOA, LCD_MASK_A, GPIO_PIN_SET);

        HAL_GPIO_WritePin(GPIOB, LCD_MASK_B, GPIO_PIN_RESET);
        color_full = ConvertToLCDformate(LCD_BIT_MASK_B, color_full);
        HAL_GPIO_WritePin(GPIOB, LCD_MASK_B, GPIO_PIN_SET);

        HAL_GPIO_WritePin(GPIOC, LCD_MASK_C, GPIO_PIN_RESET);
        color_full = ConvertToLCDformate(LCD_BIT_MASK_C, color_full);
        HAL_GPIO_WritePin(GPIOC, LCD_MASK_C, GPIO_PIN_SET);

        px_map++;
    }

    //Return CS to high
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    
    /*Call it to tell LVGL you are ready*/
    lv_display_flush_ready(disp);
}
/**
 * @brief 
 * 
 * @param color_full 
 */
uint16_t ConvertToLCDformate(uint16_t bit_Mask, uint16_t  color_full)
{
  uint16_t temp = bit_Mask & color_full;
  uint8_t count = 0;
  uint16_t data = 0;
  for (count = 0; count < sizeof(LCD_PIN); count++)
  {
    if ((temp >> count) & 0x01)
    {
      data |= (1 << count);
    }
    else
    {
      data &= ~(1 << count);
    }
  }
  return data;
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
    /*For example  ("my_..." functions needs to be implemented by you)
    int32_t x, y;
    bool touched = my_get_touch( &x, &y );

    if(!touched) {
        data->state = LV_INDEV_STATE_RELEASED;
    } else {
        data->state = LV_INDEV_STATE_PRESSED;

        data->point.x = x;
        data->point.y = y;
    }
     */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */
    lv_init();    
    lv_display_t *my_display;
    lv_tick_get_cb_t(HAL_GetTick);
     /*Else create a display yourself*/
    my_display = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
    my_display = lv_ili9341_create(TFT_HOR_RES, TFT_VER_RES, LV_TEXT_FLAG_NONE, my_lcd_send_cmd, my_lcd_send_color );

    lv_display_set_flush_cb(my_display, my_disp_flush);
    lv_display_set_buffers(my_display, draw_buf1, NULL, sizeof(BUF_SIZE_N), LV_DISPLAY_RENDER_MODE_PARTIAL);

     /*Initialize the (dummy) input device driver*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);
    
/* Create a simple label
     * ---------------------
     lv_obj_t *label = lv_label_create( lv_screen_active() );
     lv_label_set_text( label, "Hello Arduino, I'm LVGL!" );
     lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );

     * Try an example. See all the examples
     *  - Online: https://docs.lvgl.io/master/examples.html
     *  - Source codes: https://github.com/lvgl/lvgl/tree/master/examples
     * ----------------------------------------------------------------

     lv_example_btn_1();

     * Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMO_WIDGETS
     * -------------------------------------------------------------------------------------------

     lv_demo_widgets();
     */
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);

    lv_obj_t *label = lv_label_create( lv_screen_active() );
    lv_label_set_text( label, "Hello Arduino, I'm LVGL!" );
    lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    lv_timer_handler();
    HAL_Delay(500);
    /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Send short command to the LCD. This function shall wait until the transaction finishes. */
void my_lcd_send_cmd(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, const uint8_t *param, size_t param_size)
{
    //return 0;
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, LCD_MASK_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LCD_MASK_B, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, LCD_MASK_C, GPIO_PIN_RESET);

    

    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/* Send large array of pixel data to the LCD. If necessary, this function has to do the byte-swapping. This function can do the transfer in the background. */
void my_lcd_send_color(lv_display_t *disp, const uint8_t *cmd, size_t cmd_size, uint8_t *param, size_t param_size)
{
    //return 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
