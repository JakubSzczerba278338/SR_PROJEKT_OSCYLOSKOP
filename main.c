/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  * www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.h"
#include "../../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"
#include "../../../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_ts.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_WIDTH 240
#define MAX_HEIGHT 320
#define AXIS_Y_POS 0
#define TICK_HEIGHT 5
#define BUFFER_SIZE 1024
#define ADC_RESOLUTION 4095

#define REAL_RANGE_MV 20000
#define REAL_MIN_MV  -10000
#define SCREEN_RANGE_MV 24000
#define SCREEN_MIN_MV -12000

#define LCD_FRAME_BUFFER_LAYER0 LCD_FRAME_BUFFER
#define LCD_FRAME_BUFFER_LAYER1  (LCD_FRAME_BUFFER + 0x50000)
#define LCD_COLOR_ALMOST_BLACK 0xFF010101

#define MENU_BTN_W          100
#define MENU_BTN_H          60
#define MENU_MARGIN_X       15
#define MENU_GAP_X          10
#define MENU_GAP_Y          10
#define MENU_START_Y        60

#define MENU_COL1_X         MENU_MARGIN_X
#define MENU_COL2_X         (MENU_MARGIN_X + MENU_BTN_W + MENU_GAP_X)

#define MENU_ROW1_Y         MENU_START_Y
#define MENU_ROW2_Y         (MENU_START_Y + MENU_BTN_H + MENU_GAP_Y)
#define MENU_ROW3_Y         (MENU_START_Y + 2 * (MENU_BTN_H + MENU_GAP_Y))

#define MENU_TEXT_OFFSET_Y  20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t measurementData[BUFFER_SIZE];
uint16_t msrVal = 0;

volatile uint32_t adc_ready = 0;
volatile int measurement_counter = 0;
volatile uint8_t menu_visible = 0;
uint8_t button_prev_state = 0;
TS_StateTypeDef TS_State;

uint8_t show_vpp = 1;
uint8_t show_rms = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
int32_t convert(uint32_t);
uint16_t calculate_position(uint16_t);
void Draw_Buffer(uint16_t [BUFFER_SIZE], uint32_t color);
int Find_Trigger_Index(volatile uint16_t *data, uint16_t level, int limit);
void Draw_Vpp(volatile uint16_t measurements[BUFFER_SIZE]);
void Draw_RMS(volatile uint16_t measurements[BUFFER_SIZE]);
void Draw_Menu_Overlay(void);
void Draw_Grid(void);
void Draw_Full_Menu(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER_LAYER0);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

  Draw_Grid();

  BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER_LAYER1);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetColorKeying(1, LCD_COLOR_BLACK);

  if (BSP_TS_Init(MAX_WIDTH, MAX_HEIGHT) != TS_OK) {
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(0, MAX_HEIGHT / 2, (uint8_t *)"TS ERROR", CENTER_MODE);
    HAL_Delay(1000);
  }

  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font12);
  /* USER CODE END 2 */

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)measurementData, BUFFER_SIZE);

  while (1)
  {
    uint8_t button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if (button_state == GPIO_PIN_SET && button_prev_state == GPIO_PIN_RESET) {
      menu_visible = !menu_visible;
      if (menu_visible) {
        Draw_Full_Menu();
      } else {
        BSP_LCD_Clear(LCD_COLOR_BLACK);
        HAL_ADC_Start_DMA(&hadc3, (uint32_t*)measurementData, BUFFER_SIZE);
      }
      HAL_Delay(50);
    }
    button_prev_state = button_state;

    if (menu_visible) {
      BSP_TS_GetState(&TS_State);
      if (TS_State.TouchDetected) {
        uint16_t x = TS_State.X;
        uint16_t y = TS_State.Y;

        if (x >= MENU_COL1_X && x <= (MENU_COL1_X + MENU_BTN_W) &&
            y >= MENU_ROW1_Y && y <= (MENU_ROW1_Y + MENU_BTN_H)) {
          show_vpp = !show_vpp;
          Draw_Full_Menu();
          HAL_Delay(200);
        }
        else if (x >= MENU_COL2_X && x <= (MENU_COL2_X + MENU_BTN_W) &&
                 y >= MENU_ROW1_Y && y <= (MENU_ROW1_Y + MENU_BTN_H)) {
          show_rms = !show_rms;
          Draw_Full_Menu();
          HAL_Delay(200);
        }
      }
    }
    else if (adc_ready) {
      adc_ready = 0;
      uint16_t trigger_level = 2048;
      int search_limit = 2*MAX_WIDTH;
      int trigger_idx = Find_Trigger_Index(measurementData, trigger_level, search_limit);
      static int last_trigger_idx = 0;

      if (trigger_idx != 65535) {
        last_trigger_idx = trigger_idx;
      } else {
        trigger_idx = last_trigger_idx;
      }
      if (trigger_idx > search_limit) trigger_idx = 0;

      uint16_t displayData[MAX_WIDTH];

      for (int i = 0; i < MAX_WIDTH; i++) {
        displayData[i] = calculate_position(measurementData[trigger_idx + i]);
      }
      Draw_Buffer(displayData, LCD_COLOR_RED);
      if (show_vpp) Draw_Vpp(measurementData);
      if (show_rms) Draw_RMS(measurementData);
      HAL_ADC_Start_DMA(&hadc3, (uint32_t*)measurementData, BUFFER_SIZE);
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 336;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int Find_Trigger_Index(volatile uint16_t *data, uint16_t level, int limit) {
  int hysteresis = 50;
  if (level < hysteresis) hysteresis = level;
  uint16_t arm_level = level - hysteresis;
  int state = 0;

  for (int i = 0; i < limit; i++) {
    if (state == 0) {
      if (data[i] < arm_level) {
        state = 1;
      }
    } else {
      if (data[i] >= level) {
        return i;
      }
    }
  }
  return 65535;
}

int32_t convert(uint32_t AdcValue) {
  return ((int32_t)AdcValue * REAL_RANGE_MV) / ADC_RESOLUTION + REAL_MIN_MV;
}

int32_t convert_scale_only(uint32_t AdcValue) {
  return ((int32_t)AdcValue * REAL_RANGE_MV) / ADC_RESOLUTION;
}

uint16_t calculate_position(uint16_t value) {
  int32_t voltage_mv = ((int32_t)value * REAL_RANGE_MV) / ADC_RESOLUTION + REAL_MIN_MV;
  int32_t voltage_from_bottom_mv = voltage_mv - SCREEN_MIN_MV;
  return (uint16_t)((voltage_from_bottom_mv * MAX_HEIGHT) / SCREEN_RANGE_MV);
}

void Draw_Grid(void)
{
  uint32_t originalColor = BSP_LCD_GetTextColor();
  sFONT *originalFont = BSP_LCD_GetFont();

  BSP_LCD_SetFont(&Font12);

  uint16_t center_y = MAX_HEIGHT / 2;
  uint16_t center_x = MAX_WIDTH / 2;
  int div_x = MAX_WIDTH / 10;

  for (int i = 1; i < 10; i++) {
    uint16_t x = i * div_x;
    int time_label = i - 5;

    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawLine(x, 0, x, MAX_HEIGHT);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawLine(x, center_y - 3, x, center_y + 3);

    char label[4];
    sprintf(label, "%d", time_label);

    if (time_label == 0) {
      BSP_LCD_DisplayStringAt(x + 4, center_y + 6, (uint8_t *)label, LEFT_MODE);
    } else if (time_label > 0) {
      BSP_LCD_DisplayStringAt(x - 3, center_y + 6, (uint8_t *)label, LEFT_MODE);
    } else {
      BSP_LCD_DisplayStringAt(x - 10, center_y + 6, (uint8_t *)label, LEFT_MODE);
    }
  }

  for (int k = -4; k <= 4; k++)
  {
    int32_t v_mv = k * 2500;

    int32_t voltage_from_bottom = v_mv - SCREEN_MIN_MV;
    uint16_t height_from_bottom = (uint16_t)((voltage_from_bottom * MAX_HEIGHT) / SCREEN_RANGE_MV);
    uint16_t y_pos = MAX_HEIGHT - height_from_bottom;

    if (y_pos >= MAX_HEIGHT) y_pos = MAX_HEIGHT - 1;
    if (y_pos == 0) y_pos = 1;

    if (v_mv == 0) {
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    } else {
      BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    }
    BSP_LCD_DrawLine(0, y_pos, MAX_WIDTH, y_pos);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawLine(center_x - 3, y_pos, center_x + 3, y_pos);

    if (k % 2 == 0 && v_mv != 0) {
      char label_text[6];
      sprintf(label_text, "%ld", v_mv / 1000);
      BSP_LCD_DisplayStringAt(center_x + 6, y_pos - 6, (uint8_t *)label_text, LEFT_MODE);
    }
  }
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DrawLine(center_x, 0, center_x, MAX_HEIGHT);
  BSP_LCD_SetTextColor(originalColor);
  BSP_LCD_SetFont(originalFont);
}

uint16_t Calculate_Vpp(uint16_t measurements[BUFFER_SIZE]) {
  uint16_t max = measurements[0];
  uint16_t min = measurements[0];
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (measurements[i] > max) {
      max = measurements[i];
    }
    if (measurements[i] < min) {
      min = measurements[i];
    }
  }
  return max - min;
}

int32_t Calculate_RMS(volatile uint16_t *data) {
  uint64_t sum_squares = 0;
  uint32_t sum_adc = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum_adc += data[i];
  }
  int32_t mean_adc = sum_adc / BUFFER_SIZE;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    int32_t deviation = (int32_t)data[i] - mean_adc;
    sum_squares += (int64_t)(deviation * deviation);
  }
  float rms_adc = sqrtf((float)sum_squares / BUFFER_SIZE);
  return convert_scale_only((uint32_t)rms_adc);
}

void Draw_RMS(volatile uint16_t measurements[BUFFER_SIZE]) {
  char buffer[32];
  int32_t rms_mv = Calculate_RMS(measurements);
  sprintf(buffer, "RMS:%2ld.%02ldV", rms_mv / 1000, (rms_mv % 1000) / 10);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(5, 20, (uint8_t *)buffer, LEFT_MODE);
}

void Draw_Vpp(volatile uint16_t measurements[BUFFER_SIZE]) {
  char buffer[32];
  uint16_t Vpp_raw = Calculate_Vpp(measurements);
  int32_t vpp_mv = convert_scale_only(Vpp_raw);
  sprintf(buffer, "Vpp:%2ld.%02ldV", vpp_mv / 1000, (vpp_mv % 1000) / 10);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)buffer, LEFT_MODE);
}

void Draw_Buffer(uint16_t *buffer, uint32_t color) {
  static uint16_t old_buffer[MAX_WIDTH];

  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  for (int i = 1; i < MAX_WIDTH; i++) {
    BSP_LCD_DrawLine(i - 1, MAX_HEIGHT - old_buffer[i - 1], i, MAX_HEIGHT - old_buffer[i]);
  }

  BSP_LCD_SetTextColor(color);
  for (int i = 1; i < MAX_WIDTH; i++) {
    BSP_LCD_DrawLine(i - 1, MAX_HEIGHT - buffer[i - 1], i, MAX_HEIGHT - buffer[i]);
    old_buffer[i - 1] = buffer[i - 1];
  }
  old_buffer[MAX_WIDTH - 1] = buffer[MAX_WIDTH - 1];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC3) {
    adc_ready = 1;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3) {
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
    HAL_ADC_Stop_DMA(hadc);
    HAL_ADC_Start_DMA(hadc, (uint32_t*)measurementData, BUFFER_SIZE);
  }
}

void Draw_Full_Menu(void) {
  BSP_LCD_Clear(LCD_COLOR_LIGHTGRAY);

  BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGRAY);
  BSP_LCD_SetTextColor(LCD_COLOR_ALMOST_BLACK);
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"USTAWIENIA", CENTER_MODE);

  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Wybierz opcje:", CENTER_MODE);

  if (show_vpp) {
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
    BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
  }
  BSP_LCD_FillRect(MENU_COL1_X, MENU_ROW1_Y, MENU_BTN_W, MENU_BTN_H);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font16);
  if (show_vpp) BSP_LCD_DisplayStringAt(MENU_COL1_X + 20, MENU_ROW1_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Vpp:ON", LEFT_MODE);
  else          BSP_LCD_DisplayStringAt(MENU_COL1_X + 15, MENU_ROW1_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Vpp:OFF", LEFT_MODE);

  if (show_rms) {
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
    BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
  }
  BSP_LCD_FillRect(MENU_COL2_X, MENU_ROW1_Y, MENU_BTN_W, MENU_BTN_H);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  if (show_rms) BSP_LCD_DisplayStringAt(MENU_COL2_X + 20, MENU_ROW1_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"RMS:ON", LEFT_MODE);
  else          BSP_LCD_DisplayStringAt(MENU_COL2_X + 15, MENU_ROW1_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"RMS:OFF", LEFT_MODE);

  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
  BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
  BSP_LCD_FillRect(MENU_COL1_X, MENU_ROW2_Y, MENU_BTN_W, MENU_BTN_H);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(MENU_COL1_X + 30, MENU_ROW2_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Hz", LEFT_MODE);

  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
  BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
  BSP_LCD_FillRect(MENU_COL2_X, MENU_ROW2_Y, MENU_BTN_W, MENU_BTN_H);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(MENU_COL2_X + 30, MENU_ROW2_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"FFT", LEFT_MODE);

  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
  BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
  BSP_LCD_FillRect(MENU_COL1_X, MENU_ROW3_Y, MENU_BTN_W, MENU_BTN_H);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(MENU_COL1_X + 15, MENU_ROW3_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Kursory", LEFT_MODE);

  BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
  BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
  BSP_LCD_FillRect(MENU_COL2_X, MENU_ROW3_Y, MENU_BTN_W, MENU_BTN_H);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(MENU_COL2_X + 15, MENU_ROW3_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Trigger", LEFT_MODE);

  BSP_LCD_SetBackColor(LCD_COLOR_LIGHTGRAY);
  BSP_LCD_SetTextColor(LCD_COLOR_ALMOST_BLACK);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, 280, (uint8_t *)"Wcisnij przycisk USER", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 300, (uint8_t *)"aby wrocic", CENTER_MODE);
  BSP_LCD_SetBackColor(LCD_COLOR_ALMOST_BLACK);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
