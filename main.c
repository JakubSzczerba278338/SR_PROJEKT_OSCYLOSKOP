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
  *                             www.st.com/SLA0044
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
#include <stdio.h>
#include <math.h>
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
int32_t convert(uint32_t);
uint16_t calculate_position(uint16_t);
void Draw_Buffer(uint16_t [BUFFER_SIZE], uint32_t color);
int Find_Trigger_Index(volatile uint16_t *data, uint16_t level, int limit);
void Draw_Y_Axis();
void Draw_Vpp(volatile uint16_t measurements[BUFFER_SIZE]);
void Draw_RMS(volatile uint16_t measurements[BUFFER_SIZE]);
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
  BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetFont(&Font12);
  /* USER CODE END 2 */


  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADC_Start_DMA(&hadc3, measurementData, BUFFER_SIZE);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (adc_ready) {
      adc_ready = 0;
      uint16_t trigger_level = 2048;
      int search_limit = BUFFER_SIZE - MAX_WIDTH;
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

void Draw_Y_Axis(void)
{
    uint32_t originalColor = BSP_LCD_GetTextColor();
    sFONT *originalFont = BSP_LCD_GetFont();
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font12);

    BSP_LCD_DrawLine(0, AXIS_Y_POS, MAX_WIDTH, AXIS_Y_POS);

    for (int v = 0; v <= 3; v++)
    {
        uint16_t x_pos = (uint16_t)((float)v / 3.3f * MAX_WIDTH);

        if (x_pos >= MAX_WIDTH) x_pos = MAX_WIDTH - 1;

        BSP_LCD_DrawLine(x_pos, AXIS_Y_POS, x_pos, AXIS_Y_POS + TICK_HEIGHT);

        char label[5];
        sprintf(label, "%dV", v);

        if (v == 0) {
            BSP_LCD_DisplayStringAt(x_pos + 2, AXIS_Y_POS + TICK_HEIGHT + 2, (uint8_t *)label, LEFT_MODE);
        } else if (v == 3) {
             BSP_LCD_DisplayStringAt(x_pos - 20, AXIS_Y_POS + TICK_HEIGHT + 2, (uint8_t *)label, LEFT_MODE);
        } else {
            BSP_LCD_DisplayStringAt(x_pos - 6, AXIS_Y_POS + TICK_HEIGHT + 2, (uint8_t *)label, LEFT_MODE);
        }
    }

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

void Draw_Buffer(uint16_t buffer[BUFFER_SIZE], uint32_t color) {
    uint16_t x_prev = 0;
    uint16_t y_prev = 0;

    BSP_LCD_SetTextColor(color);

    for(int i = 0; i < MAX_HEIGHT; i++) {
        uint16_t x_curr = buffer[i];
        uint16_t y_curr = i;

        if (i > 0) {
            BSP_LCD_DrawLine(x_prev, y_prev, x_curr, y_curr);
        }
        x_prev = x_curr;
        y_prev = y_curr;
    }
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
    	HAL_ADC_Start_DMA(hadc, measurementData, BUFFER_SIZE);
    }
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
  *         where the assert_param error has occurred.
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
