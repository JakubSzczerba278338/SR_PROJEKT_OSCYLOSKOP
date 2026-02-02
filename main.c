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
#include <stdlib.h>
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
#define BUFFER_SIZE 8000
#define ADC_RESOLUTION 4095

#define REAL_RANGE_MV 20000
#define REAL_MIN_MV  -10000
#define SCREEN_RANGE_MV 24000
#define SCREEN_MIN_MV -12000

#define ADC_SAMPLE_RATE 1400000

#define SYSTEM_VREF_MV 3000
#define ACTUAL_BIAS_MV 1400

#define IDEAL_BIAS_MV (SYSTEM_VREF_MV / 2)
#define OFFSET_MV (IDEAL_BIAS_MV - ACTUAL_BIAS_MV)

#define ADC_OFFSET_CORRECTION ((OFFSET_MV * ADC_RESOLUTION) / SYSTEM_VREF_MV)

#define LCD_FRAME_BUFFER_LAYER0 LCD_FRAME_BUFFER
#define LCD_FRAME_BUFFER_LAYER1  (LCD_FRAME_BUFFER + 0x50000)
#define LCD_COLOR_ALMOST_BLACK 0xFF010101

#define MENU_BTN_W 100
#define MENU_BTN_H 60
#define MENU_MARGIN_X 15
#define MENU_GAP_X 10
#define MENU_GAP_Y 10
#define MENU_START_Y 60

#define MENU_COL1_X MENU_MARGIN_X
#define MENU_COL2_X (MENU_MARGIN_X + MENU_BTN_W + MENU_GAP_X)

#define MENU_ROW1_Y MENU_START_Y
#define MENU_ROW2_Y (MENU_START_Y + MENU_BTN_H + MENU_GAP_Y)
#define MENU_ROW3_Y (MENU_START_Y + 2 * (MENU_BTN_H + MENU_GAP_Y))

#define MENU_TEXT_OFFSET_Y  20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t measurementData[2 * BUFFER_SIZE];

volatile uint8_t buf_half_ready = 0;
volatile uint8_t buf_full_ready = 0;
volatile uint8_t menu_visible = 0;
uint8_t button_prev_state = 0;
TS_StateTypeDef TS_State;

uint8_t show_vpp = 1;
uint8_t show_rms = 1;
uint8_t show_hz = 1;
uint8_t cursor_mode = 0;
uint16_t cursor1_x = 80;
uint16_t cursor2_x = 160;

uint8_t trigger_auto = 1;


uint8_t trigger_locked = 0;
uint16_t current_trig_level = 2048;
uint8_t show_fft_view = 0;

/* FFT Buffers in CCMRAM (64KB available) */
/* 8192 floats * 4 bytes * 2 arrays = 64KB */
#define FFT_SAMPLES 8192
__attribute__((section(".ccmram"))) float fft_real[FFT_SAMPLES];
__attribute__((section(".ccmram"))) float fft_imag[FFT_SAMPLES];

/* Twiddle Factors for 8192-point FFT (N/2 = 4096) */
/* Placed in main RAM (32KB), as CCMRAM is full with fft buffers */
#define TWIDDLE_SIZE (FFT_SAMPLES / 2)
float twiddle_real[TWIDDLE_SIZE];
float twiddle_imag[TWIDDLE_SIZE];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
int32_t convert(uint32_t AdcValue);
uint16_t calculate_position(uint16_t value);
void Draw_Buffer(uint16_t *buffer, uint32_t color);
int Find_Trigger_Index(volatile uint16_t *data, uint16_t level, int limit, int hysteresis);
void Draw_Vpp(volatile uint16_t measurements[BUFFER_SIZE]);
void Draw_RMS(volatile uint16_t measurements[BUFFER_SIZE]);
void Draw_Freq(float freq);
void Draw_Cursor_Info(uint16_t x1, uint16_t val1, uint16_t x2, uint16_t val2, uint8_t mode);
float Calculate_Frequency(volatile uint16_t *data, uint16_t level, int hysteresis);

void Draw_Grid(void);
void Draw_FFT_Grid(void);
void Draw_Full_Menu(void);
void Draw_FFT_View(void);
void Init_FFT(void);
void Compute_FFT(float *vReal, float *vImag, uint16_t n);
/* USER CODE END PFP */

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

  Init_FFT(); /* Precompute Twiddle Factors */
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
  
  hadc3.DMA_Handle->Init.Mode = DMA_CIRCULAR;
  if (HAL_DMA_Init(hadc3.DMA_Handle) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure ADC for 3 Cycles Sampling Time (1.4 MSPS) */
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfig.Offset = 0;


  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start ADC in Circular Mode covering both halves of the buffer */
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)measurementData, 2 * BUFFER_SIZE);


  while (1)
  {
    uint8_t button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if (button_state == GPIO_PIN_SET && button_prev_state == GPIO_PIN_RESET) {
        /* USER Button: Always toggle Menu visibility */
        menu_visible = !menu_visible;
        if (menu_visible) {
            Draw_Full_Menu();
        } else {
            /* Exiting Menu - Draw appropriate background on Layer 0 */
            BSP_LCD_SelectLayer(0);
            BSP_LCD_Clear(LCD_COLOR_BLACK);
            
            if (show_fft_view) {
                /* FFT Mode - Draw FFT Grid on Layer 0 */
                Draw_FFT_Grid();
            } else {
                /* Scope Mode - Draw Scope Grid on Layer 0 */
                Draw_Grid();
            }
            /* Always restart ADC DMA (needed for both Scope and FFT) */
            HAL_ADC_Start_DMA(&hadc3, (uint32_t*)measurementData, 2 * BUFFER_SIZE);

            
            BSP_LCD_SelectLayer(1);
            BSP_LCD_Clear(LCD_COLOR_BLACK);
        }
        HAL_Delay(50);
    }

    button_prev_state = button_state;
    
    uint8_t redraw_needed = 0;

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
        else if (x >= MENU_COL1_X && x <= (MENU_COL1_X + MENU_BTN_W) &&
                 y >= MENU_ROW2_Y && y <= (MENU_ROW2_Y + MENU_BTN_H)) {
           /* Hz Button */
           show_hz = !show_hz;
           Draw_Full_Menu();
           HAL_Delay(200);
        }
        else if (x >= MENU_COL2_X && x <= (MENU_COL2_X + MENU_BTN_W) &&
                 y >= MENU_ROW2_Y && y <= (MENU_ROW2_Y + MENU_BTN_H)) {
           /* FFT Button - Toggle FFT mode */
           show_fft_view = !show_fft_view;
           Draw_Full_Menu(); /* Refresh to show new state */
           HAL_Delay(200);
        }


        else if (x >= MENU_COL2_X && x <= (MENU_COL2_X + MENU_BTN_W) &&
                 y >= MENU_ROW1_Y && y <= (MENU_ROW1_Y + MENU_BTN_H)) {
          show_rms = !show_rms;
          Draw_Full_Menu();
          HAL_Delay(200);
        }
        else if (x >= MENU_COL1_X && x <= (MENU_COL1_X + MENU_BTN_W) &&
                 y >= MENU_ROW3_Y && y <= (MENU_ROW3_Y + MENU_BTN_H)) {
           /* Kursory (Cursor) Button: OFF -> 1 -> 2 -> OFF */
           cursor_mode++;
           if (cursor_mode > 2) cursor_mode = 0;
           Draw_Full_Menu();
           HAL_Delay(200);
        }

        else if (x >= MENU_COL2_X && x <= (MENU_COL2_X + MENU_BTN_W) &&
                 y >= MENU_ROW3_Y && y <= (MENU_ROW3_Y + MENU_BTN_H)) {
           /* Trigger Button Pressed */
           trigger_auto = !trigger_auto;
           Draw_Full_Menu();
           HAL_Delay(200);
        }
      }
    }

    else {
       /* Handle touch for Cursor movement if menu hidden */
       BSP_TS_GetState(&TS_State);
       if (TS_State.TouchDetected && cursor_mode > 0) {
           if (TS_State.Y > 20) { /* Avoid top area potentially */
               uint16_t touch_x = TS_State.X;
               if (touch_x >= MAX_WIDTH) touch_x = MAX_WIDTH-1;
               
               if (cursor_mode == 1) {
                   cursor1_x = touch_x;
                   redraw_needed = 1;
               } else if (cursor_mode == 2) {

                   /* Move the closest cursor */
                   int d1 = abs(touch_x - cursor1_x);
                   int d2 = abs(touch_x - cursor2_x);
                   if (d1 <= d2) cursor1_x = touch_x;
                   else          cursor2_x = touch_x;
                   redraw_needed = 1;
               }
           }
       }
    }


    /* FFT Mode Handling */
    if (show_fft_view && !menu_visible) {
        Draw_FFT_View();
        continue; /* Skip normal scope drawing */
    }


    if (!menu_visible && (buf_half_ready || buf_full_ready || redraw_needed)) {

      volatile uint16_t *current_data_ptr;

      
      if (buf_half_ready) {
        current_data_ptr = measurementData; /* First Half */
        buf_half_ready = 0;
      } else {
        current_data_ptr = &measurementData[BUFFER_SIZE]; /* Second Half */
        buf_full_ready = 0;
      }
      
      int hysteresis = 50;
      if (trigger_auto) {
        uint16_t min_val = 4095;
        uint16_t max_val = 0;
        for(int i=0; i<BUFFER_SIZE; i++) {
           if(current_data_ptr[i] < min_val) min_val = current_data_ptr[i];
           if(current_data_ptr[i] > max_val) max_val = current_data_ptr[i];
        }
        uint16_t target_level = (min_val + max_val) / 2;
        current_trig_level = (current_trig_level * 9 + target_level) / 10;
        
        hysteresis = (max_val - min_val) / 10;
        if (hysteresis < 20) hysteresis = 20;
      } else {
        current_trig_level = 2048;
        hysteresis = 50;
      }


      int center_offset = MAX_WIDTH / 2;
      int search_start = center_offset; 
      int search_limit = BUFFER_SIZE - MAX_WIDTH - search_start; 

      int trigger_relative_idx = Find_Trigger_Index(&current_data_ptr[search_start], current_trig_level, search_limit, hysteresis);
      
      int trigger_abs_idx;
      static int last_valid_idx = 0;
      static int no_trigger_cnt = 0;

      /* Only search for trigger if we have fresh data. 
         If just redrawing cursor (redraw_needed), stick to last valid trigger to avoid jump. */
      
      if (buf_half_ready || buf_full_ready || redraw_needed) {
      }

      if (trigger_relative_idx != -1) {

        trigger_abs_idx = search_start + trigger_relative_idx;
        last_valid_idx = trigger_abs_idx;
        no_trigger_cnt = 0;
        trigger_locked = 1;
      } else {

        no_trigger_cnt++;
        if (no_trigger_cnt > 5) {
           trigger_abs_idx = center_offset;
           trigger_locked = 0;
        } else {
           trigger_abs_idx = last_valid_idx;
           trigger_locked = 1;
        }
      }
      
      if (trigger_abs_idx < center_offset) trigger_abs_idx = center_offset;
      if (trigger_abs_idx > BUFFER_SIZE - center_offset) trigger_abs_idx = center_offset;

      uint16_t displayData[MAX_WIDTH];

      for (int i = 0; i < MAX_WIDTH; i++) {
        int idx = trigger_abs_idx - center_offset + i;
        if (idx < 0) idx = 0;
        if (idx >= BUFFER_SIZE) idx = BUFFER_SIZE - 1;
        
        
        displayData[i] = calculate_position(current_data_ptr[idx]);
      }
      static uint16_t last_c1 = 0;
      static uint16_t last_c2 = 0;
      static uint16_t last_label_x = 0;
      
      if (cursor_mode > 0 && redraw_needed) {
          /* Erase old cursor LINES - full height for single, limited for dual */
          int cursor_max_y = (cursor_mode == 1) ? MAX_HEIGHT : (MAX_HEIGHT - 40);
          BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
          BSP_LCD_DrawLine(last_c1, 0, last_c1, cursor_max_y);
          if (cursor_mode == 2) {
              BSP_LCD_DrawLine(last_c2, 0, last_c2, cursor_max_y);
          }

          
          BSP_LCD_FillRect(last_c1 > 10 ? last_c1 - 10 : 0, 5, 20, 20);
          if (cursor_mode == 2) {
              BSP_LCD_FillRect(last_c2 > 10 ? last_c2 - 10 : 0, 5, 20, 20);
          }
          
          uint16_t clear_x = (last_label_x > MAX_WIDTH - 80) ? last_label_x - 85 : last_label_x + 5;
          BSP_LCD_FillRect(clear_x, 45, 90, 20);
          
          if (last_c1 > 0 && last_c1 < MAX_WIDTH - 1) {
              BSP_LCD_SetTextColor(LCD_COLOR_RED);
              BSP_LCD_DrawLine(last_c1 - 1, MAX_HEIGHT - displayData[last_c1 - 1], last_c1, MAX_HEIGHT - displayData[last_c1]);
              BSP_LCD_DrawLine(last_c1, MAX_HEIGHT - displayData[last_c1], last_c1 + 1, MAX_HEIGHT - displayData[last_c1 + 1]);
          }
          if (cursor_mode == 2 && last_c2 > 0 && last_c2 < MAX_WIDTH - 1) {
              BSP_LCD_SetTextColor(LCD_COLOR_RED);
              BSP_LCD_DrawLine(last_c2 - 1, MAX_HEIGHT - displayData[last_c2 - 1], last_c2, MAX_HEIGHT - displayData[last_c2]);
              BSP_LCD_DrawLine(last_c2, MAX_HEIGHT - displayData[last_c2], last_c2 + 1, MAX_HEIGHT - displayData[last_c2 + 1]);
          }
      }

      Draw_Buffer(displayData, LCD_COLOR_RED); 
      
      if (show_vpp) Draw_Vpp((uint16_t*)current_data_ptr);

      if (show_rms) Draw_RMS((uint16_t*)current_data_ptr);
      if (show_hz) {
          float freq = Calculate_Frequency(current_data_ptr, current_trig_level, hysteresis);
          Draw_Freq(freq);
      }
      if (cursor_mode > 0) {
          uint16_t y1 = displayData[cursor1_x];
          uint16_t y2 = displayData[cursor2_x];
          
          Draw_Cursor_Info(cursor1_x, y1, cursor2_x, y2, cursor_mode);
          
          last_c1 = cursor1_x;
          last_c2 = cursor2_x;
          last_label_x = cursor1_x;
      }
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

int Find_Trigger_Index(volatile uint16_t *data, uint16_t level, int limit, int hysteresis) {
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
  return -1;
}

int32_t convert(uint32_t AdcValue) {
  return ((int32_t)AdcValue * REAL_RANGE_MV) / ADC_RESOLUTION + REAL_MIN_MV;
}

int32_t convert_scale_only(uint32_t AdcValue) {
  return ((int32_t)AdcValue * REAL_RANGE_MV) / ADC_RESOLUTION;
}

uint16_t calculate_position(uint16_t value) {

  
  int32_t corrected_value = (int32_t)value + ADC_OFFSET_CORRECTION;
  
  int32_t voltage_mv = (corrected_value * REAL_RANGE_MV) / ADC_RESOLUTION + REAL_MIN_MV;
  int32_t voltage_from_bottom_mv = voltage_mv - SCREEN_MIN_MV;
  int32_t position = (voltage_from_bottom_mv * MAX_HEIGHT) / SCREEN_RANGE_MV;
  return (uint16_t)position;
}

void Draw_Grid(void)
{
  uint32_t originalColor = BSP_LCD_GetTextColor();
  sFONT *originalFont = BSP_LCD_GetFont();

  BSP_LCD_SetFont(&Font12);

  uint16_t center_y = MAX_HEIGHT / 2;
  uint16_t center_x = MAX_WIDTH / 2;
  int div_x = MAX_WIDTH / 10;

  float us_per_div = (24.0f * 1000000.0f) / (float)ADC_SAMPLE_RATE;

  for (int i = 1; i < 10; i++) {
    uint16_t x = i * div_x;
    int div_offset = i - 5; /* -4, -3, ... 0 ... +4 */
    int time_us = (int)(div_offset * us_per_div);

    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawLine(x, 0, x, MAX_HEIGHT);

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawLine(x, center_y - 3, x, center_y + 3);

    if (div_offset % 2 == 0) {
        char label[8];
        if (time_us == 0) sprintf(label, "0");
        else sprintf(label, "%dus", time_us);
        
        if (div_offset == 0) {
          BSP_LCD_DisplayStringAt(x + 4, center_y + 6, (uint8_t *)label, LEFT_MODE);
        } else {
          BSP_LCD_DisplayStringAt(x - 10, center_y + 6, (uint8_t *)label, LEFT_MODE);
        }
    }
  }
  
  char tb_label[16];
  sprintf(tb_label, "%.1fus/div", us_per_div);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(MAX_WIDTH - 80, MAX_HEIGHT - 15, (uint8_t *)tb_label, LEFT_MODE);


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

/* FFT Background Grid - Frequency Scale (0-100 kHz) */
void Draw_FFT_Grid(void)
{
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  
  BSP_LCD_SetColorKeying(0, LCD_COLOR_BLACK);
  
  BSP_LCD_SetFont(&Font8);
  
  float freq_per_px = 100000.0f / (float)MAX_WIDTH;
  
  BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
  
  int x_25k = (int)(25000 / freq_per_px);
  BSP_LCD_DrawLine(x_25k, 20, x_25k, MAX_HEIGHT - 25);
  
  int x_50k = (int)(50000 / freq_per_px);
  BSP_LCD_DrawLine(x_50k, 20, x_50k, MAX_HEIGHT - 25);
  
  int x_75k = (int)(75000 / freq_per_px);
  BSP_LCD_DrawLine(x_75k, 20, x_75k, MAX_HEIGHT - 25);
  
  int draw_height = MAX_HEIGHT - 50;
  BSP_LCD_DrawLine(0, MAX_HEIGHT - 26 - draw_height/4, MAX_WIDTH, MAX_HEIGHT - 26 - draw_height/4);
  BSP_LCD_DrawLine(0, MAX_HEIGHT - 26 - draw_height/2, MAX_WIDTH, MAX_HEIGHT - 26 - draw_height/2);
  BSP_LCD_DrawLine(0, MAX_HEIGHT - 26 - 3*draw_height/4, MAX_WIDTH, MAX_HEIGHT - 26 - 3*draw_height/4);
  

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DrawLine(0, MAX_HEIGHT - 25, MAX_WIDTH, MAX_HEIGHT - 25);
  
  /* Frequency Labels (well positioned) */
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(5, MAX_HEIGHT - 18, (uint8_t *)"0", LEFT_MODE);
  BSP_LCD_DisplayStringAt(x_25k - 8, MAX_HEIGHT - 18, (uint8_t *)"25k", LEFT_MODE);
  BSP_LCD_DisplayStringAt(x_50k - 8, MAX_HEIGHT - 18, (uint8_t *)"50k", LEFT_MODE);
  BSP_LCD_DisplayStringAt(x_75k - 8, MAX_HEIGHT - 18, (uint8_t *)"75k", LEFT_MODE);
  BSP_LCD_DisplayStringAt(MAX_WIDTH - 35, MAX_HEIGHT - 18, (uint8_t *)"100k", LEFT_MODE);
  
  /* Title */
  BSP_LCD_DisplayStringAt(5, 5, (uint8_t *)"FFT [Hz]", LEFT_MODE);
  
  BSP_LCD_SelectLayer(1);
}

uint16_t Calculate_Vpp(volatile uint16_t measurements[BUFFER_SIZE]) {
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

float Calculate_Frequency(volatile uint16_t *data, uint16_t level, int hysteresis) {
    /* Find first edge */
    int idx1 = Find_Trigger_Index(data, level, BUFFER_SIZE/2, hysteresis);
    if (idx1 == -1) return 0.0f;
    
    /* Find second edge after the first one + small holdoff */
    /* Holdoff prevents re-triggering on noise immediately after the edge.
       However, it was 20 samples, which at 1.4MSPS is ~14us (70kHz).
       Signals >70kHz have period <14us, so we were skipping a full cycle!
       Reduced to 5 samples (~3.5us) to support up to ~280kHz. */
    int holdoff = 5; 
    if (idx1 + holdoff >= BUFFER_SIZE) return 0.0f;
    
    int idx2 = Find_Trigger_Index(&data[idx1 + holdoff], level, BUFFER_SIZE - (idx1 + holdoff), hysteresis);
    if (idx2 == -1) return 0.0f;
    
    idx2 += (idx1 + holdoff); /* Convert relative to absolute */
    
    int period_samples = idx2 - idx1;
    if (period_samples == 0) return 0.0f;
    
    return (float)ADC_SAMPLE_RATE / period_samples;
}

void Draw_Freq(float freq) {
  char buffer[32];
  
  /* Clear previous value area to avoid artifacts */
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK); 
  BSP_LCD_FillRect(50, 35, 100, 16); 

  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetFont(&Font12);
  
  /* Draw Label fixed */
  BSP_LCD_DisplayStringAt(5, 35, (uint8_t *)"Freq:", LEFT_MODE);
  
  /* Draw Value at fixed offset (x=50) */
  if (freq > 0) {
      if (freq >= 1000) sprintf(buffer, "%.2f kHz", freq / 1000.0f);
      else              sprintf(buffer, "%d Hz", (int)freq);
  } else {
      sprintf(buffer, "---");
  }
  BSP_LCD_DisplayStringAt(50, 35, (uint8_t *)buffer, LEFT_MODE);
}

void Draw_Cursor_Info(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t mode) {
    char buffer[64];
    
    /* Convert Y position (pixels from bottom) back to voltage */
    /* Inverse of calculate_position: voltage = (y * SCREEN_RANGE / HEIGHT) + SCREEN_MIN */
    #define Y_TO_MV(y) ( (((int32_t)(y) * SCREEN_RANGE_MV) / MAX_HEIGHT) + SCREEN_MIN_MV )
    
    int mv1 = Y_TO_MV(y1);


    /* Draw C1 - full height for single mode, limited for dual */
    int cursor_h = (mode == 1) ? MAX_HEIGHT : (MAX_HEIGHT - 40);
    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
    BSP_LCD_DrawLine(x1, 0, x1, cursor_h);

    
    float us_per_px = 1000000.0f / (float)ADC_SAMPLE_RATE;
    
    /* Draw Info Box */
    BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font12);

    if (mode == 1) {
        /* Single Cursor Info - Fix negative number formatting */
        int abs_mv = (mv1 < 0) ? -mv1 : mv1;
        char sign = (mv1 < 0) ? '-' : ' ';
        sprintf(buffer, "C1:%c%d.%02dV", sign, abs_mv/1000, (abs_mv%1000)/10);
        uint16_t tx = (x1 > MAX_WIDTH - 80) ? x1 - 85 : x1 + 5;
        BSP_LCD_DisplayStringAt(tx, 50, (uint8_t *)buffer, LEFT_MODE);
    } 

    else if (mode == 2) {
        int mv2 = Y_TO_MV(y2);

        
        /* Draw C2 - limit height to avoid info box */
        BSP_LCD_SetTextColor(LCD_COLOR_MAGENTA); /* Second cursor different color */
        BSP_LCD_DrawLine(x2, 0, x2, MAX_HEIGHT - 40);

        
        /* Delta Calculations */
        int d_mv = abs(mv1 - mv2);
        int d_px = abs(x1 - x2);
        float d_us = d_px * us_per_px;

        /* Display C1 & C2 Values near cursors? Or just Deltas at bottom/top?
           Let's put Deltas in a dedicated area to avoid clutter */
           
        /* Draw Background for Info */
        BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
        BSP_LCD_FillRect(0, MAX_HEIGHT-40, MAX_WIDTH, 40);
        
        BSP_LCD_SetBackColor(LCD_COLOR_DARKGRAY);
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
        
        /* Line 1: Volts - Fix negative number formatting */
        int abs_mv1 = (mv1 < 0) ? -mv1 : mv1;
        int abs_mv2 = (mv2 < 0) ? -mv2 : mv2;
        char s1 = (mv1 < 0) ? '-' : '+';
        char s2 = (mv2 < 0) ? '-' : '+';
        sprintf(buffer, "V1:%c%d.%02d V2:%c%d.%02d dV:%d.%02d", 
                s1, abs_mv1/1000, (abs_mv1%1000)/10, 
                s2, abs_mv2/1000, (abs_mv2%1000)/10, 
                d_mv/1000, (d_mv%1000)/10);
        BSP_LCD_DisplayStringAt(5, MAX_HEIGHT-35, (uint8_t *)buffer, LEFT_MODE);

        
        /* Line 2: Time (No Frequency in Dual Mode as requested) */
        sprintf(buffer, "dT:%.1fus", d_us);

        BSP_LCD_DisplayStringAt(5, MAX_HEIGHT-20, (uint8_t *)buffer, LEFT_MODE);

        
        /* Also label cursors lightly */
        BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
        BSP_LCD_DisplayChar(x1 > 10 ? x1-5 : x1+2, 10, '1');
        BSP_LCD_SetTextColor(LCD_COLOR_MAGENTA);
        BSP_LCD_DisplayChar(x2 > 10 ? x2-5 : x2+2, 10, '2');
    }
}

void Draw_Buffer(uint16_t *buffer, uint32_t color) {
  static uint16_t old_buffer[MAX_WIDTH];

  for (int i = 1; i < MAX_WIDTH; i++) {
    /* Erase old segment */
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawLine(i - 1, MAX_HEIGHT - old_buffer[i - 1], i, MAX_HEIGHT - old_buffer[i]);

    /* Draw new segment */
    BSP_LCD_SetTextColor(color);
    BSP_LCD_DrawLine(i - 1, MAX_HEIGHT - buffer[i - 1], i, MAX_HEIGHT - buffer[i]);

    /* Update history */
    old_buffer[i - 1] = buffer[i - 1];
  }
  old_buffer[MAX_WIDTH - 1] = buffer[MAX_WIDTH - 1];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC3) {
    buf_full_ready = 1;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC3) {
    buf_half_ready = 1;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC3) {
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
    HAL_ADC_Stop_DMA(hadc);
    HAL_ADC_Start_DMA(hadc, (uint32_t*)measurementData, 2 * BUFFER_SIZE);
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
  
  if (show_hz) {
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
    BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
  }
  
  BSP_LCD_FillRect(MENU_COL1_X, MENU_ROW2_Y, MENU_BTN_W, MENU_BTN_H);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  if (show_hz) BSP_LCD_DisplayStringAt(MENU_COL1_X + 20, MENU_ROW2_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Hz:ON", LEFT_MODE);
  else         BSP_LCD_DisplayStringAt(MENU_COL1_X + 20, MENU_ROW2_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Hz:OFF", LEFT_MODE);

  /* FFT Button (ROW2 COL2) */
  if (show_fft_view) {
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
    BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
  }
  BSP_LCD_FillRect(MENU_COL2_X, MENU_ROW2_Y, MENU_BTN_W, MENU_BTN_H);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  if (show_fft_view) BSP_LCD_DisplayStringAt(MENU_COL2_X + 15, MENU_ROW2_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"FFT:ON", LEFT_MODE);
  else               BSP_LCD_DisplayStringAt(MENU_COL2_X + 15, MENU_ROW2_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"FFT:OFF", LEFT_MODE);

  
  /* Cursor Button (ROW3 COL1) */
  if (cursor_mode > 0) {
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
    BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);
  } else {
    BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
    BSP_LCD_SetBackColor(LCD_COLOR_GRAY);
  }
  BSP_LCD_FillRect(MENU_COL1_X, MENU_ROW3_Y, MENU_BTN_W, MENU_BTN_H);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  
  if (cursor_mode == 0)      BSP_LCD_DisplayStringAt(MENU_COL1_X + 15, MENU_ROW3_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Cur:OFF", LEFT_MODE);
  else if (cursor_mode == 1) BSP_LCD_DisplayStringAt(MENU_COL1_X + 15, MENU_ROW3_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Cur:1", LEFT_MODE);
  else                       BSP_LCD_DisplayStringAt(MENU_COL1_X + 15, MENU_ROW3_Y + MENU_TEXT_OFFSET_Y, (uint8_t *)"Cur:2", LEFT_MODE);

  if (trigger_auto) {
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
    BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);
  } else {
     /* Manual Mode Color */
    BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
    BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);
  }
  BSP_LCD_FillRect(MENU_COL2_X, MENU_ROW3_Y, MENU_BTN_W, MENU_BTN_H);
  
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  if (trigger_auto) BSP_LCD_SetBackColor(LCD_COLOR_DARKGREEN);
  else              BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);

  BSP_LCD_DisplayStringAt(MENU_COL2_X + 15, MENU_ROW3_Y + 15, (uint8_t *)"Trigger", LEFT_MODE);
  
  if (trigger_auto) BSP_LCD_DisplayStringAt(MENU_COL2_X + 25, MENU_ROW3_Y + 35, (uint8_t *)"Auto", LEFT_MODE);
  else              BSP_LCD_DisplayStringAt(MENU_COL2_X + 25, MENU_ROW3_Y + 35, (uint8_t *)"Man", LEFT_MODE);

  /* Lock Status Indicator */
  if (trigger_locked) BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  else                BSP_LCD_SetTextColor(LCD_COLOR_RED);
  BSP_LCD_FillCircle(MENU_COL2_X + MENU_BTN_W - 10, MENU_ROW3_Y + 10, 4);


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
/* ================= FFT OPTIMIZED IMPLEMENTATION ================= */
/* Optimized Radix-2 DIT FFT using Precomputed Twiddle Factors */

void Init_FFT(void) {
    /* Precompute Twiddle Factors: W_N^k = exp(-j * 2*pi * k / N) */
    /* k goes from 0 to N/2 - 1 */
    for (int k = 0; k < TWIDDLE_SIZE; k++) {
        float angle = -6.283185307f * (float)k / (float)FFT_SAMPLES;
        twiddle_real[k] = cosf(angle);
        twiddle_imag[k] = sinf(angle);
    }
}

void Compute_FFT(float *vReal, float *vImag, uint16_t n) {
    /* Bit reversal (stable implementation) */
    uint16_t j = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (i < j) {
            float temp = vReal[i]; vReal[i] = vReal[j]; vReal[j] = temp;
            temp = vImag[i]; vImag[i] = vImag[j]; vImag[j] = temp;
        }
        uint16_t bit = n >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
    }

    /* Butterfly Calculation using LUT */
    /* M = 1, 2, 4... N/2. */
    /* Stride = N / (2*M) */
    
    for (uint16_t m = 1; m < n; m *= 2) {
        uint16_t stride = n / (2 * m);
        
        for (uint16_t i = 0; i < n; i += 2 * m) {
            for (uint16_t k = 0; k < m; k++) {
                uint16_t tf_idx = k * stride;
                float w_r = twiddle_real[tf_idx];
                float w_i = twiddle_imag[tf_idx];
                
                float t_r = w_r * vReal[i + k + m] - w_i * vImag[i + k + m];
                float t_i = w_r * vImag[i + k + m] + w_i * vReal[i + k + m];
                
                float u_r = vReal[i + k];
                float u_i = vImag[i + k];
                
                vReal[i + k] = u_r + t_r;
                vImag[i + k] = u_i + t_i;
                vReal[i + k + m] = u_r - t_r;
                vImag[i + k + m] = u_i - t_i;
            }
        }
    }
}


void Draw_FFT_View(void) {
    if (!buf_full_ready && !buf_half_ready) return;
    
    volatile uint16_t *src;
    if (buf_half_ready) src = measurementData;
    else src = &measurementData[BUFFER_SIZE];
    
    for(int i=0; i<FFT_SAMPLES; i++) {
        if (i < BUFFER_SIZE) {
            fft_real[i] = (float)src[i];
        } else {
            fft_real[i] = 0.0f;
        }
        fft_imag[i] = 0.0f;
    }
    
    float sum = 0;
    for(int i=0; i<8000; i++) sum += fft_real[i];
    float mean = sum / 8000.0f;
    for(int i=0; i<8000; i++) fft_real[i] -= mean;

    /* 2. Compute FFT */
    Compute_FFT(fft_real, fft_imag, FFT_SAMPLES);
    
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    
    #define FFT_MAX_FREQ 100000
    #define FFT_FREQ_PER_BIN (ADC_SAMPLE_RATE / FFT_SAMPLES)
    int max_bin = FFT_MAX_FREQ / FFT_FREQ_PER_BIN;
    if (max_bin > FFT_SAMPLES/2) max_bin = FFT_SAMPLES/2;
    
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    
    float max_mag = 0;
    /* Find Max Magnitude (only up to max_bin) */
    for(int i=1; i<max_bin; i++) {
        float mag = sqrtf(fft_real[i]*fft_real[i] + fft_imag[i]*fft_imag[i]);
        fft_real[i] = mag;
        if (mag > max_mag) max_mag = mag;
    }
    if (max_mag < 1.0f) max_mag = 1.0f;
    
    /* Draw Spectrum - map 0..max_bin to 0..MAX_WIDTH */
    float bins_per_pixel = (float)max_bin / (float)MAX_WIDTH;
    
    for (int x = 0; x < MAX_WIDTH; x++) {
        float local_max = 0;
        int start_bin = (int)(x * bins_per_pixel);
        int end_bin = (int)((x + 1) * bins_per_pixel);
        if (end_bin > max_bin) end_bin = max_bin;
        
        for (int b = start_bin; b < end_bin; b++) {
            if (b > 0 && fft_real[b] > local_max) local_max = fft_real[b];
        }
        
        int bar_h = (int)((local_max / max_mag) * (MAX_HEIGHT - 50));
        if (bar_h > MAX_HEIGHT - 50) bar_h = MAX_HEIGHT - 50;
        
        if (bar_h > 0) {
            BSP_LCD_DrawLine(x, MAX_HEIGHT - 26, x, MAX_HEIGHT - 26 - bar_h);
        }
    }
    
    /* Clear flags */
    buf_full_ready = 0; 
    buf_half_ready = 0;
}


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
