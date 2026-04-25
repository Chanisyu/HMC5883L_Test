/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "HMC5883L.h"
#include "stdio.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HMC_CAL_SAMPLE_COUNT 1500U
#define HMC_CAL_SAMPLE_DELAY_MS 20U
#define OLED_PAGE_SWITCH_MS 2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t hmc_update_flag = 0;
HMC5883L_Calib_t hmc_calib = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void FormatSignedFixed2(char *buffer, size_t buffer_size, float value);
static void OLED_ShowLabelValue(uint8_t line, const char *label, float value);
static void OLED_ShowCalibrationPage(uint8_t page);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void FormatSignedFixed2(char *buffer, size_t buffer_size, float value)
{
	int32_t scaled;
	int32_t integer_part;
	int32_t fraction_part;

	scaled = (int32_t)(value * 100.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
	integer_part = scaled / 100;
	fraction_part = scaled % 100;
	if (fraction_part < 0)
	{
		fraction_part = -fraction_part;
	}

	snprintf(buffer, buffer_size, "%ld.%02ld", (long)integer_part, (long)fraction_part);
}

static void OLED_ShowLabelValue(uint8_t line, const char *label, float value)
{
	char text[16];
	char value_text[12];

	FormatSignedFixed2(value_text, sizeof(value_text), value);
	snprintf(text, sizeof(text), "%s%s", label, value_text);
	OLED_ClearLine(line);
	OLED_ShowString(line, 1, text);
}

static void OLED_ShowCalibrationPage(uint8_t page)
{
	if (page == 0U)
	{
		OLED_ShowLabelValue(1, "Ox:", hmc_calib.offset_x);
		OLED_ShowLabelValue(2, "Oy:", hmc_calib.offset_y);
		OLED_ShowLabelValue(3, "Oz:", hmc_calib.offset_z);
	}
	else
	{
		OLED_ShowLabelValue(1, "Sx:", hmc_calib.scale_x);
		OLED_ShowLabelValue(2, "Sy:", hmc_calib.scale_y);
		OLED_ShowLabelValue(3, "Sz:", hmc_calib.scale_z);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		hmc_update_flag = 1;
	}
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
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	OLED_ShowString(1,1,"Calibrating...");
	OLED_ShowString(2,1,"Rotate sensor");
	OLED_ShowString(3,1,"all directions");
	if (HMC5883L_Init() != HAL_OK)
	{
		OLED_Clear();
		OLED_ShowString(1,1,"HMC Init Fail");
		while (1)
		{
		}
	}

	if (HMC5883L_Calibrate(&hmc_calib, HMC_CAL_SAMPLE_COUNT, HMC_CAL_SAMPLE_DELAY_MS) != HAL_OK)
	{
		OLED_Clear();
		OLED_ShowString(1,1,"Calib Failed");
		OLED_ShowString(2,1,"Move sensor");
		OLED_ShowString(3,1,"more widely");
		while (1)
		{
		}
	}

	OLED_Clear();
	OLED_ShowString(1,1,"Cal Done");
	OLED_ShowString(2,1,"Showing vals");
	HAL_Delay(1500);
	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint8_t page;

		if (hmc_update_flag)
		{
			float hmc_x_cal;
			float hmc_y_cal;

			hmc_update_flag = 0;
			if (HMC5883L_GetData(&hmc_x, &hmc_y, &hmc_z) == HAL_OK)
			{
				hmc_x_cal = ((float)hmc_x - hmc_calib.offset_x) * hmc_calib.scale_x;
				hmc_y_cal = ((float)hmc_y - hmc_calib.offset_y) * hmc_calib.scale_y;
//				hmc_x_cal = ((float)hmc_x+15) * 1.05;
//				hmc_y_cal = ((float)hmc_y+285) * 1.00;
				yaw_hmc = atan2f(hmc_y_cal, hmc_x_cal) * 57.2957795f;
			}
		}

		page = ((HAL_GetTick() / OLED_PAGE_SWITCH_MS) % 2U == 0U) ? 0U : 1U;
		OLED_ShowCalibrationPage(page);
		OLED_ShowLabelValue(4, "Yaw:", yaw_hmc);
		HAL_Delay(120);
    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
