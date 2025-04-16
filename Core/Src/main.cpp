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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stm32g4xx_hal_flash_ex.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Integer.h"
#include "Measure.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static Integer Int;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us_DWT(unsigned long  uSec)
{
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}

void delay_ms_DWT(unsigned long  mSec)
{
	delay_us_DWT(mSec * 1000);
}

#define SLOTS 4
static volatile uint32_t start[4];

void start_us_DWT(int slot)
{
	if (slot < SLOTS) {
		start[slot] = DWT->CYCCNT;
	}
}

unsigned int get_us_DWT(int slot)
{
	if (slot < SLOTS) {
		unsigned long long val;
		if (DWT->CYCCNT < start[slot]) {
			val = (DWT->CYCCNT - start[slot]) + (1LL << 32);
		} else {
			val = (DWT->CYCCNT - start[slot]);
		}
		return val * 1000000L / SystemCoreClock;
	}
	return 0xFFFFFFFF;
}

void doPin(GPIO_TypeDef* port, uint16_t pin){
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	delay_ms_DWT(1000);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	delay_ms_DWT(1000);
}
void doPinFast(GPIO_TypeDef* port, uint16_t pin){
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	delay_us_DWT(10);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	delay_us_DWT(1);
}

volatile uint32_t valueGlobal;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
  // Enable cycle counter
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  MX_HRTIM1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //HAL_OPAMP_Start(&hopamp3);


#if 1
  FLASH_OBProgramInitTypeDef obInit = {0};
  HAL_StatusTypeDef status;
  obInit.OptionType = OPTIONBYTE_USER;
  obInit.USERType = OB_USER_nSWBOOT0;
  obInit.USERConfig = 0;// FLASH_OPTR_nSWBOOT0;
  //HAL_FLASH_OB_Unlock();
  status = HAL_FLASHEx_OBProgram(&obInit);
  uint32_t status2 = HAL_FLASH_GetError();
  //FLASH_OB_UserConfig(OB_USER_nSWBOOT0, FLASH_OPTR_nSWBOOT0);
  //FLASH_OB_UserConfig(OB_USER_nBOOT0, FLASH_OPTR_nBOOT0);
#endif

  FLASH_OBProgramInitTypeDef obInitRead = {0};
  HAL_FLASHEx_OBGetConfig(&obInitRead);


  int statusTransmit;
  char str[] = "start\n\r";
  printf("start SWV\n");
  statusTransmit = HAL_UART_Transmit(&huart2,(uint8_t*)str, strlen(str),1000);

#define intro_message "flash BYTES value:"
  statusTransmit = HAL_UART_Transmit(&huart2,(uint8_t*)intro_message, strlen(intro_message),1000);

  char * buffer;
  buffer = (char*)malloc(11);
  Int.toAXn(obInitRead.USERConfig, buffer, 11, true);
  statusTransmit = HAL_UART_Transmit(&huart2,(uint8_t*)buffer, strlen(buffer),1000);

#define eol_message "\n\r"
  statusTransmit = HAL_UART_Transmit(&huart2,(uint8_t*)eol_message, strlen(eol_message),1000);

#define start_message "adc value:"
  statusTransmit = HAL_UART_Transmit(&huart2,(uint8_t*)start_message, strlen(start_message),1000);


  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

	// tim2 for ADC DMA
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PA0
	// tim2 for ADC DMA
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PA0

  // control timer for tim2
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1); // PA8
  HAL_HRTIM_SimpleOCStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1);

  // switch timer
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1); // PC8
  HAL_HRTIM_SimpleOCStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1);

  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*) g_ADCBufferM, ADC_BUFFERM_LENGTH);


  //HAL_SuspendTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		HAL_GPIO_WritePin(T_PC13_GPIO_Port, T_PC13_Pin, GPIO_PIN_SET); // start at pin 2
//		delay_us_DWT(10);
//		HAL_GPIO_WritePin(T_PC13_GPIO_Port, T_PC13_Pin, GPIO_PIN_RESET); // start at pin 2
//		doPinFast(T_PC0_GPIO_Port, T_PC0_Pin);

		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET); // start at pin PA10
		delay_us_DWT(10);
		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET); // start at pin PA10

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // start at pin 2
		delay_us_DWT(20);



		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // start at pin 2
		delay_us_DWT(20);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
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
