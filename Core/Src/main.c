/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "luts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_DMA_CHANELS 4

#define TABLE_SIZE (128*1)
#define BURST_SIZE (PWM_DMA_CHANELS * TABLE_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#if (PWM_DMA_CHANELS == 3)
#define SCALE_3PHASE_AMPLITUDE scale_3phase_tables_half
//#define SCALE_3PHASE_AMPLITUDE scale_3phase_tables_zero
#define TIM_DMABURSTLENGTH_NUMBER_TRANSFERS TIM_DMABURSTLENGTH_3TRANSFERS
#elif (PWM_DMA_CHANELS == 4)
#define SCALE_4PHASE_AMPLITUDE scale_4phase_tables_half
//#define SCALE_4PHASE_AMPLITUDE scale_4phase_tables_zero
#define TIM_DMABURSTLENGTH_NUMBER_TRANSFERS TIM_DMABURSTLENGTH_4TRANSFERS
#else
#error "Unsupported number of PWM channels"
#endif

#define SQUEEZE 20 //Squeeze ratio
//#define BUILD_LUT LUT_sine_calculate
#define BUILD_LUT LUT_3sine_calculate
//#define BUILD_LUT LUT_triangle_calculate
//#define BUILD_LUT LUT_const_fill

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t pwm_amplitude;
uint16_t BurstBuffer_reference[BURST_SIZE];
uint16_t BurstBuffer_1[BURST_SIZE];
uint16_t BurstBuffer_2[BURST_SIZE];
volatile uint8_t buffer_selected = 0;
volatile int16_t u_amp, v_amp, w_amp, t_amp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  LUT_set_squeeze(SQUEEZE);

  uint16_t DataArrayU[TABLE_SIZE];
  uint16_t DataArrayV[TABLE_SIZE];
  uint16_t DataArrayW[TABLE_SIZE];
  BUILD_LUT(DataArrayU, TABLE_SIZE, &pwm_amplitude, 0,   400);
  BUILD_LUT(DataArrayV, TABLE_SIZE, &pwm_amplitude, 120, 400);
  BUILD_LUT(DataArrayW, TABLE_SIZE, &pwm_amplitude, 240, 400);

#if (PWM_DMA_CHANELS == 4)
  uint16_t DataArrayT[TABLE_SIZE];
  BUILD_LUT(DataArrayT, TABLE_SIZE, &pwm_amplitude, 90,  400);
#endif
  TIM1->ARR = pwm_amplitude;

  u_amp = 100;
  v_amp = 100;
  w_amp = 100;
  t_amp = 100;

#if (PWM_DMA_CHANELS == 3)
  merge_3phase_tables(BurstBuffer_reference, DataArrayU, DataArrayV, DataArrayW, TABLE_SIZE);
  SCALE_3PHASE_AMPLITUDE(BurstBuffer_1, BurstBuffer_reference, u_amp, v_amp, w_amp, pwm_amplitude, TABLE_SIZE);
  SCALE_3PHASE_AMPLITUDE(BurstBuffer_2, BurstBuffer_reference, u_amp, v_amp, w_amp, pwm_amplitude, TABLE_SIZE);
  TIM1->CCR4 = pwm_amplitude / 2;
#elif (PWM_DMA_CHANELS == 4)
  merge_4phase_tables(BurstBuffer_reference, DataArrayU, DataArrayV, DataArrayW, DataArrayT, TABLE_SIZE);
  SCALE_4PHASE_AMPLITUDE(BurstBuffer_1, BurstBuffer_reference, u_amp, v_amp, w_amp, t_amp, pwm_amplitude, TABLE_SIZE);
  SCALE_4PHASE_AMPLITUDE(BurstBuffer_2, BurstBuffer_reference, u_amp, v_amp, w_amp, t_amp, pwm_amplitude, TABLE_SIZE);
#endif

  if (HAL_GPIO_ReadPin(FAULT_IN_GPIO_Port, FAULT_IN_Pin) == GPIO_PIN_RESET)
  {
   	LED_FAULT_GPIO_Port->ODR &= ~LED_FAULT_Pin;
  } else {
   	LED_FAULT_GPIO_Port->ODR |= LED_FAULT_Pin;
  }
  HAL_Delay(100);

  HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start   (&htim1, TIM_CHANNEL_4);

  HAL_TIM_DMABurst_MultiWriteStart(&htim1,
		                           TIM_DMABASE_CCR1,
								   TIM_DMA_UPDATE,
								   (uint32_t*)BurstBuffer_1,
								   TIM_DMABURSTLENGTH_NUMBER_TRANSFERS,
								   BURST_SIZE);
  buffer_selected = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#define DYNAMIC_TEST 0//Dynamic test
#if DYNAMIC_TEST //Dynamic test
  int16_t tmp_amp = -100;
  typedef enum
  { UP, DOWN } dir_t;
  dir_t dir = UP;
#endif

  while (1)
  {
#if DYNAMIC_TEST //Dynamic test
	  if (dir == UP)
	  {
	     tmp_amp += 1;
	  } else {
		 tmp_amp -= 1;
	  }

	  if (tmp_amp >= 100)
	  {
		  dir = DOWN;
	  }
	  if (tmp_amp <= -100)
	  {
		  dir = UP;
	  }

	  u_amp = tmp_amp;
	  v_amp = tmp_amp;
	  w_amp = tmp_amp;
#if (PWM_DMA_CHANELS == 4)
	  t_amp = tmp_amp;
#endif
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(20);
    LED_STATUS_GPIO_Port->ODR ^= LED_STATUS_Pin;

    if (HAL_GPIO_ReadPin(FAULT_IN_GPIO_Port, FAULT_IN_Pin) == GPIO_PIN_RESET)
    {
    	LED_FAULT_GPIO_Port->ODR &= ~LED_FAULT_Pin;
    } else {
    	LED_FAULT_GPIO_Port->ODR |= LED_FAULT_Pin;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1)
    {
      GPIOB->ODR &= ~GPIO_PIN_3;


#if (PWM_DMA_CHANELS == 3)
      if (buffer_selected == 0)
      {
    	  SCALE_3PHASE_AMPLITUDE(BurstBuffer_2, BurstBuffer_reference, u_amp, v_amp, w_amp, pwm_amplitude, TABLE_SIZE);
      } else {
    	  SCALE_3PHASE_AMPLITUDE(BurstBuffer_1, BurstBuffer_reference, u_amp, v_amp, w_amp, pwm_amplitude, TABLE_SIZE);
      }
#elif (PWM_DMA_CHANELS == 4)
      if (buffer_selected == 0)
      {
    	  SCALE_4PHASE_AMPLITUDE(BurstBuffer_2, BurstBuffer_reference, u_amp, v_amp, w_amp, t_amp, pwm_amplitude, TABLE_SIZE);
      } else {
    	  SCALE_4PHASE_AMPLITUDE(BurstBuffer_1, BurstBuffer_reference, u_amp, v_amp, w_amp, t_amp, pwm_amplitude, TABLE_SIZE);
      }
#endif

      GPIOB->ODR |= GPIO_PIN_3;

    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1)
    {
      GPIOB->ODR |= GPIO_PIN_4;

      if (buffer_selected == 0)
      {
    	  HAL_TIM_DMABurst_MultiWriteStart(&htim1,
    	  		                           TIM_DMABASE_CCR1,
    	  								   TIM_DMA_UPDATE,
    	  								   (uint32_t*)BurstBuffer_1,
										   TIM_DMABURSTLENGTH_NUMBER_TRANSFERS,
    	  								   BURST_SIZE);
    	  buffer_selected = 1;
      } else {
    	  HAL_TIM_DMABurst_MultiWriteStart(&htim1,
    	  		                           TIM_DMABASE_CCR1,
    	  								   TIM_DMA_UPDATE,
    	  								   (uint32_t*)BurstBuffer_2,
										   TIM_DMABURSTLENGTH_NUMBER_TRANSFERS,
    	  								   BURST_SIZE);
    	  buffer_selected = 0;
      }

      GPIOB->ODR &= ~GPIO_PIN_4;

    }
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
  while(1)
  {
	  HAL_Delay(1000);
	  LED_STATUS_GPIO_Port->ODR ^= LED_STATUS_Pin;
  };
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
  while(1)
  {
	 HAL_Delay(1500);
	 LED_STATUS_GPIO_Port->ODR ^= LED_STATUS_Pin;
   };
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
