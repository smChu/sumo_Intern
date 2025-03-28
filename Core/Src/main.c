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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint32_t adcVal[10] = {0, };
int adc[9] = {0, };
int mult[5] = {1, 2, 4, 8, 10};
int readc = 0;
int resultVal = 0;
int duty[2] = {0, };

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
   LL_TIM_EnableIT_UPDATE(TIM6);
   LL_TIM_EnableCounter(TIM6);
   LL_TIM_EnableIT_UPDATE(TIM7);
   LL_TIM_EnableCounter(TIM7);


   LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
   LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
   LL_TIM_EnableCounter(TIM1);

   LL_TIM_EnableIT_UPDATE(TIM8);
   LL_TIM_EnableCounter(TIM8);

   LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0,
       LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
       (uint32_t)adcVal,
       LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
   LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, 10);
   LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
   LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
   LL_ADC_Enable(ADC1);
   LL_ADC_REG_StartConversionSWStart(ADC1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_EnableOverDriveMode();
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 180, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(180000000);
  LL_SetSystemCoreClock(180000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */
void Calduty(int *arr)
{
   int absolVal = 0;

   readc = arr[4] - 1950;
   //-1950~1950

   if(readc >= 0)
      absolVal = readc;
   else
      absolVal = readc*(-1);

   if(absolVal > 0 && absolVal <= 390)
      resultVal = readc*mult[0];
   else if(absolVal > 390 && absolVal <= 780)
      resultVal = readc*mult[1];
   else if(absolVal > 780 && absolVal <= 1170)
      resultVal = readc*mult[2];
   else if(absolVal > 1170 && absolVal <= 1560)
      resultVal = readc*mult[3];
   else if(absolVal > 1560 && absolVal < 1950)
      resultVal = readc*mult[4];

   if(readc >= 0){
      duty[1] = 200 - resultVal*0.0102;
      duty[0] = 200 + resultVal*0.0307;
   }
   else{
      duty[0] = 200 + resultVal*0.0102;
      duty[1] = 200 - resultVal*0.0333;
   }
}

void front(int duty1, int duty2)
{
   LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);
   LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);

   LL_TIM_OC_SetCompareCH1(TIM1, duty1);
   LL_TIM_OC_SetCompareCH2(TIM1, duty2);
}

void right(int duty1, int duty2)
{
   LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8); // 8-?   ??? set-?
   LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9); // 9-?  른쪽 reset-?

   LL_TIM_OC_SetCompareCH1(TIM1, duty1);
   LL_TIM_OC_SetCompareCH2(TIM1, duty2);
}

void left(int duty1, int duty2)
{
   LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8); // 8-?   ??? set-?
   LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9); // 9-?  른쪽 reset-?

   LL_TIM_OC_SetCompareCH1(TIM1, duty1);
   LL_TIM_OC_SetCompareCH2(TIM1, duty2);
}

void back(int duty1, int duty2)
{
   LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8); // 8-?   ??? set-?
   LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9); // 9-?  른쪽 reset-?

   LL_TIM_OC_SetCompareCH1(TIM1, duty1);
   LL_TIM_OC_SetCompareCH2(TIM1, duty2);
}

void stop()
{
   LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8); // 8-?   ??? set-?
   LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9); // 9-?  른쪽 reset-?

   LL_TIM_OC_SetCompareCH1(TIM1, 0);
   LL_TIM_OC_SetCompareCH2(TIM1, 0);
}

//float acceleration(float current, float target, float acc)
//{
//    if (current < target)
//    {
//        current = MIN(current + acc, target);
//    }
//    else if (current > target)
//    {
//        current = MAX(current - acc, target);
//    }
//    return current;
//}


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
