/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint32_t adcVal[10];
extern int adc[9];
extern int mult[5];
extern int readc;
extern int resultVal;
extern int duty[2];

int qti[2] = {0, };
int qtisum[2] = {0, };
int qtiVal[2] = {0, };

int startFlag = 0;
int cnt = 0;

int filter[11][5] = {0, };

int i = 0;
int sum[9] = {0, };

int pixyVal = 0;
int test = 0;

int RotDirection = 1; //rotate direction

int Delay1 = 0;
int Delay2 = 0;

int EscapeStopFlag = 0;
int EscapeStopcnt = 0;
int EscapeDirection = 1;

int mode = 1;

int BackFlag = 1;
int Backcnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long QTI(GPIO_TypeDef* GPIOx, uint16_t PINx)
{
   long qti_value = 0;
   LL_GPIO_SetPinMode(GPIOx, PINx, LL_GPIO_MODE_OUTPUT);
   LL_GPIO_SetOutputPin(GPIOx, PINx);

   LL_mDelay(1);

   LL_GPIO_SetPinMode(GPIOx, PINx, LL_GPIO_MODE_INPUT);
   LL_GPIO_ResetOutputPin(GPIOx, PINx);

   while(LL_GPIO_IsInputPinSet(GPIOx, PINx))
   {
      qti_value++;
   }

   return qti_value;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
     if(LL_TIM_IsActiveFlag_UPDATE(TIM8))
      {
        for(int k = 0; k < 9; k++){
           sum[k] = sum[k] - filter[k][i];
        }

         filter[0][i] = adcVal[4];
         filter[1][i] = adcVal[1];
         filter[2][i] = adcVal[2];
         filter[3][i] = adcVal[3];
         filter[4][i] = adcVal[0];
         filter[5][i] = adcVal[6];
         filter[6][i] = adcVal[7];
         filter[7][i] = adcVal[8];
         filter[8][i] = adcVal[9];

        for(int k = 0; k < 9; k++){
           sum[k] = sum[k] + filter[k][i];
        }

         i++;

         if(i >= 5){
            i = 0;
         }

         adc[0] = sum[4]/5; //Backpsd
         adc[1] = sum[1]/5; //Frontpsd1
         adc[2] = sum[2]/5; //Frontpsd2
         adc[3] = sum[3]/5; //Bottompsd
         adc[4] = sum[0]/5; //pixy x좌표
         adc[5] = sum[5]/5; //psd5
         adc[6] = sum[6]/5; //psd6
         adc[7] = sum[7]/5; //FrontIR
         adc[8] = sum[8]/5; //BackIR



         /*RotDirection[j] = adc[4];
         j++;
         if(j >= 3){
             for(int k = 2; k > 0; k--){
                RotDirection[k-1] = RotDirection[k];
             }

             j = 2;
         }*/


         LL_TIM_ClearFlag_UPDATE(TIM8);
      }

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
   if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) {
/*
      if (startFlag == 0) { //Motion right after start
         cnt++;
         if (cnt < 901) {
            left(300, 300);
         }
         else if (cnt >= 901 && cnt < 6001) {
            front(400, 400);
         }
         else if (cnt >= 6001 && cnt < 7251) {
            left(400, 400);
         }
         else if (cnt >= 7251 && cnt < 12000) {
            front(500, 500);
         }
         else {
            startFlag = 1;
         }
      }

      else { //startFlag == 1 / General Motion start
         if (adc[3] < 1300) { //BottomPSD
            if(adc[8] < 3000){ //BackIR
               back(700, 700);
            }
            else{
               if (BackFlag == 1) {
                  Backcnt++;
                  if (adc[5] > adc[6]) {
                     back(800, 400);
                     LL_mDelay(120);
                  }
                  else {
                     back(400, 800);
                     LL_mDelay(120);
                  }

                  if (Backcnt == 3) {
                     BackFlag = 2;
                  }
               }
               else {
                  front(650, 650);
               }
            }
         }
         else {
            if (qtiVal[0] < 5000 || qtiVal[1] < 5000) { //QTI Recognition Color
               if (adc[0] > 800) { //BackPSD = Wall

                  BackFlag = 1;

                  if(adc[4] > 200 && adc[4] < 3800){ //Escape

                     if(EscapeStopFlag == 0){ //Stop 1s
                        EscapeStopcnt++;
                        if(EscapeStopcnt <= 10000)
                           stop();
                        else if(EscapeStopcnt > 10000){
                           EscapeStopcnt = 0;
                           EscapeStopFlag = 1;
                           mode = 2;
                        }
                     }
                     else if(EscapeStopFlag == 1){ //EscapeDirection
                        if (adc[5] > adc[6]) {
                           EscapeDirection = 1;
                        }
                        else {
                           EscapeDirection = -1;
                        }
                        EscapeStopFlag = 2;
                     }
                     else if(EscapeStopFlag == 2){ //EscapeRotate
                        if(EscapeDirection == 1){
                           right(200, 200);
                        }
                        else{
                           left(200, 200);
                        }
                        //EscapeStopFlag = 3;
                     }

                  }
                  else{ //EscapeStart
                     front(700, 700);
                     LL_mDelay(125);
//                     if(EscapeStopFlag == 3){
//                        front(700, 700);
//                     }
//                     else{
//                        if (adc[1] > 1100) {
//
//                           if (EscapeStopFlag == 0) { //Stop 1s
//                              EscapeStopcnt++;
//                              if (EscapeStopcnt <= 10000)
//                                 stop();
//                              else if (EscapeStopcnt > 10000) {
//                                 EscapeStopcnt = 0;
//                                 EscapeStopFlag = 1;
//                                 mode = 2;
//                              }
//                           }
//                           else if (EscapeStopFlag == 1) { //EscapeDirection
//                              if (adc[5] > adc[6]) {
//                                 EscapeDirection = 1;
//                              }
//                              else {
//                                 EscapeDirection = -1;
//                              }
//                              EscapeStopFlag = 2;
//                           }
//                           else if (EscapeStopFlag == 2) { //EscapeRotate
//                              if (EscapeDirection == 1) {
//                                 right(200, 200);
//                              }
//                              else {
//                                 left(200, 200);
//                              }
//                           }
//
//                        }
//                        else {
//                           front(700, 700);
//                        }
//                     }
                  }
               }
               else {
                  back(300, 300);
                  mode = 1;
//                  Delay2++;
//                  if (Delay2 <= 2000) {
//                     back(300, 300);
//                     mode = 1;
//                  }
//                  else if (Delay2 > 2000) {
//                     Delay2 = 0;
//                  }
               }
            }
            else { //QTI Black

               EscapeStopFlag = 0;

               if (adc[4] > 100 && adc[4] < 4000) { //Recognition GreenMark

                  pixyVal = adc[4];

                  if (mode == 1) {
                     if (adc[1] > 400) {
                        if (adc[1] > 2000) {
                           front(650, 650); //Duty change -> 800
                        }
                        else {
                           Delay1++;
                           if (Delay1 <= 400) {
                              back(100, 100);
                           }
                           else if (Delay1 <= 800 && Delay1 > 400) {
                              front(600, 600);
                           }
                           else if (Delay1 > 800) {
                              Delay1 = 0;
                           }
                        }
                     }

                     else {
                        Calduty(adc);
                        front(duty[0], duty[1]);
                     }
                  }
                  else if (mode == 2) {
                     if (adc[1] < 700) { //PSD InRange
                        Calduty(adc);
                        front(duty[0], duty[1]);
                     }
                     else { //PSD OutRange
                        front(650, 650); //Duty change -> 800
                     }
                  }
               }
               else { //Not Recognition GreenMark
                  if (pixyVal < 400) {
                     left(300, 300);
                  }
                  else {
                     right(300, 300);
                  }
               }
            }
         }
      }
*/
      LL_TIM_ClearFlag_UPDATE(TIM6);
   }

  /* USER CODE END TIM6_DAC_IRQn 0 */

  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
    if(LL_TIM_IsActiveFlag_UPDATE(TIM7))
      {
             qti[0] = QTI(GPIOC, LL_GPIO_PIN_4);
             qti[1] = QTI(GPIOC, LL_GPIO_PIN_5);


             qtisum[0] = qtisum[0] - filter[9][i];//qti
             qtisum[1] = qtisum[1]- filter[10][i];

             filter[9][i] = qti[0];//qti
             filter[10][i] = qti[1];

             qtisum[0] = qtisum[0] + filter[9][i];
             qtisum[1] = qtisum[1] + filter[10][i];

             i++;

             if(i >= 5){
                i = 0;
             }

             qtiVal[0] = qtisum[0]/5; //qti1
             qtiVal[1] = qtisum[1]/5; //qti2

            LL_TIM_ClearFlag_UPDATE(TIM7);
      }

  /* USER CODE END TIM7_IRQn 0 */
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */

  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
