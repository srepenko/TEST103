/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"


/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h> /* for printf */
#include <string.h>
#include <cstdio>
#include <cstdlib>
#include "math.h"
#include "..\..\ssd1306\ssd1306.h" /* for printf */
//#include "I2Cdev.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

osThreadId defaultTaskHandle;
osThreadId TaskScrRefreshHandle;
osThreadId TaskSi7021Handle;
osThreadId myTaskEncHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static int16_t Enc1;
RTC_TimeTypeDef Time;
RTC_DateTypeDef Date;
const char *day[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskScrRefresh(void const * argument);
void StartTaskSi7021(void const * argument);
void StartTaskEnc(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
  
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//extern I2C_HandleTypeDef * I2Cdev_hi2c;

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
		for (int i=0; i<2; i++) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	
		HAL_Delay(100);
	}
	ssd1306InitI2C(SSD1306_EXTERNALVCC);
	for (int i=0; i<2; i++) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	
		HAL_Delay(100);
	}
	//HAL_TIM_Base_Start_IT(&htim1);
	if(HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL)!=HAL_OK){
		//Error_Handler(2);
	}
	//TIM1->CNT=720;TIM2->CNT=720;
	char td[]= __TIME__ " " __DATE__;
	Time.Hours = atoi(td);
  Time.Minutes = atoi(td+3);
  Time.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  //Date.WeekDay = RTC_WEEKDAY_MONDAY;
	Date.Date = atoi(td+13);
  Date.Year = atoi(td+18);
	unsigned char mon;
  *(td + 12) = 0;
  for (int i = 0; i < 12; i++)
  {
    if (!strcmp(td+9, months[i]))
    {
      mon = i+1;
    }
  }
  Date.Month = mon;
  

  if (HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
	Enc1=(int16_t)TIM1->CNT;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TaskScrRefresh */
  osThreadDef(TaskScrRefresh, StartTaskScrRefresh, osPriorityIdle, 0, 128);
  TaskScrRefreshHandle = osThreadCreate(osThread(TaskScrRefresh), NULL);

  /* definition and creation of TaskSi7021 */
  osThreadDef(TaskSi7021, StartTaskSi7021, osPriorityIdle, 0, 128);
  TaskSi7021Handle = osThreadCreate(osThread(TaskSi7021), NULL);

  /* definition and creation of myTaskEnc */
  osThreadDef(myTaskEnc, StartTaskEnc, osPriorityIdle, 0, 128);
  myTaskEncHandle = osThreadCreate(osThread(myTaskEnc), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
		//devAddr++;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 12;
  sTime.Minutes = 59;
  sTime.Seconds = 30;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0xf;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC2Filter = 0xF;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|ScrRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 ScrRST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|ScrRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EncBTM_Pin */
  GPIO_InitStruct.Pin = EncBTM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EncBTM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	char time_str[9], date_str[15];
  /* Infinite loop */
  for(;;)
  {	
		
		HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);	
		HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);	
		
		sprintf(time_str, "%.2d:%.2d:%.2d", Time.Hours,Time.Minutes,Time.Seconds);	
		ssd1306FillArea(0, 17, 128, 29);
		ssd1306DrawString(0, 16, time_str, digital7Mono_34ptFontInfo);
				
		sprintf(date_str, "%s %.1d %s 20%.1d", day[Date.WeekDay-1], Date.Date, months[Date.Month-1], Date.Year);
		uint8_t width=0;
		for (uint8_t i=0; i<strlen(date_str); i++) {
			width=width+arial_8ptFontInfo.FontTable[(date_str[i]) - arial_8ptFontInfo.FirstChar].width+arial_8ptFontInfo.FontSpace;
		}
		width=(width-arial_8ptFontInfo.FontSpace)/2;
		ssd1306FillArea(0, 0, 128, 10);
		ssd1306DrawString(63-width, 0, date_str, arial_8ptFontInfo);
		vTaskDelay(1000 / portTICK_RATE_MS);
  }
  /* USER CODE END 5 */ 
}

/* StartTaskScrRefresh function */
void StartTaskScrRefresh(void const * argument)
{
  /* USER CODE BEGIN StartTaskScrRefresh */
  /* Infinite loop */
  for(;;)
  {
    ssd1306Refresh();
    vTaskDelay(40 / portTICK_RATE_MS);
  }
  /* USER CODE END StartTaskScrRefresh */
}

/* StartTaskSi7021 function */
void StartTaskSi7021(void const * argument)
{
  /* USER CODE BEGIN StartTaskSi7021 */
	char s[10];
	uint8_t devAddr=0x80; 
	uint8_t data[3];
	uint16_t outT = 0, outH = 0;
	uint8_t RH_READ 					= 0xE5;
	uint8_t TEMP_READ 				= 0xE3;
	//uint8_t POST_RH_TEMP_READ	= 0xE0; 
	//uint8_t RESET							= 0xFE; 
	//uint8_t USER1_READ				= 0xE7;
	//uint8_t USER1_WRITE				= 0xE6;
	#define HTU21D_CRC8_POLYNOMINAL              0x13100 /* CRC8 polynomial for 16bit CRC8 x^8 + x^5 + x^4 + 1 */
	uint8_t crc;
	uint16_t data1;
  /* Infinite loop */
  for(;;)
  {
		
		if(HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 10, 100) == HAL_OK){
			ssd1306FillArea(0, 53, 77, 11);
			if (HAL_I2C_Master_Transmit(&hi2c1, devAddr, &TEMP_READ, 1, 100) != HAL_OK) Error_Handler();
			if (HAL_I2C_Master_Receive(&hi2c1, devAddr, data, 3, 100) != HAL_OK) Error_Handler();
			outT = ((((int32_t)(((data[0] << 8) | data[1]) & 0xFFFC)*17572)-(4685*65536))>>16)&0xFFFF;
			data1 = (data[0] << 8) | data[1];
			for (uint8_t bit = 0; bit < 16; bit++){
				if (data1 & 0x8000){
					data1 =  (data1 << 1) ^ HTU21D_CRC8_POLYNOMINAL;
				} else {
					data1 <<= 1;
				}
			}
			crc =  data1 >>= 8;
			if (crc == data[2]) sprintf(s, "T: %.2f", (float)outT/100); else sprintf(s, "T: error");	
			ssd1306DrawString(0, 53, s, arial_8ptFontInfo);
			if (HAL_I2C_Master_Transmit(&hi2c1, devAddr, &RH_READ, 1, 100) != HAL_OK) Error_Handler();
			if (HAL_I2C_Master_Receive(&hi2c1, devAddr, data, 3, 100) != HAL_OK) Error_Handler();
			outH = ((((int32_t)(((data[0] << 8) | data[1]) & 0xFFFC)*125)-6)>>16)&0xFFFF;
			data1 = (data[0] << 8) | data[1];
			for (uint8_t bit = 0; bit < 16; bit++){
				if (data1 & 0x8000){
					data1 =  (data1 << 1) ^ HTU21D_CRC8_POLYNOMINAL;
				} else {
					data1 <<= 1;
				}
			}
			crc =  data1 >>= 8;
			if (crc == data[2]) sprintf(s, "H: %.2d%%", outH); else sprintf(s, "H: errrr");	
			ssd1306DrawString(42, 53, s, arial_8ptFontInfo);
		} else {
			ssd1306FillArea(0, 53, 77, 11);
			sprintf(s, "HTU21 error");
			ssd1306DrawString(0, 53, s, arial_8ptFontInfo);
		}
    vTaskDelay(2000 / portTICK_RATE_MS);
		
  }
  /* USER CODE END StartTaskSi7021 */
}

/* StartTaskEnc function */
void StartTaskEnc(void const * argument)
{
  /* USER CODE BEGIN StartTaskEnc */
	char s[10];
  /* Infinite loop */
  for(;;)
  {
    Enc1=(int16_t)TIM1->CNT>>1;
		sprintf(s, "Enc: %.1d", Enc1);	
		ssd1306FillArea(82, 53, 46, 11);
		ssd1306DrawString(82, 53, s, arial_8ptFontInfo);
		vTaskDelay(100 / portTICK_RATE_MS);
  }
  /* USER CODE END StartTaskEnc */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
