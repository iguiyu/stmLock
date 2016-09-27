/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
/* 
 * v0.1
 * 2016-08-24
 * �������ܣ�
 * 1. ���Ӽ�������ͽ����쳣���ܣ�
 *     1. StartLockupEXTask�����������쳣
 *     2. StartLockdownEXTask�����⽵���쳣
 * 2. ���Ӽ��Ƿ������ͷǷ���ѹ�Ĺ��ܣ�StartBadLockStateTask
 * �޸����ܣ�
 * 1. �޸����Ͷ��Ź���
 *
 * v0.2
 * 2016-08-25
 * �������ܣ�
 * 1. ���߹���
 * �޸����ܣ�
 * 1. �Ƿ������ͷǷ���ѹ�߼��޸ģ�
 *	1. �Ƿ���ѹ�������Ȼص���͵㣬����֪����ǰ����״̬��Ȼ��ص���ߵ�
 *	2. �Ƿ����������ǻص���͵�
 *
 * v0.3
 * 2016-08-25
 * �������ܣ�
 * 1. ����Ҫȷ�ϻ�ִ�Ķ��Ŵ���
 *
 * v0.4
 * 2016-08-26
 * �������ܣ�
 * 1. �յ����ź�˯��GSMģ��
 * 2. ��STOPģʽ��ΪSLEEPģʽ����ֹ�ղ�������
 * �޸����ܣ�
 * 1. �޸ķǷ���ѹ�ͷǷ��������󣬴����߼����䡣
 *
 * v0.5
 * 2016-08-29
 * �޸����ܣ�
 * 1. ���ý�ֹ�����жϵķ�ʽ��������
 *
 * v0.6
 * 2016-08-30
 * �޸����ܣ�
 * 1. �����е���������ȼ�������osPriorityNormal
 *
 * v0.7
 * 2016-9-01
 * �������ܣ�
 * 1. ���SIM�������Ƿ�����
 * 2. ���GSM����ע�����
 */

#include <time.h>
#include <string.h>
#include "geomag.h"
#include "misc.h"
#include "gsm.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId defaultTaskHandle;
osThreadId gsmTXTaskHandle;
osThreadId gsmRXTaskHandle;
osThreadId lockupEXHandle;
osThreadId lockdownEXHandle;
osThreadId badLockStateHandle;
osMessageQId gsmMsgQueueHandle;
osMessageQId gsmRXQueueHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint8_t usart1_rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);
void StartgsmTXTask(void const * argument);
void StartgsmRXTask(void const * argument);
void StartLockupEXTask(void const * argument);
void StartLockdownEXTask(void const * argument);
void StartBadLockStateTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void set_unix_ts(uint8_t *buf);
extern void set_date_time(uint8_t *tsbuf);
void check_shell(void);
void init_state(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of gsmTXTask */
  osThreadDef(gsmTXTask, StartgsmTXTask, osPriorityNormal, 0, 64);
  gsmTXTaskHandle = osThreadCreate(osThread(gsmTXTask), NULL);

  /* definition and creation of gsmRXTask */
  osThreadDef(gsmRXTask, StartgsmRXTask, osPriorityNormal, 0, 64);
  gsmRXTaskHandle = osThreadCreate(osThread(gsmRXTask), NULL);

  /* definition and creation of lockupEX */
  osThreadDef(lockupEX, StartLockupEXTask, osPriorityNormal, 0, 64);
  lockupEXHandle = osThreadCreate(osThread(lockupEX), NULL);

  /* definition and creation of lockdownEX */
  osThreadDef(lockdownEX, StartLockdownEXTask, osPriorityNormal, 0, 64);
  lockdownEXHandle = osThreadCreate(osThread(lockdownEX), NULL);

  /* definition and creation of badLockState */
  osThreadDef(badLockState, StartBadLockStateTask, osPriorityNormal, 0, 64);
  badLockStateHandle = osThreadCreate(osThread(badLockState), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of gsmMsgQueue */
  osMessageQDef(gsmMsgQueue, 16, uint32_t);
  gsmMsgQueueHandle = osMessageCreate(osMessageQ(gsmMsgQueue), NULL);

  /* definition and creation of gsmRXQueue */
  osMessageQDef(gsmRXQueue, 64, uint8_t);
  gsmRXQueueHandle = osMessageCreate(osMessageQ(gsmRXQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	HAL_UART_Receive_DMA(&huart1, &usart1_rx_byte, 1);
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB11 PB13 
                           PB14 PB15 PB6 PB7 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
// ���ն��Ż�����
struct UartCache
{
	uint8_t index;
	uint8_t value[128];
};

// �Ƿ���Խ���˯��״̬��
int can_sleep(void)
{
	return (osOK == osThreadIsSuspended(defaultTaskHandle)) &&
		(osThreadBlocked == osThreadGetState(gsmTXTaskHandle)) &&
		(osThreadBlocked == osThreadGetState(gsmRXTaskHandle)) &&
		(osOK == osThreadIsSuspended(lockupEXHandle)) &&
		(osOK == osThreadIsSuspended(lockdownEXHandle)) &&
		(osOK == osThreadIsSuspended(badLockStateHandle));
}
// ��˯��̬���Ѻ�ָ�����
void wakeup_tasks(void)
{
	// ��ǰֻ��ָ�Ĭ������
	if (osOK == osThreadIsSuspended(defaultTaskHandle))
		osThreadResume(defaultTaskHandle);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1) {
		__HAL_UART_FLUSH_DRREGISTER(&huart1); // Clear the buffer to prevent overrun
		osMessagePut(gsmRXQueueHandle, usart1_rx_byte, 0);
	} else if (huart == &huart2)
		put_usart2_byte();
}
// ���״̬��ʶ
enum {
	SHELL_OPEN,	// 0 ��Ǳ���
	SHELL_CLOSE,	// 1 ��ǹر�
};
volatile int shell_state = -1;

/*
 * ����״̬ͼ��
 * K2K3: Y��ʾ�й⣬N��ʾ�޹�
 * ����������ת��Χ��0~90�ȡ�
 * ����״̬ת����NN -> NY -> YY
 * ����״̬ת��: YY -> NY -> NN
 *
 * 	       90
 *	      _YY_
 *	     /    \
 *	    /      \
 *         YN      NY
 *        /          \
 *       /            \
 * 180  YN -----------NN   0
 *
 * ע�⣺
 * 	��0�ȣ����ڽ���״̬����90�ȣ���������״̬��
 * 	ʱ�����ⲻ�ȶ������������жϡ�
 *
 */

enum {
	LOCK_UP,	// 0 ��������
	LOCK_UPPING,	// 1 ����������
	LOCK_DOWN,	// 2 ���ѽ���
	LOCK_DOWNING	// 3 �������½�
};

enum {
	LOCK_DEST_UP,	// 0 Ҫ����
	LOCK_DEST_DOWN,	// 1 Ҫ����
};
// lock_dest��ʾ����ǰ��Ҫ�������ǽ������������������ٽ��Ķ�����
volatile int lock_dest = -1;
volatile int lock_state = -1;
// ���ݵ�����״̬
volatile int transient_lock_down = 0;

enum {
	K_LIGHT_ON,		// 0 �����й�
	K_LIGHT_OFF,		// 1 �����޹�
};

volatile int k2_state = -1;
volatile int k3_state = -1;

// ���ݹܽ�״̬��ʼ����״̬��־λ
void init_state(void)
{
	lock_dest = LOCK_DEST_UP;
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
		shell_state = SHELL_CLOSE;
	else 
		shell_state = SHELL_OPEN;
	
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2))
		k2_state = K_LIGHT_ON;
	else
		k2_state = K_LIGHT_OFF;
	
	if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
		k3_state = K_LIGHT_ON;
	else
		k3_state = K_LIGHT_OFF;
	
	if (k2_state == K_LIGHT_ON && k3_state == K_LIGHT_ON)
		lock_state = LOCK_UP;
	
	if (k2_state == K_LIGHT_OFF && k3_state == K_LIGHT_OFF)
		lock_state = LOCK_DOWN;
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

void lock_up()
{
	if (lock_state == LOCK_UP || lock_state == LOCK_UPPING)
		return;
	lock_dest = LOCK_DEST_UP;
	lock_state = LOCK_UPPING;
	__lock_up();
}
// ͬʱ���Ѽ���쳣������
// ���ڴ���͵�����
void lock_up_ex()
{
	if (lock_state == LOCK_UP || lock_state == LOCK_UPPING)
		return;
	lock_dest = LOCK_DEST_UP;
	lock_state = LOCK_UPPING;
	// ��StartLockupEXTask�����ж�
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
	__lock_up();
	if (osOK == osThreadIsSuspended(lockupEXHandle))
		osThreadResume(lockupEXHandle);
}

void lock_down()
{
	if (lock_state == LOCK_DOWN || lock_state == LOCK_DOWNING)
		return;
	lock_dest = LOCK_DEST_DOWN;
	lock_state = LOCK_DOWNING;;
	transient_lock_down = 0;
	__lock_down();
}
// ͬʱ���Ѽ���쳣������
// ���ڴ���ߵ�����
void lock_down_ex()
{
	if (lock_state == LOCK_DOWN || lock_state == LOCK_DOWNING)
		return;
	lock_dest = LOCK_DEST_DOWN;
	lock_state = LOCK_DOWNING;
	transient_lock_down = 0;
	// ��StartLockdownEXTask�����ж�
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
	__lock_down();
	if (osOK == osThreadIsSuspended(lockdownEXHandle))
		osThreadResume(lockdownEXHandle);
}

static volatile int geomag_wait_tick = 0;
static volatile int geomag_wait_enable = 0;

void enable_geomag_wait(void)
{
	geomag_wait_tick = 0;
	geomag_wait_enable = 1;
}

void disable_geomag_wait(void)
{
	geomag_wait_enable = 0;
	geomag_wait_tick = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static int detect_car = 0;
	
	extern volatile int in_maintenance_mode;
	extern volatile int in_nop_mode;
	
	if (in_nop_mode || in_maintenance_mode)
		return;
	
	switch (GPIO_Pin) {
	case GPIO_PIN_0:
		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_Pin))
			shell_state = SHELL_CLOSE;
		else
			shell_state = SHELL_OPEN;
		break;
	// ����k2
	case GPIO_PIN_2:
		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_Pin)) {
			k2_state = K_LIGHT_ON;
			if (lock_dest == LOCK_DEST_UP && k3_state == K_LIGHT_ON) {
				// �����ٽ粻�ȶ�״̬��
				if (lock_state == LOCK_UP)
					break;
				lock_off();
				lock_state = LOCK_UP;
				HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
				geomag_sleep();
				osMessagePut(gsmMsgQueueHandle, GSM_SEND_MSG_TYPE_AU, 0);
			}
		} else {
			k2_state = K_LIGHT_OFF;
			if (lock_dest == LOCK_DEST_DOWN && k3_state == K_LIGHT_OFF) {
				// �����ٽ粻�ȶ�״̬��
				if (lock_state == LOCK_DOWN)
					break;
				lock_off();
				lock_state = LOCK_DOWN;
				HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
				if (1 == transient_lock_down)
					break;
				geomag_wakeup();
				geomag_reset();
				geomag_corr();
				// ������ʱ��30s���޳�����������
				enable_geomag_wait();
				osMessagePut(gsmMsgQueueHandle, GSM_SEND_MSG_TYPE_AD, 0);
			}
		}
		break;
	// ����k3
	case GPIO_PIN_3:
		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_Pin)) {
			k3_state = K_LIGHT_ON;
			if (lock_dest == LOCK_DEST_UP && k2_state == K_LIGHT_ON) {
				// �����ٽ粻�ȶ�״̬��
				if (lock_state == LOCK_UP)
					break;
				lock_off();
				lock_state = LOCK_UP;
				HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
				geomag_sleep();
				osMessagePut(gsmMsgQueueHandle, GSM_SEND_MSG_TYPE_AU, 0);
			}
		} else {
			k3_state = K_LIGHT_OFF;
			// ���ж�lock_dest != LOCK_DEST_UP��Ϊ�˷�ֹ������
			if (lock_dest == LOCK_DEST_DOWN && k2_state == K_LIGHT_OFF) {
				// �����ٽ粻�ȶ�״̬��
				if (lock_state == LOCK_DOWN)
					break;
				lock_off();
				lock_state = LOCK_DOWN;
				HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
				if (1 == transient_lock_down)
					break;
				geomag_wakeup();
				geomag_reset();
				geomag_corr();
				// ������ʱ��30s���޳�����������
				enable_geomag_wait();
				osMessagePut(gsmMsgQueueHandle, GSM_SEND_MSG_TYPE_AD, 0);
			}
		}
		break;
	case GPIO_PIN_4:
		// �ش�δ���ѧϰ�������Ч������
		if (geomag_not_ready() || lock_state != LOCK_DOWN)
			break;
		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_Pin)) {
			// �޳�
			// �ټ�������Ƿ��г�
			if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_Pin))
				break;
			if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_Pin))
				break;
			// ��⵽�г���ʱ�Ź����޳��������
			if (0 == detect_car)
				break;
			detect_car = 0;
			osMessagePut(gsmMsgQueueHandle, ENABLE_AK(GSM_SEND_MSG_TYPE_ML), 0);
			lock_up_ex();
		} else {
			// �г���
			osMessagePut(gsmMsgQueueHandle, ENABLE_AK(GSM_SEND_MSG_TYPE_MH), 0);
			disable_geomag_wait();
			detect_car = 1;
		}
		break;
	}
	// ���Ƿ���������ѹ
	if (GPIO_Pin == GPIO_PIN_2 || GPIO_Pin == GPIO_PIN_3) {
		if (((lock_state == LOCK_UP) && !(k2_state == K_LIGHT_ON && k3_state == K_LIGHT_ON)) ||
				((lock_state == LOCK_DOWN) && !(k2_state == K_LIGHT_OFF && k3_state == K_LIGHT_OFF))) {
			if (osOK == osThreadIsSuspended(badLockStateHandle))
				osThreadResume(badLockStateHandle);
		}
	}
}

//
static __inline void gsmQueuePut(uint32_t info, uint32_t millisec)
{
	//if (osOK == osThreadIsSuspended(gsmTXTaskHandle))
	//	osThreadResume(gsmTXTaskHandle);
	osMessagePut(gsmMsgQueueHandle, info, millisec);
}

// ��ȡ��ѹ����λ�Ǻ���
uint32_t read_voltage(void)
{
	uint32_t v = 0;
	
	HAL_ADC_Start(&hadc);
	if (HAL_OK == HAL_ADC_PollForConversion(&hadc, 2000))
		v = HAL_ADC_GetValue(&hadc) * 3300 >> 11;
	HAL_ADC_Stop(&hadc);
	return v;
}
// ����ά��ģʽ
volatile int in_maintenance_mode = 0;
// 
void enter_maintenance_mode(int isgsm)
{
	if (isgsm)
		gsm_switch();	// �ر�gsm��������ά��ģʽ
	else
		gsmQueuePut(ENABLE_AK(GSM_SEND_MSG_TYPE_RP), 1000);
	geomag_sleep();
	ir_off();
	lock_off();
	in_maintenance_mode = 1;
	while (1)
		osThreadSuspend(NULL);
}
// ������ϵ�ʱ��������
void check_shell(void)
{
	int i;
	int send_msg = 0;
	// ���5�Σ�ȷ��ȷʵ�ѹر�
	for (i = 0; i < 5; ++i) {
		shell_state = shell_is_open() ? SHELL_OPEN : SHELL_CLOSE;
		if (SHELL_OPEN == shell_state) {
			if (0 == send_msg) {
				gsmQueuePut(GSM_SEND_MSG_TYPE_HO, 1000);
				osDelay(5000);
				// ���ŷ��������˯��
				while (osThreadBlocked != osThreadGetState(gsmTXTaskHandle)) {
					osDelay(1000);
				}
				send_msg = 1;
			}
			gsm_switch();
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			i = -1;
			continue;
		}
		osDelay(1000);
	}
	if (send_msg == 1) {
		gsm_switch();
		osDelay(5000);
	}
}

// ����ѹ��С��4.2vʱ����ά��ģʽ
void check_voltage(void)
{
	uint32_t voltage;
	voltage = read_voltage();
	if (voltage <= 4200)
		 enter_maintenance_mode(0);
}

// ��ʼ��ʱ����
void lock_up_init(void)
{
	int try;
	int i;

	// ����������λ�ã�ֱ�ӷ��ء�
	if (LOCK_UP == lock_state) {
		gsmQueuePut(GSM_SEND_MSG_TYPE_AU, 1000);
		return;
	}
	lock_dest = LOCK_DEST_UP;
	// ��Ҫ������ж�
	// ��ʼʱ�������ܴ�����������״̬����Ҫ���ص����λ�á�
	if (k2_state == K_LIGHT_ON && k3_state == K_LIGHT_OFF) {
		lock_state = LOCK_DOWNING;
		__lock_down();
	} else {	// δ�����λ�ã�������
		lock_state = LOCK_UPPING;
		__lock_up();
	}
	try = 0;
	while (LOCK_UP != lock_state) {
		// ��������£�5s���������λ�á�
		for (i = 0; i < 50; ++i) {
			osDelay(100);
			if (LOCK_UP == lock_state) {
				HAL_Delay(1000);
				HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
				return;
			}
		}
		lock_off();
		// QoS Level 0
		gsmQueuePut(GSM_SEND_MSG_TYPE_FD, 1000);
		if (10 == try)
			 enter_maintenance_mode(0);
		beep(1000);
		osDelay(1000);
		beep(1000);
		osDelay(1000);
		beep(1000);
		
		lock_down();
		for (i = 0; i < 50; ++i) {
			osDelay(100);
			if (LOCK_DOWN == lock_state)
				break;
		}
		lock_off();
		// �жϿ��ܱ�������
		HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
		osDelay(30000);	// 30s

		try++;
		lock_up();
	}
	HAL_Delay(1000);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

#define TO_HEX(d) ((d) > 9 ? (((d) - 10) + 'A') : ((d) + '0'))

void make_send_msg(int msg_type)
{
	struct send_msg_struct *msg = &sms_mem.sms.msg;
	uint32_t voltage;
	
	memset(msg, ' ', sizeof(*msg));
	msg->head[0] = 'X';
	msg->head[1] = 'Y';
	// voltage
	voltage = read_voltage();
	msg->voltage[0] = (voltage / 1000) + '0';
	msg->voltage[1] = '.';
	msg->voltage[2] = (voltage / 100) % 10 + '0';
	if (AK_ENABLED(msg_type)) {
		msg->confirm = '1';
		sms_mem.try = 3;
		sms_mem.is_sent = 0;
	} else {
		msg->confirm = '0';
		sms_mem.try = 0;
	}
	sms_mem.phone_no = gsm_cmd_cmgs[0];
	// unix ts
	set_unix_ts(msg->ts);
	switch (GSM_SEND_MSG_TYPE(msg_type)) {
	case GSM_SEND_MSG_TYPE_AU:
		msg->cmd[0] = 'A';
		msg->cmd[1] = 'U';
		msg->sm1 = 'U';
		msg->sm2 = 'L';
		break;
	case GSM_SEND_MSG_TYPE_AD:
		msg->cmd[0] = 'A';
		msg->cmd[1] = 'D';
		msg->sm1 = 'D';
		msg->sm2 = 'L';
		break;
	case GSM_SEND_MSG_TYPE_MH:
		msg->cmd[0] = 'M';
		msg->cmd[1] = 'H';
		msg->sm1 = 'U';
		msg->sm2 = 'H';
		get_geomag_state();
		msg->data[0] = TO_HEX((geomag_state->state & 0xf000) >> 12);
		msg->data[1] = TO_HEX((geomag_state->state & 0x0f00) >> 8);
		msg->data[2] = TO_HEX((geomag_state->state & 0x00f0) >> 4);
		msg->data[3] = TO_HEX(geomag_state->state & 0x000f);
		break;
	case GSM_SEND_MSG_TYPE_ML:
		msg->cmd[0] = 'M';
		msg->cmd[1] = 'L';
		msg->sm1 = 'D';
		msg->sm2 = 'L';
		get_geomag_state();
		msg->data[0] = TO_HEX((geomag_state->state & 0xf000) >> 12);
		msg->data[1] = TO_HEX((geomag_state->state & 0x0f00) >> 8);
		msg->data[2] = TO_HEX((geomag_state->state & 0x00f0) >> 4);
		msg->data[3] = TO_HEX(geomag_state->state & 0x000f);
		break;
	case GSM_SEND_MSG_TYPE_HO:
		msg->cmd[0] = 'H';
		msg->cmd[1] = 'O';
		break;
	case GSM_SEND_MSG_TYPE_RP:
		msg->cmd[0] = 'R';
		msg->cmd[1] = 'P';
		break;
	case GSM_SEND_MSG_TYPE_FD:
		msg->cmd[0] = 'F';
		msg->cmd[1] = 'D';
		break;
	case GSM_SEND_MSG_TYPE_FU:
		msg->cmd[0] = 'F';
		msg->cmd[1] = 'U';
		break;
	case GSM_SEND_MSG_TYPE_SU:
		msg->cmd[0] = 'S';
		msg->cmd[1] = 'U';
		break;
	case GSM_SEND_MSG_TYPE_AK:
		msg->cmd[0] = 'A';
		msg->cmd[1] = 'K';
		break;
	}
	msg->end = 0x1a;
}

// �޲���ģʽ
volatile int in_nop_mode = 0;
void enter_nop_mode(void)
{
	geomag_sleep();
	ir_off();
	lock_off();
	in_nop_mode = 1;
	osThreadSuspend(NULL);
}

void leave_nop_mode(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
	ir_on();
	HAL_Delay(1000);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
	in_nop_mode = 0;
}

// �����������
void processOrder(struct send_msg_struct *msg)
{
	if (msg->head[0] == 'M' && msg->head[1] == 'G') {
		set_date_time(msg->ts);
		if (in_nop_mode && msg->cmd[0] != 'A' && msg->cmd[1] != 'K')
			return;
		// ��������
		if (msg->cmd[0] == 'U' && msg->cmd[1] == 'A') {
			// �����ǰ�ѽ�����ȡ�����ܵĶ�ʱ������
			if (lock_state == LOCK_DOWN)
				disable_geomag_wait();
			lock_up_ex();
		// ���Ž���
		} else if (msg->cmd[0] == 'D' && msg->cmd[1] == 'A') {
			lock_down_ex();
		// �������״̬
		} else if (msg->cmd[0] == 'G' && msg->cmd[1] == 'S') {
			gsmQueuePut(GSM_SEND_MSG_TYPE_SU, 1000);
		} else if (msg->cmd[0] == 'A' && msg->cmd[1] == 'K') {
			sms_mem.is_sent = 1;
			if (in_maintenance_mode)
				enter_maintenance_mode(1);
			if (in_nop_mode) {
				leave_nop_mode();
				// ���ܴ��޲���ģʽ�ָ������Ѷ��ŷ�������������Ͷ���
				if (osOK == osThreadIsSuspended(gsmTXTaskHandle))
					osThreadResume(gsmTXTaskHandle);
			}
		// ǿ������
		} else if (msg->cmd[0] == 'F' && msg->cmd[1] == 'U') {
			// ����鵱ǰ����λ��
			lock_up_ex();
		} else if (msg->cmd[0] == 'F' && msg->cmd[1] == 'D') {
			// ����鵱ǰ����λ��
			lock_down_ex();
		}
		// ����Ҫȷ�ϻ�ִ�Ķ��ţ����ͻ�ִ����
		if ('1' == msg->confirm)
			gsmQueuePut(GSM_SEND_MSG_TYPE_AK, 1000);
	}
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	beep(200);
	osDelay(1000);
	beep(200);
	// ����ģ���ʼ��
	gsm_init();
	// ������
	//check_shell();
	// �ش�ģ���ʼ��
	geomag_init();
	// ��ʼ����״̬��־λ
	init_state();
	// check voltage
	check_voltage();
	// lock up
	lock_up_init();
  /* Infinite loop */
	for (;;) {
		osThreadSuspend(NULL);
		check_voltage();
	}
  /* USER CODE END 5 */ 
}

/* StartgsmTXTask function */
void StartgsmTXTask(void const * argument)
{
  /* USER CODE BEGIN StartgsmTXTask */
	osEvent event;
  /* Infinite loop */
	for (;;) {
		//
		event = osMessageGet(gsmMsgQueueHandle, osWaitForever);
		if (osEventMessage == event.status) {
// ��0��Ϊ1�����������Ź���
#if 1
			make_send_msg(event.value.v);
			gsm_send_msg();
			//
			if (sms_mem.sms.msg.confirm == '1') {
				while (1) {
					// 5sӦ�ù���
					osDelay(5000);
					if (sms_mem.is_sent)
						break;
					if (0 == sms_mem.try) {
						if (sms_mem.phone_no == gsm_cmd_cmgs[0]) {
							sms_mem.phone_no = gsm_cmd_cmgs[1];
						} else if (sms_mem.phone_no == gsm_cmd_cmgs[1]) {
							sms_mem.phone_no = gsm_cmd_cmgs[2];
						} else {
							// ���ŷ�����ȥ�������޲���ģʽ
							enter_nop_mode();
							// �뿪�޲���ģʽ�󣬼�������֮ǰδ���ͳɹ��Ķ���
							sms_mem.phone_no = gsm_cmd_cmgs[0];
						}
						sms_mem.try = 3;
					}
					gsm_send_msg();
				}
			}
#endif
		}
	}
  /* USER CODE END StartgsmTXTask */
}

/* StartgsmRXTask function */
struct UartCache cache;
void StartgsmRXTask(void const * argument)
{
  /* USER CODE BEGIN StartgsmRXTask */
  /* Infinite loop */
	extern int cpin_ok;
	extern int creg_ok;
	// struct UartCache cache;
	osEvent event;
	uint8_t getSMS= 0;
	
	/* Infinite loop */
	memset(&cache, 0, sizeof(cache));
	for(;;) {
		event = osMessageGet(gsmRXQueueHandle, osWaitForever);
		if (event.status == osEventMessage) {
			cache.value[cache.index++] = event.value.v;
			if (NULL != strstr((char *)cache.value,"\r\n")) {
				if (getSMS == 1){
					// ��������
					processOrder((struct send_msg_struct *)cache.value);
					getSMS = 0;
				} else if (strcmp((const char *)cache.value, "\r\n") == 0) { /* ?????? */
					memset(&cache, 0, sizeof(cache));
					continue;
				} else if (NULL != strstr((char *)cache.value, "ERROR\r\n")) {
					//sendGSMCmd(&lastTxGSMCmd);
				} else if (NULL != strstr((char *)cache.value, "+CMT:")) {
					getSMS = 1;
				} else if (NULL != strstr((char *)cache.value, "+CPIN:READY")) {
					cpin_ok = 1;
				} else if (NULL != strstr((char *)cache.value, "+CREG: 1,1")) {	// ������
					creg_ok = 1;
				} else if (NULL != strstr((char *)cache.value, "+CREG: 1,5")) {	// ����
					creg_ok = 1;
				}
				memset(&cache, 0, sizeof(cache));
				//gsm_sleep();				
			}
		}
	}
  /* USER CODE END StartgsmRXTask */
}

/* StartLockupEXTask function */
void StartLockupEXTask(void const * argument)
{
  /* USER CODE BEGIN StartLockupEXTask */
	int try;
	int i;
  /* Infinite loop */
	for(;;) {
sleep:
		// �󲿷�ʱ�䴦�ڹ���״̬����������ʱ���ѡ�
		osThreadSuspend(NULL);
		if (in_nop_mode || in_maintenance_mode)
			continue;
		
		HAL_Delay(1000);
		HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
		try = 0;
		while (LOCK_UP != lock_state) {
			for (i = 0; i < 50; ++i) {
				osDelay(100);
				if (LOCK_UP == lock_state) {
					HAL_Delay(1000);
					HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
					goto sleep;
				}
			}
			lock_off();
			// QoS Level 0
			gsmQueuePut(GSM_SEND_MSG_TYPE_FD, 1000);
			if (10 == try)
				enter_maintenance_mode(0);
			beep(1000);
			osDelay(1000);
			beep(1000);
			osDelay(1000);
			beep(1000);
		
			lock_down();
			for (i = 0; i < 50; ++i) {
				osDelay(100);
				if (LOCK_DOWN == lock_state)
					break;
			}
			HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
			lock_off();
			osDelay(30000);	// 30s

			try++;
			lock_up();
		}
		HAL_Delay(1000);
		HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
	}
  /* USER CODE END StartLockupEXTask */
}

/* StartLockdownEXTask function */
void StartLockdownEXTask(void const * argument)
{
  /* USER CODE BEGIN StartLockdownEXTask */
	int try;
	int i;
  /* Infinite loop */
	for(;;) {
sleep:
		// �󲿷�ʱ�䴦�ڹ���״̬�����Ž���ʱ���ѡ�
		osThreadSuspend(NULL);
		if (in_nop_mode || in_maintenance_mode)
			continue;
		
		HAL_Delay(1200);
		HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
		try = 0;
		while (LOCK_DOWN != lock_state) {
			for (i = 0; i < 50; ++i) {
				osDelay(100);
				if (LOCK_DOWN == lock_state) {
					HAL_Delay(1000);
					HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
					goto sleep;
				}
			}
			lock_off();
			// QoS Level 0
			gsmQueuePut(GSM_SEND_MSG_TYPE_FU, 1000);
			if (10 == try)
				enter_maintenance_mode(0);
			beep(1000);
			osDelay(1000);
			beep(1000);
			osDelay(1000);
			beep(1000);
			
			lock_up();
			for (i = 0; i < 50; ++i) {
				osDelay(100);
				if (LOCK_UP == lock_state)
					break;
			}
			HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
			lock_off();
			osDelay(30000);	// 30s

			try++;
			lock_down();
		}
		HAL_Delay(1000);
		HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
	}
  /* USER CODE END StartLockdownEXTask */
}

/* StartBadLockStateTask function */
void StartBadLockStateTask(void const * argument)
{
  /* USER CODE BEGIN StartBadLockStateTask */
  	uint32_t millisec;
	uint32_t tick;
	int i;
  /* Infinite loop */
	for(;;) {
sleep:
		osThreadSuspend(NULL);
		if (in_nop_mode || in_maintenance_mode)
			continue;
		HAL_Delay(1200);
		if (osOK != osThreadIsSuspended(lockupEXHandle))
			continue;
		if (osOK != osThreadIsSuspended(lockdownEXHandle))
			continue;

		millisec = 0;
		// �Ƿ���ѹ��������
		if (lock_state == LOCK_UP && !(k2_state == K_LIGHT_ON && k3_state == K_LIGHT_ON)){
			// �ȴ�2���ӣ�����״̬�ȶ�
			osDelay(2000);
			// �ֻص���ߵ��ˡ�
			if (lock_state == LOCK_UP && k2_state == K_LIGHT_ON && k3_state == K_LIGHT_ON)
				goto sleep;
			// QoS Level 0
			gsmQueuePut(GSM_SEND_MSG_TYPE_FD, 1000);
			while (1) {
				// �Ȼص�����״̬,������֪���Լ�����
				tick = 0;
				while (lock_state != LOCK_DOWN || k2_state != K_LIGHT_OFF || k3_state != K_LIGHT_OFF) { 
					lock_dest = LOCK_DEST_DOWN;
					lock_state = LOCK_DOWNING;
					transient_lock_down = 1;
					__lock_down();
					beep(1000);
					millisec += 1000;
					tick += 1000;
					// �������ǿ�����Ҫ5s���ܽ�����͵�
					if (lock_state != LOCK_DOWN && tick >= 5000) {
						lock_off();
						tick = 0;
						osDelay(2000);
						millisec += 2000;
					}
					if (60000 <= millisec) {	// 1 minute
						if (lock_state != LOCK_DOWN || k2_state != K_LIGHT_OFF || k3_state != K_LIGHT_OFF)
							enter_maintenance_mode(0);
					}
				}
				HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
				transient_lock_down = 0;
				osDelay(1000);
				millisec += 1000;
				// ����������
				lock_up();
				tick = 0;
				for (i = 0; i < 100; ++i) {
					osDelay(100);
					tick += 100;
					if (LOCK_UP == lock_state) {
						// ����ߵ�
						if (k2_state == K_LIGHT_ON && k3_state == K_LIGHT_ON) {
							HAL_Delay(1000);
							HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
							goto sleep;
						}
						break;
					} else {
						// 2sӦ�ÿ���������ߵ�
						if (tick >= 2000) {
							lock_off();
							osDelay(2000);
							i += 20;
							tick = 0;
							if (lock_state != LOCK_UP)
								__lock_up();
						}
					}
				}

				lock_off();
				osDelay(1000);
				millisec += 1000 + i * 100;
				if (60000 <= millisec) {	// 1 minute
					if (lock_state != LOCK_UP || k2_state != K_LIGHT_ON || k3_state != K_LIGHT_ON)
						enter_maintenance_mode(0);
				}
			}
		// �Ƿ�����������򵥣�ֻ�轵��
		} else if (lock_state == LOCK_DOWN && !(k2_state == K_LIGHT_OFF && k3_state == K_LIGHT_OFF)) {
			osDelay(2000);
			// �ֻص���͵��ˡ�
			if (lock_state == LOCK_DOWN && k2_state == K_LIGHT_OFF && k3_state == K_LIGHT_OFF)
				goto sleep;
			// QoS Level 0
			gsmQueuePut(GSM_SEND_MSG_TYPE_FU, 1000);
			while (1) {
				lock_dest = LOCK_DEST_DOWN;
				lock_state = LOCK_DOWNING;
				transient_lock_down = 1;
				__lock_down();
				tick = 0;
				for (i = 0; i < 100; ++i) {
					beep(100);
					tick += 100;
					if (LOCK_DOWN == lock_state) {
						if (k2_state == K_LIGHT_OFF && k3_state == K_LIGHT_OFF) {
							HAL_Delay(1000);
							HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
							transient_lock_down = 0;
							goto sleep;
						}
						break;
					} else {
						// �������ǿ�����Ҫ5s���ܽ�����͵�
						if (tick >= 5000) {
							lock_off();
							osDelay(2000);
							i += 20;
							tick = 0;
							if (lock_state != LOCK_DOWN)
								__lock_down();
						}
					}
				}

				lock_off();
				osDelay(1000);
				millisec += 1000 + i * 100;
				if (60000 <= millisec) {	// 1 minute
					if (lock_state != LOCK_DOWN || k2_state != K_LIGHT_OFF || k3_state != K_LIGHT_OFF)
						enter_maintenance_mode(0);
				}
			}
		}
	}
  /* USER CODE END StartBadLockStateTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
	// �޳���ʱ��30s���޳���������
	if (1 == geomag_wait_enable && htim->Instance == TIM1) {
		geomag_wait_tick++;
		if (30000 <= geomag_wait_tick) {	// 30s
			// ȷ������޳���
			if (has_car()) {
				disable_geomag_wait();
				return;
			}
			lock_up_ex();
			disable_geomag_wait();
		}
	}
/* USER CODE END Callback 1 */
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
	while(1) ;
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
