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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt_delay.h"
#include "LCD1602.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "sim808.h"
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
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

char str_gas[10];
char str_temp[10];
char str_humi[10];
float percentage_gas;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay (uint16_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim3))<time);
}

void beep(cnt)
{
	for(int i=0;i<cnt;i++)
	{
		HAL_GPIO_WritePin(Buzz_GPIO_Port,Buzz_Pin,1);
		HAL_Delay(150);
		HAL_GPIO_WritePin(Buzz_GPIO_Port,Buzz_Pin,0);
		HAL_Delay(150);
	}
}

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
//----DHT11

#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_11

void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	delay (18000);   // wait for 18ms
    HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
	delay (20);   // wait for 20us
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	delay (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1; // 255
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		delay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//----DHT22

#define DHT22_PORT GPIOB
#define DHT22_PIN GPIO_PIN_11

void DHT22_Start (void)
{
	Set_Pin_Output(DHT22_PORT, DHT22_PIN); // set the pin as output
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
	delay(1200);   // wait for > 1ms

	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
	delay (20);   // wait for 30us

	Set_Pin_Input(DHT22_PORT, DHT22_PIN);   // set as input
}

uint8_t DHT22_Check_Response (void)
{
	Set_Pin_Input(DHT22_PORT, DHT22_PIN);   // set as input
	uint8_t Response = 0;
	delay (40);  // wait for 40us
	if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) // if the pin is low
	{
		delay (80);   // wait for 80us

		if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;  // if the pin is high, response is ok
		else Response = -1;
	}

	while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));   // wait for the pin to go low
	return Response;
}

uint8_t DHT22_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));   // wait for the pin to go high
		delay (40);   // wait for 40 us

		if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));  // wait for the pin to go low
	}

	return i;
}
//////////////////////


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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	DWT_Init();
	lcd_init();
	lcd_put_cur(0, 5);
	lcd_send_string("Hello");
	HAL_Delay(3000);
	lcd_clear();
	beep(2);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	HAL_TIM_Base_Start(&htim3);
	
	int cnt_sms=0;
	int notify = 1;
	lcd_put_cur(1, 12);
	lcd_send_string("Noti");
  while (1)
  {
		if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)==0)
		{
			while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)==0);
			notify = ! notify;
			beep(1);
			if(notify)
			{
				lcd_put_cur(1, 12);
				lcd_send_string("Noti");
			}
			else
			{
				lcd_put_cur(1, 12);
				lcd_send_string("      ");
			}
		}
		if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)==0)
		{
			while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)==0);
			beep(1);
			NVIC_SystemReset();
		}
		
		HAL_GPIO_TogglePin(LEDBT_GPIO_Port,LEDBT_Pin);
		
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
		percentage_gas = (adc_value / 4095.0) * 100.0;
		HAL_ADC_Stop(&hadc1);
		lcd_put_cur(1, 0);
		sprintf((char *)str_gas,"Gas:%.0f", percentage_gas);
		lcd_send_string((char*)str_gas);
		lcd_send_string("% ");
		
		if(percentage_gas > 41 && cnt_sms==0 && notify == 1)
		{
			HAL_GPIO_WritePin(Buzz_GPIO_Port,Buzz_Pin,1);
			HAL_GPIO_WritePin(Lamp_GPIO_Port,Lamp_Pin,1);
			HAL_GPIO_WritePin(Fan_GPIO_Port,Fan_Pin,1);
			
			SIM808_SendSMS(&huart1, "0961548172", "SOS ! ro ri khi Gas");
			cnt_sms ++;
			HAL_Delay(1000);
			
			if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin) == 0)
			{
				while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)==0);
				notify = ! notify;
				beep(1);
				if(notify)
				{
					lcd_put_cur(1, 12);
					lcd_send_string("Noti");
				}
				else
				{
					lcd_put_cur(1, 12);
					lcd_send_string("      ");
				}
			}
			if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)==0)
			{
				while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)==0);
				beep(1);
				NVIC_SystemReset();
			}
		}
		else
		{
			HAL_GPIO_WritePin(Buzz_GPIO_Port,Buzz_Pin,0);
			HAL_GPIO_WritePin(Lamp_GPIO_Port,Lamp_Pin,0);
			HAL_GPIO_WritePin(Fan_GPIO_Port,Fan_Pin,0);
			if(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin) == 0)
			{
				while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)==0);
				notify = ! notify;
				beep(1);
				if(notify)
				{
					lcd_put_cur(1, 12);
					lcd_send_string("Noti");
				}
				else
				{
					lcd_put_cur(1, 12);
					lcd_send_string("      ");
				}
			}
			if(HAL_GPIO_ReadPin(BT2_GPIO_Port,BT2_Pin)==0)
			{
				while(HAL_GPIO_ReadPin(BT1_GPIO_Port,BT1_Pin)==0);
				beep(1);
				NVIC_SystemReset();
			}
			
		}
		
		//-----DHT22		
//		DHT22_Start();
//	  Presence = DHT22_Check_Response();
//	  Rh_byte1 = DHT22_Read ();
//      Rh_byte2 = DHT22_Read ();
//	  Temp_byte1 = DHT22_Read ();
//	  Temp_byte2 = DHT22_Read ();
//	  SUM = DHT22_Read();
//	  TEMP = ((Temp_byte1<<8)|Temp_byte2);
//	  RH = ((Rh_byte1<<8)|Rh_byte2);
//	  Temperature = (float) (TEMP/10.0);
//	  Humidity = (float) (RH/10.0);
		//
		DHT11_Start();
	  Presence = DHT11_Check_Response();
	  Rh_byte1 = DHT11_Read ();
	  Rh_byte2 = DHT11_Read ();
	  Temp_byte1 = DHT11_Read ();
	  Temp_byte2 = DHT11_Read ();
	  SUM = DHT11_Read();
	  TEMP = Temp_byte1;
	  RH = Rh_byte1;
	  Temperature = (float) TEMP;
	  Humidity = (float) RH;
		//
		lcd_put_cur(0, 0);
		sprintf((char *)str_temp,"T=%.0f", Temperature);
		lcd_send_string((char*)str_temp);
		lcd_send_string("C  ");
		
		sprintf((char *)str_humi,"H=%.0f", Humidity);
		lcd_send_string((char*)str_humi);
		lcd_send_string("%  ");
//-----------------------------

		HAL_Delay(2000);
		cnt_sms = 0;

		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDBT_GPIO_Port, LEDBT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Lamp_Pin|Fan_Pin|DHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzz_Pin|LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|LCD_EN_Pin|LCD_RW_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LEDBT_Pin */
  GPIO_InitStruct.Pin = LEDBT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDBT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Lamp_Pin Fan_Pin DHT_Pin */
  GPIO_InitStruct.Pin = Lamp_Pin|Fan_Pin|DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BT2_Pin BT1_Pin */
  GPIO_InitStruct.Pin = BT2_Pin|BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzz_Pin LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin
                           LCD_D4_Pin LCD_EN_Pin LCD_RW_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = Buzz_Pin|LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|LCD_EN_Pin|LCD_RW_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
