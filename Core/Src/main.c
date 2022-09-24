/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define usTIM TIM4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_PinState button;
uint32_t numTicks;
uint8_t state;
int16_t ambient_temperature;
int16_t object_temperature;
int heightdistance;
int height;
int BPM;
char uartBuf[100];
char uartBuf2[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void usDelay(uint32_t uSec);
void setLED();
void scanMLX90614();
int getHeight(uint32_t fixedHeight);
int isButtonLiftUp();
int isButtonHold();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const float speedOfSound = 0.0343/2;
const int BPM_N = 10;
char data[32] = " ";
char data1[32] = " ";
short buttonInput[2] = {0};
int BPM_final = 0;
int BPM_CNT = 0;
uint8_t Buffer[25] = {0};
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";
GPIO_TypeDef* LEDPorts[4] = {
		LED1_GPIO_Port,
		LED2_GPIO_Port,
		LED3_GPIO_Port,
		LED4_GPIO_Port
};
uint8_t LEDPins[4] = {
		LED1_Pin,
		LED2_Pin,
		LED3_Pin,
		LED4_Pin
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char uart_buf[50];
	int uart_buf_len;
	state = 0;
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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	uint8_t data_read[4];
	uint8_t data_read1[4];
	int16_t aux;
	int16_t aux1;


	uart_buf_len = sprintf(uart_buf, "Timer\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t *)uart_buf, uart_buf_len, 100);

	//Start timer
	HAL_TIM_Base_Start(&htim10);

	HAL_ADC_Start(&hadc1);

	uint16_t timer_val;


	int nb_beat=0;
	int ang_sensor_old=0;
	int ang_sensor;

	int ok=0;
	int BPM_list[30];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  scanMLX90614();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    buttonInput[1] = HAL_GPIO_ReadPin(PressButton_GPIO_Port, PressButton_Pin);

    setLED();

	switch(state)
	{
		case 0:
			if(isButtonHold())
			{
				height = getHeight(200);
			}
			break;
		case 1:
			if(isButtonHold())
			{
				HAL_I2C_Mem_Read(&hi2c1,(0x5A<<1), 0x06, 1, (uint8_t *)data_read, 2, 100);

				aux = (int16_t) ((data_read[1] << 8) | data_read[0]);
				ambient_temperature = aux * 0.02 - 273.15;

				HAL_Delay(100);

				HAL_I2C_Mem_Read(&hi2c1,(0x5A<<1), 0x07, 1, (uint8_t *)data_read1, 2, 100);

				aux1 = (int16_t) ((data_read1[1] << 8) | data_read1[0]);
				object_temperature = aux1 * 0.02 - 273.15;

				HAL_Delay(100);

//				sprintf(data,"\n\r \f Ambient = %d \n\r", ambient_temperature);
//
//				sprintf(data1,"\f Object = %d \n\r", ambient_temperature1);
//
//				HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 1000);
//				HAL_UART_Transmit(&huart2, (uint8_t*)data1, strlen(data1), 1000);
			}
			break;
		case 2:
			{
				HAL_ADC_Start(&hadc1);
				if(BPM_CNT < BPM_N)
				{
					if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
					{
						ang_sensor = HAL_ADC_GetValue(&hadc1);

						// Serial plotter debug
//						uart_buf_len = sprintf(uart_buf, "%d, %d \r\n", ang_sensor,ok);
//						HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

						ok = (int) (ang_sensor>2100 && ang_sensor>ang_sensor_old);

						if(nb_beat==0 && ok==1)
						{
							timer_val=__HAL_TIM_GET_COUNTER(&htim10);
							nb_beat=1;
						}
						if(nb_beat==1 && ok==0)
						{
							timer_val = __HAL_TIM_GET_COUNTER(&htim10) - timer_val;

							nb_beat=0;

							BPM = (int) ((timer_val/1000.0)*60);

							if(BPM>=40 && BPM<=120)
							{
								BPM_list[BPM_CNT++] = BPM;
								// CubeIDE Serial Monitor
								uart_buf_len = sprintf(uart_buf, "%d \r\n", BPM);
								HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
							}
						}
						ang_sensor = ang_sensor_old;
					}
				}

				else if (BPM_final == 0)
				{
					for (int j = 0;j < BPM_N; j++)
					{
						BPM_final += BPM_list[j];
					}
					BPM_final /= BPM_N;
				}
				else
				{
					uart_buf_len = sprintf(uart_buf, "Heart Rate Finished \r\n");
					HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
				}
			}
			break;
		case 3:
			// oximeter
			break;
		default:
			uart_buf_len = sprintf(uart_buf, "Height: %d m\r\n", height);
			HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
			uart_buf_len = sprintf(uart_buf, "Body Temperature: %d c\r\n", object_temperature);
			HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
			uart_buf_len = sprintf(uart_buf, "Heart Rate: %d BPM\r\n", BPM_final);
			HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
			exit(0);
	}

	if(isButtonLiftUp() && (state != 2 || BPM_final != 0)) state += 1;

	if(state == 2) {
		HAL_Delay(10);
	}
	else {
		HAL_Delay(100);
	}

	buttonInput[0] = buttonInput[1];

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8000-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED_Pin|Trigger2_Pin
                          |Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_Pin PressButton_Pin */
  GPIO_InitStruct.Pin = Button_Pin|PressButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED_Pin Trigger2_Pin
                           Trigger_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED_Pin|Trigger2_Pin
                          |Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Echo2_Pin Echo_Pin */
  GPIO_InitStruct.Pin = Echo2_Pin|Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void usDelay(uint32_t uSec) {
	if(uSec < 2) uSec = 2;
	usTIM->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
	usTIM->EGR = 1; 			/*Re-initialises the timer*/
	usTIM->SR &= ~1; 		//Resets the flag
	usTIM->CR1 |= 1; 		//Enables the counter
	while((usTIM->SR&0x0001) != 1);
	usTIM->SR &= ~(0x0001);
}

int getHeight(uint32_t fixedHeight) {

	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
	//set TRIG to LOW for few uSec
	 usDelay(3);
	//*** START Ultrasonic measure routine ***//
	 HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
	 usDelay(10);

	 HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

	//what for ECHO pin rising edge
	 while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_RESET);
	 //start measuring ECHO pulse width in usec
	 numTicks = 0;
	 while(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET)
	 {
		 numTicks++;
		 usDelay(2); //2.8 usec
	 };
	 //estimate distance in cm
	 heightdistance = (numTicks + 0.0f)*2.8*speedOfSound;
	 return fixedHeight - heightdistance;

}

void setLED() {
    for(int i = 0; i < 4; i++) {
    	if(i == 2) {
    		HAL_GPIO_WritePin(LEDPorts[i], LEDPins[i], state == 2 && (BPM_CNT%2 || BPM_final != 0));
    	}
    	else {
    		HAL_GPIO_WritePin(LEDPorts[i], LEDPins[i], i == state);
    	}
    }
}

int isButtonHold() {
	return buttonInput[0] == buttonInput[1] && buttonInput[0] == 1;
}

int isButtonLiftUp() {
	return buttonInput[0] == 1 && buttonInput[1] == 0;
}

void scanMLX90614() {
	HAL_StatusTypeDef HAL_STATUS;
	HAL_STATUS = HAL_I2C_IsDeviceReady(&hi2c1, (0x5A), 4, 100);
	if(HAL_STATUS==HAL_OK){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  HAL_Delay(3000);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}

	uint8_t i = 0, ret;

	HAL_Delay(1000);

	/*-[ I2C Bus Scanning ]-*/
	HAL_UART_Transmit(&huart2, StartMSG, sizeof(StartMSG), 10000);
	for(i=1; i<128; i++)
	{
	  ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
	  if (ret != HAL_OK) /* No ACK Received At That Address */
	  {
		  HAL_UART_Transmit(&huart2, Space, sizeof(Space), 10000);
	  }
	  else if(ret == HAL_OK)
	  {
		  sprintf(Buffer, "0x%X", i);
		  HAL_UART_Transmit(&huart2, Buffer, sizeof(Buffer), 10000);
	  }
	}
	HAL_UART_Transmit(&huart2, EndMSG, sizeof(EndMSG), 10000);
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
