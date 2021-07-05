/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>
#include <string.h>
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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char TxDataBuffer[32] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };
char now_freq[32] =
{ 0 };
char now_vhigh[32] =
{ 0 };
char now_vlow[32] =
{ 0 };
char now_Duty[32] =
{ 0 };
uint16_t ADCin = 0;
uint16_t V_ADC_high = 0;
uint16_t V_ADC_low = 0;
int16_t A=0;
float time=0;
uint64_t _micro = 0;
int mode =0; //set sawtooth as default
// 12 bits dac
uint16_t dataOut = 0;
//float volt;
//upper 4 bits of dac
uint8_t DACConfig = 0b0011;
float freq =1.0;
float v_high =3.3;
float v_low =0.0;
float T=0; //Period
float angle=0.0;
int Duty_Cycle =50;


int is_saw_reverse =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint64_t micros();
void UARTRecieveAndResponsePolling();
int16_t UARTRecieveIT();
enum _StateDisplay
{
  StateDisplay_Start = 0,
  StateDisplay_MenuRoot_Print =10,
  StateDisplay_MenuRoot_WaitInput,
  StateDisplay_Saw_Print =20,
  StateDisplay_Saw_WaitInput,
  StateDisplay_Sine_Print =30,
  StateDisplay_Sine_WaitInput,
  StateDisplay_Square_Print =40,
  StateDisplay_Square_WaitInput

};
uint8_t STATE_Display = 0;

int count =1;
uint8_t Is_LED_on =1;
int LED_TOGGLE_DELAY =500;
int is_press =0;
float Duty_On =0.0;
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
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCin, 1); // read ADC in

	HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);//set low ไว้
	  {
	  char temp[]="HELLO WORLD\r\n please type something to test UART\r\n";
	  HAL_UART_Transmit(&huart2, (uint8_t*)temp, strlen(temp),10);
	  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		static uint64_t timestamp = 0;
		if (micros() - timestamp > 10000) //10khz = 100 Microsec
		{
			timestamp = micros();
			V_ADC_high = v_high *4096/3.3;
			V_ADC_low = v_low *4096/3.3;
			A = V_ADC_high - V_ADC_low;
			if(mode==0)
			{
				//V_ADC_high = v_high *4096/3.3;
				//V_ADC_low = v_low *4096/3.3;
				time += 0.01;
				T = 1/freq;
				//A = V_ADC_high - V_ADC_low;
				if(is_saw_reverse ==0)
				{
					//dataOut++; //input get into black DAC
					if (time < T)
					{
//						A = V_ADC_high - V_ADC_low;
						dataOut = ((A)/T)*time+V_ADC_low;
					}
					else
					{
						time =0;
					}
				}
				//dataOut++; //input get in black DAC
				else if(is_saw_reverse ==1)
				{
					//dataOut--; //input get into black DAC
					if (time <= T)
					{
						dataOut = ((-A)/T)*time+V_ADC_high;
					}
					else
					{
						time =0;
					}
				}
				//dataOut %= 4096; //sawtooth
			}
			else if(mode==1)
			{
				angle +=0.01;//radian
				dataOut =((A/2)* sin(2.0*M_PI*freq*angle))+((V_ADC_high+V_ADC_low)/2);
			}
			else if(mode==2)
			{
				time += 0.01;
				T = 1/freq;
				Duty_On = T*Duty_Cycle/100;
				if(time <= Duty_On)
				{
					dataOut = V_ADC_high;
				}
				else if(time > Duty_On && time <= T)
				{
					dataOut = V_ADC_low;
				}
				else
				{
					time = 0.0;
				}
			}

			if (hspi3.State == HAL_SPI_STATE_READY // if not send data
					&& HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)
							== GPIO_PIN_SET)
			{
				MCP4922SetOutput(DACConfig, dataOut);//dataOut to digital
				// analog get in nucleo to adc
			}
			//volt = (ADCin/4095.0) *3.3;// sawtooth
		}
		/*Method 2 Interrupt Mode*/
		HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);

		/*Method 2 W/ 1 Char Received*/
		int16_t inputchar = UARTRecieveIT();
		if(inputchar!=-1)
		{

			sprintf(TxDataBuffer, "ReceivedChar:[%c]\r\n", inputchar);
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);


		}
		switch (STATE_Display)
		{
			case StateDisplay_Start:
				STATE_Display = StateDisplay_MenuRoot_Print;
				break;
			case StateDisplay_MenuRoot_Print: //display one time state
				yemp();
				STATE_Display = StateDisplay_MenuRoot_WaitInput;
				break;
			case StateDisplay_MenuRoot_WaitInput: //wait state for input
				switch (inputchar)
				{
			      case 0:
			        //no input ; just wait input
			        break;
			      case -1:
			        //no input ; just wait input
			        break;
			      case '0':
			        STATE_Display = StateDisplay_Saw_Print; // sawtooth
			        break;
			      case '1':
			        STATE_Display = StateDisplay_Sine_Print; //sine wave
			        break;
			      case '2':
			        STATE_Display = StateDisplay_Square_Print; //square wave
			        break;
			      default: // actually error , you can add error message
			        wng_press();
			        STATE_Display = StateDisplay_MenuRoot_Print;
			        break;
				}
				break;
			case StateDisplay_Saw_Print: //display one time state //saw
				mode =0;
				sawt_msg();
				STATE_Display = StateDisplay_Saw_WaitInput;
				break;
			case StateDisplay_Sine_Print: //display state
				mode=1;
				sinet_msg();
			    STATE_Display = StateDisplay_Sine_WaitInput;
			    break;

			case StateDisplay_Sine_WaitInput: //make decision state
			    switch (inputchar)
			    {
			    	case 0:
			    		//no input ; just wait input
			    		break;
			    	case -1:
			    		//no input ; just wait input
			    	    break;
				    case '+': //+0.1Hz
				    	if(freq < 10)
				    	{
				    		freq +=0.1;
				    	}
				    	else
				    	{
				    		limit_msg();
				    	}
				    	freq_msg();
				    	STATE_Display = StateDisplay_Sine_Print;
				    	break;
				      case '-': // -0.1Hz
				    	  if(freq > 0.1)
				    	  {
				    		  freq -=0.1;
				    	  }
				    	  else
				    	  {
				    		  freq =0;
				    		  limit_msg();
				    	  }
				    	  freq_msg();
				    	  STATE_Display = StateDisplay_Sine_Print;
				    	  break;
				      case 'h': //v high+0.1V
				    	  if(v_high+0.02 < 3.3)
				      	  {
				    		  v_high +=0.1;
				      	  }
				    	  else
				    	  {
				    		  limit_msg();
				    	  }
				    	  vhigh_msg();
				      	  STATE_Display = StateDisplay_Sine_Print;
				      	  break;
				      case 'j': //v high -0.1V
				    	  if(v_high-0.02 > v_low)
				    	  {
				    		  v_high -=0.1;
				      	  }
				      	  else
				      	  {
				      		v_high =v_low;
				      		limit_msg();
				      	  }
				    	  vhigh_msg();
				      	  STATE_Display = StateDisplay_Sine_Print;
				      	  break;
				      case 'k': //v low +0.1V
				    	  if(v_high-0.02 > v_low)
				      	  {
				    		  v_low +=0.1;
				      	  }
				    	  else
				    	  {
				    		  limit_msg();
				    	  }
				    	  vlow_msg();
				      	  STATE_Display = StateDisplay_Sine_Print;
				      	  break;
				      case 'l': //v low -0.1V
				    	  if(v_low+0.02 >=0.1)
				    	  {
				    		  v_low -=0.1;
				      	  }
				      	  else
				      	  {
				      		v_low =0.0;
				      		limit_msg();
				      	  }
				    	  vlow_msg();
				      	  STATE_Display = StateDisplay_Sine_Print;
				      	  break;
			    	case 'x': //
			    		STATE_Display = StateDisplay_MenuRoot_Print;
			    		break;
			    	default: // actually error , you can add error message
			    	    wng_press();
			    	    STATE_Display = StateDisplay_Sine_Print;
			    	    break;
			    }
			    break;
			case StateDisplay_Square_Print: //display state
			    mode=2;
			    square_msg();
			    STATE_Display = StateDisplay_Square_WaitInput;
			    break;
			case StateDisplay_Square_WaitInput: //make decision state
			    switch (inputchar)
			    {
			     	case 0:
			     		//no input ; just wait input
			     		break;
			    	case -1:
			    		//no input ; just wait input
			    		break;
			    	case '8': // forward
			    		if(Duty_Cycle+1 <100)
			    		{
			    			Duty_Cycle +=10;
			    		}
			    		else
			    		{
			    			limit_msg();
			    		}
			    		dutyc_msg();
			    		STATE_Display = StateDisplay_Square_Print;
			    		break;
			    	case '2': // backward
			    		if(Duty_Cycle-1 >0)
			    		{
			    			Duty_Cycle -=10;
			    		}
			    		else
			    		{
			    			limit_msg();
			    		}
			    		dutyc_msg();
			    		STATE_Display = StateDisplay_Square_Print;
			    		break;
			    	case '+': //+0.1Hz
			    		if(freq < 10)
			    		{
			    			freq +=0.1;
			    		}
			    		 else
			    		{
			    			 limit_msg();
			    		}
			    		freq_msg();
			    		STATE_Display = StateDisplay_Square_Print;
			    		break;
			    	case '-': // -0.1Hz
			    		if(freq > 0.1)
			    		{
			    			freq -=0.1;
			    		}
			    		else
			    		{
			    			freq =0;
			    			limit_msg();
			    		}
			    		freq_msg();
			    		STATE_Display = StateDisplay_Square_Print;
			    		break;
			    	case 'h': //v high+0.1V
			    		if(v_high+0.02 < 3.3)
			    		{
			    			v_high +=0.1;
			    		}
			    		else
			    		{
			    			limit_msg();
			    		}
			    		vhigh_msg();
			    		STATE_Display = StateDisplay_Square_Print;
			    		break;
			    	case 'j': //v high -0.1V
			    		if(v_high-0.02 > v_low)
			    		{
			    			v_high -=0.1;
			    		}
			    		else
			    		{
			    			v_high =v_low;
			    			limit_msg();
			    		}
			    		vhigh_msg();
			    		STATE_Display = StateDisplay_Square_Print;
			    		break;
			    	case 'k': //v low +0.1V
			    		if(v_high-0.02 > v_low)
			    		{
			    			v_low +=0.1;
			    		}
			    		else
			    		{
			    			limit_msg();
			    		}
			    		vlow_msg();
			    		STATE_Display = StateDisplay_Square_Print;
			    		break;
			    	case 'l': //v low -0.1V
			    		if(v_low+0.02 >=0.1)
			    		{
			    			v_low -=0.1;
			    		}
			    		else
			    		{
			    			v_low =0.0;
			    			limit_msg();
			    		}
			    		vlow_msg();
			    		STATE_Display = StateDisplay_Square_Print;
			    		break;
			    	case 'x': //
			    		STATE_Display = StateDisplay_MenuRoot_Print;
			    		break;
			    	default: // actually error , you can add error message
			    		wng_press();
			    		STATE_Display = StateDisplay_Square_Print;
			    		break;
			    }
			    break;
			case StateDisplay_Saw_WaitInput: //sawtooth
			      switch (inputchar)
			      {
			      case 0:
			        //no input ; just wait input
			        break;
			      case -1:
			    	  //no input ; just wait input
			    	  break;
			      case '+': //+0.1Hz
			    	  if(freq < 10)
			    	  {
			    		  freq +=0.1;
			    	  }
			    	  else
			    	  {
			    		  limit_msg();
			    	  }
			    	  freq_msg();
			    	  STATE_Display = StateDisplay_Saw_Print;
			    	  break;
			      case '-': // -0.1Hz
			    	  if(freq > 0.1)
			    	  {
			    		  freq -=0.1;
			    	  }
			    	  else
			    	  {
			    		  freq =0;
			    		  limit_msg();
			    	  }
			    	  freq_msg();
			    	  STATE_Display = StateDisplay_Saw_Print;
			    	  break;
			      case 'h': //v high+0.1V
			    	  if(v_high+0.02 < 3.3)
			      	  {
			    		  v_high +=0.1;
			      	  }
			    	  else
			    	  {
			    		  limit_msg();
			    	  }
			    	  vhigh_msg();
			      	  STATE_Display = StateDisplay_Saw_Print;
			      	  break;
			      case 'j': //v high -0.1V
			    	  if(v_high-0.02 > v_low)
			    	  {
			    		  v_high -=0.1;
			      	  }
			      	  else
			      	  {
			      		v_high =v_low;
			      		  limit_msg();
			      	  }
			    	  vhigh_msg();
			      	  STATE_Display = StateDisplay_Saw_Print;
			      	  break;
			      case 'k': //v low +0.1V
			    	  if(v_high-0.02 > v_low)
			      	  {
			    		  v_low +=0.1;
			      	  }
			    	  else
			    	  {
			    		  limit_msg();
			    	  }
			    	  vlow_msg();
			      	  STATE_Display = StateDisplay_Saw_Print;
			      	  break;
			      case 'l': //v low -0.1V
			    	  if(v_low+0.02 >=0.1)
			    	  {
			    		  v_low -=0.1;
			      	  }
			      	  else
			      	  {
			      		v_low =0.0;
			      		limit_msg();
			      	  }
			    	  vlow_msg();
			      	  STATE_Display = StateDisplay_Saw_Print;
			      	  break;
			      case '8': // forward
			    	  is_saw_reverse =0;
			    	  STATE_Display = StateDisplay_Saw_Print;
			    	  break;
			      case '2': // backward
			    	  is_saw_reverse =1;
			      	  STATE_Display = StateDisplay_Saw_Print;
			      	  break;
			      case 'x': //
			    	  STATE_Display = StateDisplay_MenuRoot_Print;
			    	  break;

			      default: // actully error , you can add error message
			    	  wng_press();
			    	  STATE_Display = StateDisplay_Saw_Print;
			    	  break;
			      }
			      break;
		}

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LOAD_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1); // interrupt send data
	//HAL_SPI_Receive_IT(hspi, pData, Size) //receive data
}
void UARTRecieveAndResponsePolling()
{
	char Recieve[32]={0};
	//start Receive in Polling mode
	HAL_UART_Receive(&huart2, (uint8_t*)Recieve, 32, 1000);
	//Create feedback text
	sprintf(TxDataBuffer, "Received:[%s]\r\n", Recieve);
	//send text
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);

}

void yemp()
{
	char yemp[]= "Menu\r\n press 0 for sawtooth \r\n press 1 for sinewave\r\n press 2 for square\r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)yemp, strlen(yemp),30);
}
void wng_press()
{
	char emp[]= "error please press the correct key\r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)emp, strlen(emp),10);
}
void LED_msg()
{
	char lemp[]= "a: speed +1 Hz\r\n s: speed -1 Hz\r\n d: On/Off \r\n x: back \r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)lemp, strlen(lemp),30);
}
void freq_msg()
{
	sprintf(now_freq, "frequency is at : [%.1f] Hz \r\n", freq);
	HAL_UART_Transmit(&huart2, (uint8_t*)now_freq, strlen(now_freq), 30);
}
void vhigh_msg()
{
	sprintf(now_vhigh, "V high at : [%.1f] V \r\n", v_high);
	HAL_UART_Transmit(&huart2, (uint8_t*)now_vhigh, strlen(now_vhigh), 30);
}
void vlow_msg()
{
	sprintf(now_vlow, "V low at : [%.1f] V \r\n", v_low);
	HAL_UART_Transmit(&huart2, (uint8_t*)now_vlow, strlen(now_vlow), 30);
}
void dutyc_msg()
{
	sprintf(now_Duty, "Duty Cycle at [%d] percent \r\n", Duty_Cycle);
	HAL_UART_Transmit(&huart2, (uint8_t*)now_Duty, strlen(now_Duty), 30);
}
void sawt_msg()
{
	char lsaemp[]= "This is sawtooth\r\n +: speed +0.1 Hz	-: speed -0.1 Hz\r\n h: v high +0.1V 	j: v high -0.1V \r\n k: v low +0.1V		l: v low -0.1V \r\n 8: slope up 		2: slope down\r\n x: back \r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)lsaemp, strlen(lsaemp),40);
}
void sinet_msg()
{
	char simp[]= "This is sine wave\r\n +: speed +0.1 Hz	-: speed -0.1 Hz\r\n h: v high +0.1V 	j: v high -0.1V \r\n k: v low +0.1V		l: v low -0.1V x: back \r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)simp, strlen(simp),40);
}
void square_msg()
{
	char sqimp[]= "This is square wave\r\n +: speed +0.1 Hz	-: speed -0.1 Hz\r\n h: v high +0.1V 	j: v high -0.1V \r\n k: v low +0.1V		l: v low -0.1V \r\n 8: +10% dutycycle 	2: -10% dutycycle\r\n x: back \r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)sqimp, strlen(sqimp),40);
}
void limit_msg()
{
	char limitemp[]= "at limit \r\n ";
	HAL_UART_Transmit(&huart2, (uint8_t*)limitemp, strlen(limitemp),10);
}

int16_t UARTRecieveIT()
{
	//stored data last position
	static uint32_t dataPos =0;
	//create dummy data
	int16_t data=-1;
	//check pos in data vs last position
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		//read data to buffer
		data=RxDataBuffer[dataPos];
		//move to next pos
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);  // or  HAL_UART_TransmitIT(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer));
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim11)
	{
		_micro += 65535;
	}
}

inline uint64_t micros()
{
	return htim11.Instance->CNT + _micro;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
