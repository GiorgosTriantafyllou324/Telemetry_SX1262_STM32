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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <SX1262.h>
#include <PressSens.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ID 2
#define TRANSMITS 10

#define RTC_PERIOD_S   12   /* Period in seconds of the RTC interrupts */
#define RESET_TIME_HRS 24  /* Period in hours after which the TPMS returns to the default settings */

#define RESET_TIME RESET_TIME_HRS * 3600 / RTC_PERIOD_S
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
MCU   mcu;
Radio radio;

Pressure_Sensor pr;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

LPTIM_HandleTypeDef hlptim1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef status_hal = HAL_ERROR, test1 = HAL_ERROR, test2 = HAL_ERROR, test3 = HAL_ERROR;
I2Cstatus_t i2c_status;
RadioState_t counter;
HAL_StatusTypeDef hal_status;

bool Rx_counter;

typedef enum
{
	RX = 0,
	TX = 1,
	TX_CONT = 2,

}TXRX;
TXRX txrx;

//bool new_msg = false;
uint64_t last_message_time;
uint64_t last_tx_time;
float packet_loss;
bool first_reception = true;
uint32_t received_msgs = 0, rx_cnt = 0, initial_rx_cnt = 0;
uint32_t tx_cnt = 0, rx_tick, tick1, tick2, tick3;

uint8_t tpms_msg[5];
int t1,t2,t3,t4,t5,t6,t7,t8, t_sleep,t_sleep_start;
int sleep_time = 940;
uint16_t wait_time;
int tst, i;

uint8_t active   = 0;
uint8_t active1 = 0;
uint8_t first_rx = 0;
uint8_t first_wakeup = 0;
uint32_t wake_rx_tick;

uint8_t tx_counter = 0;
uint8_t rx_enable = 0;
uint32_t reset_test;

uint32_t rtc_reset_counter = 0;




typedef enum
{
	REQUEST_MSG = 11,
	CFGR_MSG    = 12,

}TPMS_msg_t;

typedef enum
{
	SAMPLE_0_5 = 1,  /* Sample every 0.5s */
	SAMPLE_1_0 = 2,  /* Sample every 1.0s */
	SAMPLE_3_0 = 3,  /* Sample every 3.0s */

}TPMS_samplingFreq_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_LPTIM1_Init(void);
/* USER CODE BEGIN PFP */
//static void MX_RTC_Init(void);
void DeInitializeTPMS(void);
void InitializeTPMS(void);
void TPMS_sleep(uint32_t period_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == mcu.dio2.pin)
	{
		SX1262dio2Interrupt(&mcu, &radio);
	}

	else if (GPIO_Pin == mcu.dio1.pin)
	{
		radio.msg_pending = true;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */

  MCU_init(&mcu, &hspi1, GPIOA, NSS_Pin, GPIOB, NRESET_Pin, GPIOB, BUSY_Pin, GPIOB, DIO1_Pin, GPIOA,
       	   DIO2_Pin, GPIOB, DIO3_Pin, GPIOB, RF_SWITCH_Pin, GPIOC, RF_SWITCH_SUPP_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  i2c_status += sensor_init(&pr, &hi2c1, VSENS_GPIO_Port, VSENS_Pin);
  Radio_struct_init(&radio);
  counter += SX1262GfskInit(&mcu, &radio, 0, 900);

  last_message_time = HAL_GetTick();

  SleepParams_t sleep;
  sleep.rtcMode = RTC_DISABLED;
  sleep.startMode = WARM_START;


  HAL_PWREx_EnableSRAM2ContentRetention();
  HAL_PWREx_EnableLowPowerRunMode();

  while (1)
  {
	  if (!active)
	  {
		  if (!active1) {

			  active1 = 1;
			  SX1262SetStandby(&mcu, &radio, STDBY_RC);

			  SX1262SetSleep(&mcu, &radio, sleep);

	//---------------------------------------------------------//deinit i2c/spi

			  DeInitializeTPMS();

	//---------------------------------------------------------// STOP2

			  HAL_SuspendTick();

			  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x9C40, 1,0);


	//		  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
			  HAL_PWR_EnterSTANDBYMode();

			  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

			  HAL_ResumeTick();

	//---------------------------------------------------------//init i2c/spi

			  InitializeTPMS();

	//---------------------------------------------------------//

			  tst = 16;
			  SX1262SetStandby(&mcu, &radio, STDBY_RC);
			  tst = 17;
			  radio.msg_pending = false;
			  SX1262SetRx(&mcu, &radio, 0xFFFF);
			  i++;
		  }
		  uint32_t current = HAL_GetTick();
		  while (HAL_GetTick() - current < 100)
		  {
			  if (radio.msg_pending)
			  {
				  active = 1;
				  first_wakeup = 1;
				  rtc_reset_counter = 0;
				  break;
			  }
			  rtc_reset_counter++;
		  }
		  if (!active) {active1 = 0;}
	  }


	  if (radio.msg_pending)
	  {
		  SX1262SetStandby(&mcu, &radio, STDBY_RC); // 1ms

		  SX1262dio1Interrupt(&mcu, &radio);

		  SX1262ClearIrqStatus(&mcu, &radio, 0xFFFF);

		  SX1262SetSleep(&mcu, &radio, sleep);

		  if ((radio.tpms_rx_msg[0] >> 4) == REQUEST_MSG)
		  {
			  wait_time = ((radio.tpms_rx_msg[1] << 8) | radio.tpms_rx_msg[2])  + (10 * ID);

			  if (wait_time >= 10000)
			  {
				  wait_time = 3000;
				  active = 0;
			  }

			  uint8_t tx_freq = radio.tpms_rx_msg[0] & 0b11;

			  switch(ID)
			  {
			  case 0:
				  switch (tx_freq)
			  	  {
			  	  case SAMPLE_0_5:     /* 0.5s */
			  		  sleep_time = 514;
			  		  wait_time += 25; //13
			  		  break;

			  	  case SAMPLE_1_0:     /* 1.0s */
			  		  sleep_time = 1011;
			  		  wait_time += 20;
			  		  break;

			  	  case SAMPLE_3_0:     /* 3.0s */
			  		  sleep_time = 3002;
			  		  wait_time += 9;
			  		  break;

			 	  default:
			  		  break;
			  	  }
			  	  break;

			  case 1:
				  switch (tx_freq)
				  {
				  case SAMPLE_0_5:     /* 0.5s */
					  sleep_time = 506;
					  wait_time += 26;
					  break;

				  case SAMPLE_1_0:     /* 1.0s */
					  sleep_time = 994;
					  wait_time += 15;
					  break;

				  case SAMPLE_3_0:     /* 3.0s */
					  sleep_time = 2948;
					  wait_time -= 32;
					  break;

				  default:
					  break;
				  }
				  break;

			  case 2:
			      switch (tx_freq)
				  {
				  case SAMPLE_0_5:     /* 0.5s */
				      sleep_time = 511;
					  wait_time += 2;
					  break;

				  case SAMPLE_1_0:     /* 1.0s */
					  sleep_time = 1003;
					  wait_time += 33;
					  break;

				  case SAMPLE_3_0:     /* 3.0s */
					  sleep_time = 2979;
					  wait_time -= 3;
					  break;

				  default:
					  break;
				  }
				  break;

			  case 3:
			      switch (tx_freq)
			      {
			      case SAMPLE_0_5:     /* 0.5s */
			          sleep_time = 523 - 10;
			      	  wait_time += 52;
			      	  break;

			      case SAMPLE_1_0:     /* 1.0s */
			      	  sleep_time = 1018 - 10;
			      	  wait_time += 45;
			      	  break;

			      case SAMPLE_3_0:     /* 3.0s */
			      	  sleep_time = 2997 - 5;
			      	  wait_time += 24;
			      	  break;

			      default:
			      	  break;
			      }
			      break;

			  default:
				  break;
			  }
			  sleep_time += (int8_t)(radio.tpms_rx_msg[3]);
			  wait_time += (int8_t)(radio.tpms_rx_msg[4]);
			  TPMS_sleep(wait_time);
			  tx_counter = 0;

			  while (tx_counter < TRANSMITS)
			  {
				  sensor_measure(&pr, TEMP_OSR_256, PRESS_OSR_256); // 10ms

				  tpms_msg[0] = ID;
				  tpms_msg[1] = pr.temperature_reg >> 16;
				  tpms_msg[2] = pr.temperature_reg >> 8;
				  tpms_msg[3] = pr.pressure_reg >> 16;
				  tpms_msg[4] = pr.pressure_reg >> 8;

				  SX1262SetStandby(&mcu, &radio, STDBY_RC);

				  HAL_GPIO_WritePin(RF_SWITCH_SUPP_GPIO_Port, RF_SWITCH_SUPP_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(RF_SWITCH_GPIO_Port, RF_SWITCH_Pin, GPIO_PIN_SET);

				  SX1262SendPayload(&mcu, &radio, tpms_msg, radio.packetParams.Params.Gfsk.PayloadLength - 1, 10, false);

				  HAL_Delay(5);

				  if(first_wakeup)
					  break;

				  if (tx_counter < TRANSMITS - 1)
				  {

					  SX1262SetSleep(&mcu, &radio, sleep);

					  TPMS_sleep(sleep_time);
					  SX1262SetStandby(&mcu, &radio, STDBY_RC);
				  }
				  tx_counter++;
			  }
	//......................................................................
			  SX1262SetSleep(&mcu, &radio, sleep);
			  TPMS_sleep(100);
			  SX1262SetStandby(&mcu, &radio, STDBY_RC);
	//......................................................................
		  }
		  else if ((radio.tpms_rx_msg[0] >> 4) == CFGR_MSG)
		  {
			  int8_t   target_power     =  radio.tpms_rx_msg[1];
			  uint16_t target_freqInMHz = (radio.tpms_rx_msg[2] << 8) | radio.tpms_rx_msg[3];

			  if ((target_power != radio.power) || (target_freqInMHz * 1000000 != radio.rfFrequency))
			  {
				  SX1262GfskInit(&mcu, &radio, target_power, target_freqInMHz);
				  active = 0;
				  active1 = 0;
			  }
		  }

		  radio.msg_pending = false;
		  first_rx = 1;
	  }

	  if (active)
	  {
		  if (first_rx)
		  {
			  if (radio.opMode == SLEEP_MODE)
				  SX1262SetStandby(&mcu, &radio, STDBY_RC);
			  SX1262SetRx(&mcu, &radio, 0xFFFF);

			  wake_rx_tick = HAL_GetTick();

			  first_rx = 0;
		  }

		  if (!first_rx)
		  {
			  while(HAL_GetTick() - wake_rx_tick < 500)
			  {
				  if (radio.msg_pending)
					  break;

			  }
			  if (!radio.msg_pending)
				  active = 0;

			  SX1262SetStandby(&mcu, &radio, STDBY_RC);
		  }
	  }
	  //reset_test = RESET_TIME;
	  /* Checks if the system has to be reset to its default values */
	  if (rtc_reset_counter > RESET_TIME)
	  {
		    __disable_irq();
		    HAL_NVIC_SystemReset();
	  }
	  first_wakeup = 0;

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 4;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00000103;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  hlptim1.Init.RepetitionCounter = 0;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */

  HAL_GPIO_WritePin(GPIOB, NRESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, GPIO_PIN_SET);

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|NSS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRESET_Pin|RF_SWITCH_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, NRESET_Pin, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RF_SWITCH_SUPP_Pin|VSENS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DIO2_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin NSS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO3_Pin BUSY_Pin */
  GPIO_InitStruct.Pin = DIO3_Pin|BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NRESET_Pin RF_SWITCH_Pin */
  GPIO_InitStruct.Pin = NRESET_Pin|RF_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_SWITCH_SUPP_Pin VSENS_Pin */
  GPIO_InitStruct.Pin = RF_SWITCH_SUPP_Pin|VSENS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void DeInitializeTPMS(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructureA;

	  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	  HAL_SPI_DeInit(&hspi1);
	  __HAL_RCC_SPI1_CLK_DISABLE();

	  HAL_I2C_DeInit(&hi2c1);
	  __HAL_RCC_I2C1_CLK_DISABLE();

	  GPIO_InitStructure.Pin = GPIO_PIN_All;
	  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStructure.Pull = GPIO_NOPULL;

	  GPIO_InitStructureA.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 /*| GPIO_PIN_4*/ | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 /*| GPIO_PIN_13 | GPIO_PIN_14*/ | GPIO_PIN_15;
	  GPIO_InitStructureA.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStructureA.Pull = GPIO_NOPULL;

	  HAL_GPIO_Init(GPIOA, &GPIO_InitStructureA);
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);


	  __HAL_RCC_GPIOA_CLK_DISABLE();
	  __HAL_RCC_GPIOB_CLK_DISABLE();
	  __HAL_RCC_GPIOC_CLK_DISABLE();
	  __HAL_RCC_GPIOH_CLK_DISABLE();
}


void InitializeTPMS(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();

	__HAL_SPI_ENABLE(&hspi1);
	__HAL_I2C_ENABLE(&hi2c1);
}


void TPMS_sleep(uint32_t period_ms)
{
	//---------------------------------------------------------//deinit i2c/spi

	DeInitializeTPMS();

	//---------------------------------------------------------// STOP2

	HAL_SuspendTick();

	HAL_LPTIM_Counter_Start(&hlptim1, period_ms);
	HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 1, period_ms);

	//HAL_PWR_EnterSTANDBYMode();
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);


	HAL_LPTIM_Counter_Stop(&hlptim1);


	HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
	HAL_ResumeTick();

	//---------------------------------------------------------//init i2c/spi

	InitializeTPMS();

	//---------------------------------------------------------//
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
