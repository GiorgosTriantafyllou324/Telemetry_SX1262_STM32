/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <SX1262.h>
#include <TeleMsgs_USB.h>
#include <TPMS.h>
#include "stm32f4xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ADC_SAMPLES 1000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
MCU   mcu;
Radio radio;
TelemetryData_t teleData;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* --------- ADC Variables BEGIN ----------- */
volatile uint16_t adc1results[3];
volatile uint32_t adc1_ch1_sum = 0,   adc1_ch2_sum = 0,   adc1_ch3_sum = 0;
volatile float    adc1_ch1_value = 0, adc1_ch2_value = 0, adc1_ch3_value = 0;
volatile uint16_t adc1_conv_cplt = 0;

volatile uint16_t adc2results[3];
volatile uint32_t adc2_ch12_sum = 0,   adc2_ch13_sum = 0,   adc2_ch15_sum = 0;
volatile float    adc2_ch12_value = 0, adc2_ch13_value = 0, adc2_ch15_value = 0;
volatile uint16_t adc2_conv_cplt = 0;

float linear_roll, linear_heave;
uint32_t last_linear_can_tx;
uint16_t linear_can_tx_interval = 90;
uint8_t  aero_fans_can_counter = 0;
/* --------- ADC Variables END ----------- */



/* -------- TPMS variables BEGIN --------- */
int8_t	 cal_sleep_time = 0;
int8_t 	 cal_wait_time  = 0;
bool 	 tpms_msg_sent = true;
int64_t  tpms_msg_time = 0;
uint32_t tpms_interval = 500;
uint32_t wait_time;
bool     config = false;
int8_t   tpms_power = 5;
uint16_t tpms_freq = 900;
TPMS_samplingFreq_t tx_freq;

// TPMS window variables
uint32_t current_tick;
int      window_start = 0;
uint16_t window_width = 120;
int64_t  delay        = 0;
int64_t  width_start  = 0;
int64_t  width_end    = 0;
/* --------- TPMS variables END ---------- */



/* -- Telemetry timing Variables BEGIN -- */
uint32_t rx_tick;
uint32_t tx_tick;
uint16_t tx_msg_delay = 20;  // --> 15ms
/* --- Telemetry timing Variables END --- */



/* ----- Telemetry Parameters BEGIN ----- */
uint8_t  tx_msg_id = 1;
int8_t   prev_dbm  = 22;
int8_t   curr_dbm  = 22;
uint16_t prev_freq = 900;
uint16_t curr_freq = 900;
/* ------ Telemetry Parameters END ------ */



/* --------- CAN-bus Variables BEGIN ----------- */
CAN_TxHeaderTypeDef TxHeader1;
CAN_RxHeaderTypeDef RxHeader1;
uint8_t TxData1[8];
uint8_t RxData1[8];

CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData2[8];
uint8_t RxData2[8];

uint32_t TxMailbox;

uint16_t can1_msgs_rx = 0;
uint16_t can2_msgs_rx = 0;
/* ---------- CAN-bus Variables END ------------ */




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef CanTxTPMSmessage(TelemetryData_t* teleData);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* MCU Configuration--------------------------------------------------------*/
  MCU_init(&mcu, &hspi1, GPIOA, NSS_Pin, GPIOB, NRESET_Pin, GPIOB,
		   BUSY_Pin, GPIOB, DIO1_Pin, GPIOB, DIO2_Pin, GPIOB, DIO3_Pin,
		   GPIOC, RF_SWITCH_Pin, GPIOC, RF_SWITCH_SUPP_Pin,
		   GPIOC, RADIO_TX_LED_Pin, GPIOD, RADIO_RX_LED_Pin,
		   GPIOA, CAN_TX_LED_Pin, GPIOC, CAN_RX_LED_Pin);

  /* TPMS Initializations */
  init_packet_types();
  TpmsStructsInit();
  radio.per.first_reception = true;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  SX1262GfskInit(&mcu, &radio, prev_dbm, prev_freq);
  //SX1262LoRaInit(&mcu, &radio, 22, 900);

  rx_tick = HAL_GetTick();


  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1results, 3);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2results, 3);

#if (FUNCTION == STNDBY)
	  SX1262SetFunction(&mcu, &radio, TPMS);
	  SX1262SetRxDutyCycle(&mcu, &radio, 200, 200);
#endif

  while (1)
  {
	if (radio.opMode != SLEEP_MODE)
		SX1262GetStatus(&mcu, &radio);


#if (FUNCTION == TELEMETRY_RX)
	{
		if (radio.opMode != RX_MODE)
			SX1262SetRxBoosted(&mcu, &radio, 0xFFFF);

		if (radio.msg_pending)
		{
			radio.rx_interval = HAL_GetTick() - rx_tick;
			rx_tick = HAL_GetTick();    // Used to calculate rx_interval

			SX1262dio1Interrupt(&mcu, &radio);
			SX1262GetStatus(&mcu, &radio);
			SX1262CalculatePacketLoss(&radio);
			SX1262ClearIrqStatus(&mcu, &radio, 0xFFFF);

			// Reset
			SX1262SetStandby(&mcu, &radio, STDBY_XOSC);
			SX1262SetRxBoosted(&mcu, &radio, 0xFFFF);

			process_rx_packet(&radio, &teleData);
			Transmit_USB(&radio, &teleData, radio.rx_msg[0]);


			radio.tele_rx_time = HAL_GetTick() - rx_tick;
			radio.msg_pending = false;
		}
	}
#endif

#if (FUNCTION == TELEMETRY_TX)

	if (HAL_GetTick() - tx_tick > tx_msg_delay)
	{
		radio.tx_interval = HAL_GetTick() - tx_tick;
		tx_tick = HAL_GetTick();

		CreateTeleTxMsg(&radio, tx_msg_id, &teleData);
		tx_msg_id++;
		if (tx_msg_id > PACKETS_NUM)
			tx_msg_id = 1;

		SX1262SendPayload(&mcu, &radio, radio.tx_msg, radio.packetParams.Params.Gfsk.PayloadLength - 2, 20, false);

		radio.tele_tx_time = HAL_GetTick() - tx_tick;
	}

	SX1262GetStatus(&mcu, &radio);
#endif

#if (FUNCTION == TELEMETRY_TPMS)
		if (tpms_msg_sent)  // TPMS sent message and a new time has to be decided
		{
			tpms_msg_time = HAL_GetTick() + tpms_interval;
			tpms_msg_sent = false;
		}

		if (HAL_GetTick() - tx_tick > tx_msg_delay)
		{
			radio.tx_interval = HAL_GetTick() - tx_tick;
			tx_tick = HAL_GetTick();

			/* TPMS Tx Message */
			SX1262SetFunction(&mcu, &radio, TPMS);

			if (config)
				SX1262CreateTpmsCfgrMsg(&radio, tpms_power, tpms_freq);
			else
			{
				wait_time = tpms_msg_time - HAL_GetTick() - 130; // -40
				switch (tpms_interval)
				{
				case (500):     /* 0.5s */
					 tx_freq = SAMPLE_0_5;
					 break;

				case (1000):     /* 1.0s */
					 tx_freq = SAMPLE_1_0;
					 break;

				case (3000):    /* 3.0s */
		             tx_freq = SAMPLE_3_0;
					 break;

				default:
					 break;
				}
				SX1262CreateTpmsRequestMsg(&radio, wait_time, tx_freq, cal_sleep_time, cal_wait_time);
			}

			SX1262SendPayload(&mcu, &radio, radio.tpms_tx_msg, radio.packetParams.Params.Gfsk.PayloadLength - 1, 5, true);
			radio.tpms_tx_time = HAL_GetTick() - tx_tick;

			/* Telemery Tx Message */
			tx_tick = HAL_GetTick();
			SX1262SetFunction(&mcu, &radio, TELEMETRY);

			CreateTeleTxMsg(&radio, tx_msg_id, &teleData);
			tx_msg_id++;
			if (tx_msg_id > PACKETS_NUM)
				tx_msg_id = 1;

			SX1262SendPayload(&mcu, &radio, radio.tx_msg, radio.packetParams.Params.Gfsk.PayloadLength - 2, 50, false);
			radio.tele_tx_time = HAL_GetTick() - tx_tick;
		}

		delay = (int64_t)(HAL_GetTick()) - tpms_msg_time;
		if (delay > window_start)  // 15 NORMALLY
		{
			radio.msg_pending = false;

			tpms_fr.not_received++;
			tpms_fl.not_received++;
			tpms_rr.not_received++;
			tpms_rl.not_received++;

			SX1262SetFunction(&mcu, &radio, TPMS);

//			tpms_msg_time = HAL_GetTick() + tpms_interval;

			SX1262SetRxBoosted(&mcu, &radio, 0xFFFF);
			tpms_msg_sent = true;
			current_tick = HAL_GetTick();
			width_start = HAL_GetTick() - tpms_msg_time;
			while (HAL_GetTick() - current_tick < window_width) // wait to get TPMS messages // 100 NORMALLY
			{
 				if (radio.msg_pending)
				{
 					radio.msg_pending = false;

					SX1262dio1Interrupt(&mcu, &radio);
					SX1262GetStatus(&mcu, &radio);
					SX1262ClearIrqStatus(&mcu, &radio, 0xFFFF);

					radio.tpms_variation = HAL_GetTick() - tpms_msg_time;

					TpmsGetMessage(&radio, &teleData);

					// Reset
					SX1262SetStandby(&mcu, &radio, STDBY_XOSC);
					SX1262SetRxBoosted(&mcu, &radio, 0xFFFF);
				}
			}
			width_end = HAL_GetTick() - tpms_msg_time;
			SX1262SetStandby(&mcu, &radio, STDBY_XOSC);

			CanTxTPMSmessage(&teleData);
		}
#endif

#if (FUNCTION == STANDBY)

		SX1262SetStandby(&mcu, &radio, STDBY_XOSC);
		//SX1262SetRxDutyCycle(&mcu, &radio, 200, 200);
		//SX1262GetStatus(&mcu, &radio);
#endif




	if ((prev_freq != curr_freq) || (prev_dbm != curr_dbm))
	{
		SX1262GfskInit(&mcu, &radio, curr_dbm, curr_freq);
		prev_freq = curr_freq;
		prev_dbm = curr_dbm;
	}


	if (HAL_GetTick() - last_linear_can_tx > linear_can_tx_interval)
	{
		last_linear_can_tx = HAL_GetTick();
		/* Sensory CAN BUS */
		TxHeader2.DLC = 4;
		TxHeader2.IDE = CAN_ID_STD;
		TxHeader2.RTR = CAN_RTR_DATA;
		TxHeader2.StdId = 0x4D2;
		TxHeader2.TransmitGlobalTime = DISABLE;
		TxData2[0] = (uint8_t)(linear_roll);
		TxData2[1] = (uint8_t)((linear_roll - TxData2[0]) * 100);
		TxData2[2] = (uint8_t)(linear_heave);
		TxData2[3] = (uint8_t)((linear_heave - TxData2[2]) * 100);
		HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox);

		aero_fans_can_counter++;
		if (aero_fans_can_counter > 9)  // ~1s
		{
			TxHeader2.DLC = 1;
			TxHeader2.IDE = CAN_ID_STD;
			TxHeader2.RTR = CAN_RTR_DATA;
			TxHeader2.StdId = 0x313;
			TxHeader2.TransmitGlobalTime = DISABLE;

			/* Ensures that vicor which supplies the aero fans is not overtemp */
			TxData2[0] = teleData.dash.aero_fans_on && (!teleData.accu.vicor_overtemp);
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox);

			aero_fans_can_counter = 0;
		}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef FilterConfig0;
  FilterConfig0.FilterIdHigh = 0;
  FilterConfig0.FilterIdLow = 0;
  FilterConfig0.FilterMaskIdHigh = 0;
  FilterConfig0.FilterMaskIdLow = 0;
  FilterConfig0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  FilterConfig0.FilterBank = 0;
  FilterConfig0.SlaveStartFilterBank = 14;
  FilterConfig0.FilterMode = CAN_FILTERMODE_IDMASK;
  FilterConfig0.FilterScale = CAN_FILTERSCALE_32BIT;
  FilterConfig0.FilterActivation = ENABLE;

  if(HAL_CAN_ConfigFilter(&hcan1, &FilterConfig0)!=HAL_OK) {
	  Error_Handler();
  }
  if(HAL_CAN_Start(&hcan1)!=HAL_OK) {
	  Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef FilterConfig1;
  FilterConfig1.FilterIdHigh = 0;
  FilterConfig1.FilterIdLow = 0;
  FilterConfig1.FilterMaskIdHigh = 0;
  FilterConfig1.FilterMaskIdLow = 0;
  FilterConfig1.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  FilterConfig1.FilterBank = 14;
  FilterConfig1.SlaveStartFilterBank = 14;
  FilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
  FilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
  FilterConfig1.FilterActivation = ENABLE;
  if(HAL_CAN_ConfigFilter(&hcan2, &FilterConfig1)!=HAL_OK) {
	  Error_Handler();
  }
  if(HAL_CAN_Start(&hcan2)!=HAL_OK) {
	  Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END CAN2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NSS_Pin|CAN_TX_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRESET_GPIO_Port, NRESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RF_SWITCH_Pin|RF_SWITCH_SUPP_Pin|CAN_RX_LED_Pin|RADIO_TX_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_RX_LED_GPIO_Port, RADIO_RX_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NSS_Pin CAN_TX_LED_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|CAN_TX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRESET_Pin */
  GPIO_InitStruct.Pin = NRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUSY_Pin DIO3_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin|DIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO2_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_SWITCH_Pin RF_SWITCH_SUPP_Pin CAN_RX_LED_Pin RADIO_TX_LED_Pin */
  GPIO_InitStruct.Pin = RF_SWITCH_Pin|RF_SWITCH_SUPP_Pin|CAN_RX_LED_Pin|RADIO_TX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_RX_LED_Pin */
  GPIO_InitStruct.Pin = RADIO_RX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_RX_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Primary CAN */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1)
	{
		HAL_GPIO_WritePin(mcu.can_rx_led.type, mcu.can_rx_led.pin, GPIO_PIN_SET);

		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, RxData1);
		can1_msgs_rx++;
		process_can1_rx_msg(&teleData, RxHeader1.StdId, RxData1);

		HAL_GPIO_WritePin(mcu.can_rx_led.type, mcu.can_rx_led.pin, GPIO_PIN_RESET);
	}
}

/* Sensory CAN */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan2)
	{
		HAL_GPIO_WritePin(mcu.can_rx_led.type, mcu.can_rx_led.pin, GPIO_PIN_SET);

		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader2, RxData2);
		can2_msgs_rx++;
		process_can2_rx_msg(&teleData, RxHeader2.StdId, RxData2);

		HAL_GPIO_WritePin(mcu.can_rx_led.type, mcu.can_rx_led.pin, GPIO_PIN_RESET);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == &hadc1)
	{
		adc1_conv_cplt++;

		if (adc1_conv_cplt < ADC_SAMPLES)
		{
			adc1_ch1_sum += adc1results[0];
			adc1_ch2_sum += adc1results[1];
			adc1_ch3_sum += adc1results[2];
		}
		else
		{
			adc1_conv_cplt = 0;

			adc1_ch1_value = (float)(adc1_ch1_sum) / ADC_SAMPLES;
			adc1_ch2_value = (float)(adc1_ch2_sum) / ADC_SAMPLES;
			adc1_ch3_value = (float)(adc1_ch3_sum) / ADC_SAMPLES;

			linear_roll  = 74.8 - 75.0 * (adc1_ch1_value / 4096);


			adc1_ch1_sum = 0;
			adc1_ch2_sum = 0;
			adc1_ch3_sum = 0;
		}
	}

	else if (hadc == &hadc2)
	{
		adc2_conv_cplt++;

		if (adc2_conv_cplt < ADC_SAMPLES)
		{
			adc2_ch12_sum += adc2results[0];
			adc2_ch13_sum += adc2results[1];
			adc2_ch15_sum += adc2results[2];
		}
		else
		{
			adc2_conv_cplt = 0;

			adc2_ch12_value = (float)(adc2_ch12_sum) / ADC_SAMPLES;
			adc2_ch13_value = (float)(adc2_ch13_sum) / ADC_SAMPLES;
			adc2_ch15_value = (float)(adc2_ch15_sum) / ADC_SAMPLES;

			linear_heave = 74.8 - 75.0 * (adc2_ch15_value / 4096);

			adc2_ch12_sum = 0;
			adc2_ch13_sum = 0;
			adc2_ch15_sum = 0;
		}
	}
}



HAL_StatusTypeDef CanTxTPMSmessage(TelemetryData_t* teleData)
{
	HAL_StatusTypeDef result;

	TxHeader2.DLC = 8;
	TxHeader2.IDE = CAN_ID_STD;
	TxHeader2.RTR = CAN_RTR_DATA;
	TxHeader2.StdId = 0x4F2;
	TxHeader2.TransmitGlobalTime = DISABLE;
	TxData2[0] = teleData->tpms.fr_press >> 8;    /* Gives a resolution of 1mBar */
	TxData2[1] = teleData->tpms.fr_press;
	TxData2[2] = teleData->tpms.fl_press >> 8;
	TxData2[3] = teleData->tpms.fl_press;
	TxData2[4] = teleData->tpms.rr_press >> 8;
	TxData2[5] = teleData->tpms.rr_press;
	TxData2[6] = teleData->tpms.rl_press >> 8;
	TxData2[7] = teleData->tpms.rl_press;
	result = HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox);
	if (result != HAL_OK)
		return result;


	TxHeader2.DLC = 8;
	TxHeader2.IDE = CAN_ID_STD;
	TxHeader2.RTR = CAN_RTR_DATA;
	TxHeader2.StdId = 0x4F3;
	TxHeader2.TransmitGlobalTime = DISABLE;
	TxData2[0] = teleData->tpms.fr_temp >> 8;
	TxData2[1] = teleData->tpms.fr_temp;
	TxData2[2] = teleData->tpms.fl_temp >> 8;
	TxData2[3] = teleData->tpms.fl_temp;
	TxData2[4] = teleData->tpms.rr_temp >> 8;
	TxData2[5] = teleData->tpms.rr_temp;
	TxData2[6] = teleData->tpms.rl_temp >> 8;
	TxData2[7] = teleData->tpms.rl_temp;
	result = HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox);

	return result;
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
