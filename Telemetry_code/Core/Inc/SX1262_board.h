#ifndef SX1262_BOARD_H_
#define SX1262_BOARD_H_

#include "stm32f4xx_hal.h"

#define TCXO_MAX_VOLTAGE TCXO_1_8V  // Maximum operating voltage that the TCXO used on the PCB can stand - Protection purposes
#define TCXO_SETUP_TIME  50         // [ms]

#define SPI_TIMEOUT    50   // [ms]
#define MAX_RX_BYTES   255
#define TPMS_MSG_BYTES 5
#define READ_MSG_LEN   200   // Length of the read_msg array (SPI) used for debugging

#define TELEMETRY_TX   0
#define TELEMETRY_RX   1
#define TELEMETRY_TPMS 2
#define TPMS_DEBUG     3
#define TPMS_RELEASE   4
#define STNDBY         5

#define FUNCTION TELEMETRY_TPMS




typedef struct Gpio_t
{
	GPIO_TypeDef   *type;
	uint16_t        pin;

}Gpio_t;

void Gpio_init(Gpio_t* GPIO, GPIO_TypeDef *type, uint16_t pin);



typedef struct Spi_t
{
	SPI_HandleTypeDef *spiHandle;
	Gpio_t             NSS;

}Spi_t;

void SPI_init(Spi_t* SPI, SPI_HandleTypeDef *handle, GPIO_TypeDef *type, uint16_t pin);



typedef struct MCU
{
	Spi_t spi;

	Gpio_t nreset;
	Gpio_t busy;
	Gpio_t dio1;
	Gpio_t dio2;
	Gpio_t dio3;
	Gpio_t rf_switch;
	Gpio_t rf_switch_supp;

#if ((FUNCTION != TPMS_DEBUG) && (FUNCTION != TPMS_RELEASE))
	Gpio_t radio_tx_led;
	Gpio_t radio_rx_led;
	Gpio_t can_tx_led;
	Gpio_t can_rx_led;
#endif

}MCU;

#if ((FUNCTION != TPMS_DEBUG) && (FUNCTION != TPMS_RELEASE))
void MCU_init(MCU *mcu, SPI_HandleTypeDef *spiHandle,
		      GPIO_TypeDef *NSS, uint16_t NSS_pin,
		      GPIO_TypeDef *NRESET, uint16_t NRESET_pin,
			  GPIO_TypeDef *BUSY, uint16_t BUSY_pin,
			  GPIO_TypeDef *DIO1, uint16_t DIO1_pin,
			  GPIO_TypeDef *DIO2, uint16_t DIO2_pin,
			  GPIO_TypeDef *DIO3, uint16_t DIO3_pin,
			  GPIO_TypeDef *RF_SWITCH, uint16_t RF_SWITCH_pin,
			  GPIO_TypeDef *RF_SWITCH_SUPP, uint16_t RF_SWITCH_SUPP_pin,
			  GPIO_TypeDef *RADIO_TX_LED, uint16_t RADIO_TX_LED_pin,
			  GPIO_TypeDef *RADIO_RX_LED, uint16_t RADIO_RX_LED_pin,
			  GPIO_TypeDef *CAN_TX_LED, uint16_t CAN_TX_LED_pin,
			  GPIO_TypeDef *CAN_RX_LED, uint16_t CAN_RX_LED_pin);
#else
void MCU_init(MCU *mcu, SPI_HandleTypeDef *spiHandle,
		      GPIO_TypeDef *NSS, uint16_t NSS_pin,
		      GPIO_TypeDef *NRESET, uint16_t NRESET_pin,
			  GPIO_TypeDef *BUSY, uint16_t BUSY_pin,
			  GPIO_TypeDef *DIO1, uint16_t DIO1_pin,
			  GPIO_TypeDef *DIO2, uint16_t DIO2_pin,
			  GPIO_TypeDef *DIO3, uint16_t DIO3_pin,
			  GPIO_TypeDef *RF_SWITCH, uint16_t RF_SWITCH_pin,
			  GPIO_TypeDef *RF_SWITCH_SUPP, uint16_t RF_SWITCH_SUPP_pin);
#endif




#endif /* SX1262_BOARD_H_ */
