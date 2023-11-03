/* This file needs modifications according to the STM model and the PCB layout */

#include <SX1262_board.h>
#include "stm32l4xx_hal.h"
#include "SX1262.h"


void Gpio_init(Gpio_t* GPIO, GPIO_TypeDef *type, uint16_t pin) {

	GPIO->type = type;
	GPIO->pin  = pin;
}


void SPI_init(Spi_t* SPI, SPI_HandleTypeDef *handle, GPIO_TypeDef *type, uint16_t pin) {

    SPI->spiHandle = handle;
    SPI->NSS.pin    = pin;
    SPI->NSS.type   = type;
}


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
			  GPIO_TypeDef *CAN_RX_LED, uint16_t CAN_RX_LED_pin)
{
	Gpio_init(&(mcu->spi.NSS), NSS, NSS_pin);

	SPI_init(&(mcu->spi), spiHandle, NSS, NSS_pin);

	Gpio_init(&(mcu->nreset), NRESET, NRESET_pin);
	Gpio_init(&(mcu->busy), BUSY, BUSY_pin);
	Gpio_init(&(mcu->dio1), DIO1, DIO1_pin);
	Gpio_init(&(mcu->dio2), DIO2, DIO2_pin);
	Gpio_init(&(mcu->dio3), DIO3, DIO3_pin);
	Gpio_init(&(mcu->rf_switch), RF_SWITCH, RF_SWITCH_pin);
	Gpio_init(&(mcu->rf_switch_supp), RF_SWITCH_SUPP, RF_SWITCH_SUPP_pin);

	Gpio_init(&(mcu->radio_tx_led), RADIO_TX_LED, RADIO_TX_LED_pin);
	Gpio_init(&(mcu->radio_rx_led), RADIO_RX_LED, RADIO_RX_LED_pin);
	Gpio_init(&(mcu->can_tx_led), CAN_TX_LED, CAN_TX_LED_pin);
	Gpio_init(&(mcu->can_rx_led), CAN_RX_LED, CAN_RX_LED_pin);

	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_SET);
}

#else
void MCU_init(MCU *mcu, SPI_HandleTypeDef *spiHandle,
		      GPIO_TypeDef *NSS, uint16_t NSS_pin,
		      GPIO_TypeDef *NRESET, uint16_t NRESET_pin,
			  GPIO_TypeDef *BUSY, uint16_t BUSY_pin,
			  GPIO_TypeDef *DIO1, uint16_t DIO1_pin,
			  GPIO_TypeDef *DIO2, uint16_t DIO2_pin,
			  GPIO_TypeDef *DIO3, uint16_t DIO3_pin,
			  GPIO_TypeDef *RF_SWITCH, uint16_t RF_SWITCH_pin,
			  GPIO_TypeDef *RF_SWITCH_SUPP, uint16_t RF_SWITCH_SUPP_pin)
{
	Gpio_init(&(mcu->spi.NSS), NSS, NSS_pin);

	SPI_init(&(mcu->spi), spiHandle, NSS, NSS_pin);

	Gpio_init(&(mcu->nreset), NRESET, NRESET_pin);
	Gpio_init(&(mcu->busy), BUSY, BUSY_pin);
	Gpio_init(&(mcu->dio1), DIO1, DIO1_pin);
	Gpio_init(&(mcu->dio2), DIO2, DIO2_pin);
	Gpio_init(&(mcu->dio3), DIO3, DIO3_pin);
	Gpio_init(&(mcu->rf_switch), RF_SWITCH, RF_SWITCH_pin);
	Gpio_init(&(mcu->rf_switch_supp), RF_SWITCH_SUPP, RF_SWITCH_SUPP_pin);

	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_SET);
}
#endif


