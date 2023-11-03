
#include <stdint.h>
#include <string.h>

//#include "SX1262_board.h"
#include "SX1262_hal.h"
#include "SX1262.h"


uint8_t test = 0;

/*****************************************************************************
*  @Description	  Transmits through SPI the requested command and writes
*                 the data to the chip
******************************************************************************/
void SX1262WriteCommand(MCU* mcu, Radio* radio, uint8_t command, uint8_t* data, uint8_t len)
{
	uint8_t write_msg[len + 1];
	uint8_t read_msg[len + 1];
	for (uint8_t i = 0; i < len + 1; ++i)
	{
		write_msg[i] = 0;
		read_msg[i]  = 0;
	}

	write_msg[0] = command;
	for (uint8_t pos = 0; pos < len; ++pos)
		write_msg[pos + 1] = data[pos];

	if (len > 20)
		test++;

	if (radio->opMode != SLEEP_MODE || command != SET_STANDBY) // Wakeup command is sent even when Busy Pin is HIGH
		while(HAL_GPIO_ReadPin(mcu->busy.type, mcu->busy.pin) != GPIO_PIN_RESET);  // Wait until BUSY pin goes LOW
	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mcu->spi.spiHandle, write_msg, read_msg, sizeof(write_msg), SPI_TIMEOUT);
	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_SET);

	// Debugging
	uint8_t limit;
	if (len + 1 <= READ_MSG_LEN)
		limit = len + 1;
	else
		limit = READ_MSG_LEN;

	for (uint8_t i = 0; i < limit; ++i)
		radio->spi_read_msg[i] = read_msg[i];
	for (uint8_t i = len + 1; i < READ_MSG_LEN; ++i)
		radio->spi_read_msg[i] = 0;

	for (uint8_t cnt = 15; cnt < 199; ++cnt)
		if (radio->spi_read_msg[cnt] != 0)
			test++;
}


/*****************************************************************************
*  @Description	  Transmits through SPI the requested command and reads the
*                 data sent from the chip
******************************************************************************/
void SX1262ReadCommand(MCU* mcu, Radio* radio, uint8_t command, uint8_t len)
{
	uint8_t write_msg[len + 2];
	uint8_t read_msg[len + 2];
	for (uint8_t i = 0; i < len + 2; ++i)
	{
		write_msg[i] = 0;
		read_msg[i]  = 0;
	}

	write_msg[0] = command;

	while(HAL_GPIO_ReadPin(mcu->busy.type, mcu->busy.pin) != GPIO_PIN_RESET);  // Wait until BUSY pin goes LOW
	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mcu->spi.spiHandle, write_msg, read_msg, sizeof(write_msg), SPI_TIMEOUT);
	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_SET);

	// Debugging
	uint8_t limit;
	if (len + 2 <= READ_MSG_LEN)
		limit = len + 2;
	else
		limit = READ_MSG_LEN;

	for (uint8_t i = 0; i < limit; ++i)
		radio->spi_read_msg[i] = read_msg[i];
	for (uint8_t i = len + 2; i < READ_MSG_LEN; ++i)
		radio->spi_read_msg[i] = 0;

	for (uint8_t cnt = 15; cnt < 199; ++cnt)
		if (radio->spi_read_msg[cnt] != 0)
			test++;
}


/*****************************************************************************
*  @Description	  Writes to the requested register. Each Register address
*                 corresponds to 1 Byte memory. If data is more than
*                 1 byte, the given address is the address of the 1st Byte
******************************************************************************/
void SX1262WriteRegister(MCU* mcu, Radio* radio, uint16_t address, uint8_t* data, uint8_t len)
{

	uint8_t write_msg[len + 2];
	for (uint8_t i = 0; i < len + 2; ++i)
		write_msg[i] = 0;

	write_msg[0] = address >> 8;
	write_msg[1] = address;
	for (uint8_t i = 2; i < len + 2; ++i)
		write_msg[i] = data[i - 2];

	SX1262WriteCommand(mcu, radio, WRITE_REGISTER, write_msg, sizeof(write_msg));
}


/*****************************************************************************
*  @Description	  Reads from the requested register. Each Register address
*                 corresponds to 1 Byte memory. If the requested data is more
*                 than 1 byte (len > 1), the given address is the address of
*                 the 1st Byte
******************************************************************************/
void SX1262ReadRegister(MCU* mcu, Radio* radio, uint16_t address, uint8_t *data, uint8_t len) // len = # of Bytes of the received message
{
	uint8_t write_msg[len + 4];
	uint8_t read_msg[len + 4];
	for (uint8_t i = 0; i < len + 4; ++i)
	{
		write_msg[i] = 0;
		read_msg[i]  = 0;
	}

	write_msg[0] = READ_REGISTER;
	write_msg[1] = address >> 8;
	write_msg[2] = address;

	while(HAL_GPIO_ReadPin(mcu->busy.type, mcu->busy.pin) != GPIO_PIN_RESET);  // Wait until BUSY pin goes LOW
	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mcu->spi.spiHandle, write_msg, read_msg, sizeof(write_msg), SPI_TIMEOUT);
	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_SET);

	for (uint8_t i = 0; i < len; ++i)
		data[i] = read_msg[i + 4];

	// Debugging
	uint8_t limit;
	if (len + 4 <= READ_MSG_LEN)
		limit = len + 4;
	else
		limit = READ_MSG_LEN;

	for (uint8_t i = 0; i < limit; ++i)
		radio->spi_read_msg[i] = read_msg[i];
	for (uint8_t i = len + 4; i < READ_MSG_LEN; ++i)
		radio->spi_read_msg[i] = 0;
}


/*****************************************************************************
*  @Description	  Writes to the requested register and then reads back to
*                 ensure that the data has been written successfully.
******************************************************************************/
RadioState_t SX1262SafeWriteRegister(MCU* mcu, Radio* radio, uint16_t address, uint8_t* data, uint8_t len)
{
	uint8_t received_data[len];
	for (uint8_t i = 0; i < len; ++i)
		received_data[i] = 0;

	SX1262WriteRegister(mcu, radio, address, data, len);
	SX1262ReadRegister(mcu, radio, address, received_data, sizeof(received_data));

	for (uint8_t i = 0; i < len; ++i)
		if (data[i] != received_data[i])
			return RADIO_SAFE_WRITE_ERROR;
	return RADIO_OK;
}


/*****************************************************************************
*  @Description	  Writes to the buffer. The buffer has 256 positions so the
*                 offset (0 - 255) has to be given. The buffer is used for
*                 transmitted / received payload
******************************************************************************/
void SX1262WriteBuffer(MCU* mcu, Radio* radio, uint8_t offset, uint8_t *data, uint8_t len)
{
	uint8_t write_msg[len + 1];
	for (uint8_t i = 0; i < len + 1; ++i)
		write_msg[i] = 0;

	write_msg[0] = offset;
	for (uint8_t i = 0; i < len; ++i)
		write_msg[i + 1] = data[i];

	if (len > 40)
		test++;

	SX1262WriteCommand(mcu, radio, WRITE_BUFFER, write_msg, sizeof(write_msg));
}


/*****************************************************************************
*  @Description	  Reads from the buffer. The buffer has 256 positions so the
*                 offset (0 - 255) has to be given. The buffer is used for
*                 transmitted / received payload
******************************************************************************/
void SX1262ReadBuffer(MCU* mcu, Radio* radio, uint8_t offset, uint8_t *received_data, uint8_t len)
{
	/*
	GetRxBufferStatus(rxDataLen, &offset);
	if( *rxDataLen > maxLen ) {
	    return 1;
	}
	while(digitalRead(SX126x_BUSY));
	*/

	uint8_t write_msg[len + 3];
	uint8_t read_msg[len + 3];
	for (uint8_t i = 0; i < len + 3; ++i)
	{
		write_msg[i] = 0;
		read_msg[i]  = 0;
	}

	write_msg[0] = READ_BUFFER;
	write_msg[1] = offset;

	while(HAL_GPIO_ReadPin(mcu->busy.type, mcu->busy.pin) != GPIO_PIN_RESET);  // Wait until BUSY pin goes LOW
	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(mcu->spi.spiHandle, write_msg, read_msg, sizeof(write_msg), SPI_TIMEOUT);
	HAL_GPIO_WritePin(mcu->spi.NSS.type, mcu->spi.NSS.pin, GPIO_PIN_SET);

	for (uint8_t i = 0; i < len; ++i)
		received_data[i] = read_msg[i + 3];


	// Debugging
	uint8_t limit;
	if (len + 3 <= READ_MSG_LEN)
		limit = len + 3;
	else
		limit = READ_MSG_LEN;

	for (uint8_t i = 0; i < limit; ++i)
		radio->spi_read_msg[i] = read_msg[i];
	for (uint8_t i = len + 3; i < READ_MSG_LEN; ++i)
		radio->spi_read_msg[i] = 0;
}


/*****************************************************************************
*  @Description	  Writes to the buffer and then reads back to
*                 ensure that the data has been written successfully.
******************************************************************************/
RadioState_t SX1262SafeWriteBuffer(MCU* mcu, Radio* radio, uint8_t offset, uint8_t *data, uint8_t len)
{
	uint8_t received_data[len];
	for (uint8_t i = 0; i < len; ++i)
		received_data[i] = 0;

	SX1262WriteBuffer(mcu, radio, offset, data, len);
	SX1262ReadBuffer(mcu, radio, offset, received_data, sizeof(received_data));

	for (uint8_t i = 0; i < len; ++i)
		if (data[i] != received_data[i])
			return RADIO_SAFE_WRITE_ERROR;
	return RADIO_OK;
}



