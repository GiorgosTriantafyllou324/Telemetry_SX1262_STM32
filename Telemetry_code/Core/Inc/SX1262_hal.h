
#ifndef SX1262_HAL_H_
#define SX1262_HAL_H_

#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include <stdint.h>
#include <string.h>

#include "SX1262_board.h"



/* Forward declaration from SX1262.h file */
struct Radio;
typedef struct Radio Radio;



typedef enum
{
	RADIO_OK                    = 0,
	RADIO_SAFE_WRITE_ERROR      = 1,
	RADIO_PAYLOAD_LENGTH_ERROR  = 2,

}RadioState_t;



void SX1262WriteCommand(MCU* master, Radio* radio, uint8_t command, uint8_t* data, uint8_t len);
void SX1262ReadCommand(MCU* mcu, Radio* radio, uint8_t command, uint8_t len);

void SX1262WriteRegister(MCU* master, Radio* radio, uint16_t address, uint8_t* data, uint8_t len);
void SX1262ReadRegister(MCU* master, Radio* radio, uint16_t address, uint8_t *data, uint8_t len);
RadioState_t SX1262SafeWriteRegister(MCU* mcu, Radio* radio, uint16_t address, uint8_t* data, uint8_t len);

void SX1262WriteBuffer(MCU* mcu, Radio* radio, uint8_t offset, uint8_t *data, uint8_t len);
void SX1262ReadBuffer(MCU* mcu, Radio* radio, uint8_t offset, uint8_t *data, uint8_t len);
RadioState_t SX1262SafeWriteBuffer(MCU* mcu, Radio* radio, uint8_t offset, uint8_t *data, uint8_t len);



#endif /* SX1262_HAL_H_ */
