
#ifndef PRESSSENS_H_
#define PRESSSENS_H_

#include "stdio.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stdbool.h"
#include "SX1262_board.h"

typedef enum
{
	reset_cmd     = 0x1E,  // 00011110
	adc_read_cmd  = 0x00,  // 00000000
	prom_read_cmd = 0xA0,  // 10100000   Does not include Ad2, Ad1, Ad0

}Command_t;



/* Temperature Conversion Sampling Ratios */
typedef enum
{
	TEMP_OSR_256  = 0x50,  // 01010000
	TEMP_OSR_512  = 0x52,  // 01010010
	TEMP_OSR_1024 = 0x54,  // 01010100
	TEMP_OSR_2048 = 0x56,  // 01010110
	TEMP_OSR_4096 = 0x58,  // 01011000

}Temp_osr_t;



/* Pressure Conversion Sampling Ratios */
typedef enum
{
	PRESS_OSR_256  = 0x40,  // 01000000
	PRESS_OSR_512  = 0x42,  // 01000010
	PRESS_OSR_1024 = 0x44,  // 01000100
	PRESS_OSR_2048 = 0x46,  // 01000110
	PRESS_OSR_4096 = 0x48,  // 01001000

}Press_osr_t;



typedef enum
{
	I2C_OK    = 0,
	I2C_ERROR = 1,

}I2Cstatus_t;



typedef struct Pressure_Sensor
{
  I2C_HandleTypeDef *i2cHandle;

  /* VSENSE Pin - Supplies the sensor and activates the I2C pull-ups */
  GPIO_TypeDef* VSENSE_port;
  uint16_t      VSENSE_pin;

  /* Memory */
  uint16_t PROM[8];  // cell PROM[7] used only for CRC4 calculation
  uint32_t temperature_reg;
  uint32_t pressure_reg;

  float temperature;  // degrees Celcius
  float pressure;     // mbar

  uint8_t prom_crc;
  uint8_t calculated_crc;

} Pressure_Sensor;



void pressure_sensor_struct_init(Pressure_Sensor *pr, I2C_HandleTypeDef *i2c_Handle, GPIO_TypeDef* VSENSE_port, uint16_t VSENSE_pin);

I2Cstatus_t reset(const Pressure_Sensor *pr);
I2Cstatus_t read_prom(Pressure_Sensor *pr, const uint8_t reg_address);
I2Cstatus_t get_temperatures(Pressure_Sensor *pr, const Temp_osr_t osr);
I2Cstatus_t read_pressure_register(Pressure_Sensor *pr, const Press_osr_t osr);
void calibrate(Pressure_Sensor *pr);
I2Cstatus_t check_crc(Pressure_Sensor *pr);

I2Cstatus_t sensor_init(Pressure_Sensor *pr, I2C_HandleTypeDef *i2c_Handle, GPIO_TypeDef* VSENSE_port, uint16_t VSENSE_pin);
I2Cstatus_t sensor_measure(Pressure_Sensor *pr, const Temp_osr_t temp_osr, const Press_osr_t press_osr);

I2Cstatus_t get_adc_conversion(Pressure_Sensor *pr, uint8_t osr);
unsigned char crc4(unsigned short int n_prom[]);
void delay(const uint16_t ms);










#endif /* PRESSSENS_H_ */
