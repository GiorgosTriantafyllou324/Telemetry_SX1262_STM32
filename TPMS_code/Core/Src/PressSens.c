#include "stdio.h"
#include "PressSens.h"
#include "stdbool.h"
#include <stdint.h>

#define DEV_ADDRESS    0x76  // 01110110
#define TIMEOUT		   50

void pressure_sensor_struct_init(Pressure_Sensor *pr, I2C_HandleTypeDef *i2c_Handle, GPIO_TypeDef* VSENSE_port, uint16_t VSENSE_pin)
{

	pr->i2cHandle = i2c_Handle;

	pr->VSENSE_port = VSENSE_port;
	pr->VSENSE_pin  = VSENSE_pin;

	/* Memory */
	for (uint8_t i = 0; i < 8; ++i)
	  pr->PROM[i] = 0;

	pr->temperature_reg = 0;
	pr->pressure_reg    = 0;
	pr->temperature     = 0.0;
	pr->pressure    	= 0.0;

	pr->prom_crc       = 0;
	pr->calculated_crc = 0;
}


I2Cstatus_t reset(const Pressure_Sensor *pr) {

	uint8_t reset_command = reset_cmd;
	 if (HAL_I2C_Master_Transmit(pr->i2cHandle, DEV_ADDRESS << 1, &reset_command, 1, TIMEOUT) != HAL_OK)
		 return I2C_ERROR;
	 return I2C_OK;
}


I2Cstatus_t read_prom(Pressure_Sensor *pr, const uint8_t reg_address) {

	uint8_t prom_data[2];
	uint8_t PROM_read_command = prom_read_cmd | (reg_address << 1);

	if (HAL_I2C_Master_Transmit(pr->i2cHandle, DEV_ADDRESS << 1, &PROM_read_command, 1, TIMEOUT) != HAL_OK)
		return I2C_ERROR;

	if (HAL_I2C_Master_Receive(pr->i2cHandle, DEV_ADDRESS << 1, prom_data, 2, TIMEOUT) != HAL_OK)
		return I2C_ERROR;

	pr->PROM[reg_address] = (prom_data[0] << 8) | prom_data[1];

	return I2C_OK;
}


I2Cstatus_t get_temperatures(Pressure_Sensor *pr, const Temp_osr_t osr) {

	if (get_adc_conversion(pr, osr) != I2C_OK)
		return I2C_ERROR;

	return I2C_OK;
}


I2Cstatus_t read_pressure_register(Pressure_Sensor *pr, const Press_osr_t osr) {

	if (get_adc_conversion(pr, osr) != I2C_OK)
		return I2C_ERROR;

	return I2C_OK;
}


void calibrate(Pressure_Sensor *pr) {

	/* Temperature Calibration */
	int32_t dT   = pr->temperature_reg - ((uint32_t)(pr->PROM[5]) << 8);
	int32_t temp = 2000 + ((int64_t)dT * pr->PROM[6] >> 23);

	/* Pressure Calibration */
	int64_t off  = ((int64_t)(pr->PROM[2]) << 16) + (((int64_t)(pr->PROM[4]) * dT) >> 7);
	int64_t sens = ((int64_t)(pr->PROM[1]) << 15) + (((int64_t)(pr->PROM[3]) * dT) >> 8);

	/* Pressure without second order temperature compensation */
	int32_t p = ((pr->pressure_reg * sens >> 21) - off) >> 13;

	/* Second Order Temperature Compensation */
	int64_t ti, offi, sensi;

	if (temp > 2000) {
		ti    = (dT * dT) >> 36;
		offi  = ((temp - 2000) * (temp - 2000)) >> 4;
		sensi = 0;
	}
	else {
		ti    = (3 * (int64_t)dT * (int64_t)dT) >> 33;
		offi  = (3 * (temp - 2000) * (temp - 2000)) >> 1;
		sensi = (5 * (temp - 2000) * (temp - 2000)) >> 3;

		if (temp < -1500) {
			offi  = offi + 7 * (temp + 1500) * (temp + 1500);
			sensi = sensi + 4 * (temp + 1500) * (temp + 1500);
		}
	}
	pr->temperature = (temp - ti) * 0.01;
	pr->pressure    = ((((pr->pressure_reg * (sens - sensi))/ 2097152) - (off - offi)) / 8192) * 0.1;

	return;
}


I2Cstatus_t check_crc(Pressure_Sensor *pr) {

	pr->prom_crc = pr->PROM[0] >> 12;
	pr->calculated_crc = crc4(pr->PROM);

	if (pr->prom_crc != pr->calculated_crc)
		return I2C_ERROR;

	return I2C_OK;
}


I2Cstatus_t sensor_init(Pressure_Sensor *pr, I2C_HandleTypeDef *i2c_Handle, GPIO_TypeDef* VSENSE_port, uint16_t VSENSE_pin) {

	pressure_sensor_struct_init(pr, i2c_Handle, VSENSE_port, VSENSE_pin);
	HAL_Delay(2);

	HAL_GPIO_WritePin(pr->VSENSE_port, pr->VSENSE_pin, GPIO_PIN_SET);

	if (reset(pr) != I2C_OK)
		return I2C_ERROR;

	for (uint8_t reg_address = 0; reg_address < 7; ++reg_address)
		if (read_prom(pr, reg_address) != I2C_OK)
			return I2C_ERROR;

	if (check_crc(pr) != I2C_OK)
		return I2C_ERROR;

	return I2C_OK;
}


I2Cstatus_t sensor_measure(Pressure_Sensor *pr, const Temp_osr_t temp_osr, const Press_osr_t press_osr) {

	HAL_GPIO_WritePin(pr->VSENSE_port, pr->VSENSE_pin, GPIO_PIN_SET);
	HAL_Delay(2);

	if (get_temperatures(pr, temp_osr) != I2C_OK)
		return I2C_ERROR;

	if (read_pressure_register(pr, press_osr) != I2C_OK)
		return I2C_ERROR;

#if FUNCTION != TPMS_RELEASE
	calibrate(pr);
#endif

	return I2C_OK;
}


/* ---------------------------------- Private Functions --------------------------------------- */
I2Cstatus_t get_adc_conversion(Pressure_Sensor *pr, uint8_t osr) {

	uint8_t adc_data_array[3];

	if (HAL_I2C_Master_Transmit(pr->i2cHandle, DEV_ADDRESS << 1, &osr, 1, TIMEOUT) != HAL_OK)
		return I2C_ERROR;

	if ((osr < PRESS_OSR_2048) || ((osr >= TEMP_OSR_256) && (osr < TEMP_OSR_2048)))
		delay(4);
	else
		delay(11);

	uint8_t adc_read_command = adc_read_cmd;
	if (HAL_I2C_Master_Transmit(pr->i2cHandle, DEV_ADDRESS << 1, &adc_read_command, 1, TIMEOUT) != HAL_OK)
		return I2C_ERROR;

	if (HAL_I2C_Master_Receive(pr->i2cHandle, DEV_ADDRESS << 1, adc_data_array, 3, TIMEOUT) != HAL_OK)
		return I2C_ERROR;

	if (osr < TEMP_OSR_256) // Pressure measurement
		pr->pressure_reg = (adc_data_array[0] << 16) | (adc_data_array[1] << 8) | adc_data_array[0];

	else			  // Temperature measurement
		pr->temperature_reg = (adc_data_array[0] << 16) | (adc_data_array[1] << 8) | adc_data_array[0];

	return I2C_OK;
}


unsigned char crc4(unsigned short int n_prom[]) {  // Firstly, n_prom[] was unsigned int  // n_prom defined as 8x unsigned int (n_prom[8])

	int cnt; // simple counter
	unsigned int n_rem = 0; // crc remainder
	unsigned char n_bit;
	n_prom[0] = ((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
	n_prom[7] = 0; // Subsidiary value, set to 0

	for (cnt = 0; cnt < 16; cnt++) {  // operation is performed on bytes
		if (cnt % 2 == 1)
			n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else
			n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000))
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem = (n_rem << 1);
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
	return (n_rem ^ 0x00);
}


void delay(const uint16_t ms) {

	uint64_t tick0 = HAL_GetTick();
	while(HAL_GetTick() - tick0 < ms);
	return;
}

/* I2C functions */
//HAL_I2C_Master_Transmit(hi2c1, DevAddress, pData, Size, Timeout);
//HAL_I2C_Master_Receive(hi2c1, DevAddress, pData, Size, Timeout);
//HAL_I2C_Mem_Write(hi2c1, DevAddress, MemAddress, MemAddressSize, pData, size, Timeout);
//HAL_I2C_Mem_Read(hi2c1, DevAddress, MemAddress, MemAddressSize, pData, size, Timeout);

