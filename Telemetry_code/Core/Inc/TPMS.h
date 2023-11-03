#ifndef INC_TPMS_H_
#define INC_TPMS_H_

#include <SX1262.h>
#include <TeleMsgs_USB.h>



typedef struct
{
	uint8_t  id;
	uint16_t PROM[7]; /* PressSens.h library has PROM with 8 cells because of additional CRC calculations */

	uint32_t temperature_reg;
	uint32_t pressure_reg;

	float temperature;  // [degrees Celcius]
	float pressure;     // [mbar]


	int8_t   rssi;            // RSSI value received from the last packet sent by that sensor
	uint16_t wrongCRC;        // Counts the wrong CRC messages for every TPMS board
	int32_t  time_variation;  // HAL_GetTick() - tpms_msg_time
	uint16_t received_msgs;   // Counts the received messages from that specific TPMS sensor
	uint16_t not_received;    // increments every time an expected message does not arrive

}TPMS_data_t;


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



extern PacketParams_t tpms_packet;
extern PacketParams_t tele_packet;

extern uint8_t TeleGfskSyncWord[8];
extern uint8_t TpmsGfskSyncWord[3];

extern TPMS_data_t tpms_fr, tpms_fl, tpms_rr, tpms_rl;


void TpmsStructsInit(void);
void init_packet_types(void);
void SX1262CreateTpmsRequestMsg(Radio* radio, uint16_t delay_ms, uint8_t tx_freq, int8_t sleep_time, int8_t wait_time);
void SX1262CreateTpmsCfgrMsg(Radio* radio,  int8_t power, uint16_t freqInMHz);
RadioState_t SX1262SetFunction(MCU* mcu, Radio* radio, Function_t function);
void TpmsCalibrateSensorData(uint32_t temperature_reg, uint32_t pressure_reg, uint8_t id);
void TpmsGetMessage(Radio* radio, TelemetryData_t* teleData);



#endif /* INC_TPMS_H_ */
