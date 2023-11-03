#include <TPMS.h>

PacketParams_t tpms_packet;
PacketParams_t tele_packet;

uint8_t TpmsGfskSyncWord[3] = {0xF0, 0xF0, 0xF0};
uint8_t TeleGfskSyncWord[8] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0};

TPMS_data_t tpms_fr, tpms_fl, tpms_rr, tpms_rl;


/* PROM memory is written differently and it's unique for each sensor
 * It has to do with the manufacturing process of the sensor          */
void TpmsStructsInit(void)
{
	/* TPMS Front Right */
	tpms_fr.id = 0;
	tpms_fr.PROM[0] = 1;
	tpms_fr.PROM[1] = 29759;
	tpms_fr.PROM[2] = 29011;
	tpms_fr.PROM[3] = 18234;
	tpms_fr.PROM[4] = 18934;
	tpms_fr.PROM[5] = 27339;
	tpms_fr.PROM[6] = 26943;

	/* TPMS Front Left */
	tpms_fl.id = 1;
	tpms_fl.PROM[0] = 1;
	tpms_fl.PROM[1] = 30183;
	tpms_fl.PROM[2] = 28072;
	tpms_fl.PROM[3] = 18109;
	tpms_fl.PROM[4] = 18077;
	tpms_fl.PROM[5] = 26770;
	tpms_fl.PROM[6] = 26528;

	/* TPMS Rear Right */
	tpms_rr.id = 2;
	tpms_rr.PROM[0] = 1;
	tpms_rr.PROM[1] = 29553;
	tpms_rr.PROM[2] = 28976;
	tpms_rr.PROM[3] = 17686;
	tpms_rr.PROM[4] = 18245;
	tpms_rr.PROM[5] = 27727;
	tpms_rr.PROM[6] = 26979;

	/* TPMS Rear Left */
	tpms_rl.id = 3;
	tpms_rl.PROM[0] = 1;
	tpms_rl.PROM[1] = 28725;
	tpms_rl.PROM[2] = 28333;
	tpms_rl.PROM[3] = 16814;
	tpms_rl.PROM[4] = 17470;
	tpms_rl.PROM[5] = 27433;
	tpms_rl.PROM[6] = 26274;
}


void init_packet_types(void)
{
	tpms_packet.PacketType = PACKET_TYPE_GFSK;
	tpms_packet.Params.Gfsk.PreambleLength = 16;
	tpms_packet.Params.Gfsk.PreambleMinDetect = PREAMBLE_DETECTOR_08_BITS;
	tpms_packet.Params.Gfsk.SyncWordLength = 0x18;
	tpms_packet.Params.Gfsk.AddrComp = ADDRESSCOMP_FILT_OFF;
	tpms_packet.Params.Gfsk.HeaderType = GFSK_PACKET_FIXED_LENGTH;
	tpms_packet.Params.Gfsk.PayloadLength = TPMS_MSG_BYTES + 1;  // Payload plus CRC
	tpms_packet.Params.Gfsk.CrcType = GFSK_CRC_1_BYTES;
	tpms_packet.Params.Gfsk.DcFree = WHITENING_ON;

	tele_packet.PacketType = PACKET_TYPE_GFSK;
	tele_packet.Params.Gfsk.PreambleLength = 32;
	tele_packet.Params.Gfsk.PreambleMinDetect = PREAMBLE_DETECTOR_16_BITS;
	tele_packet.Params.Gfsk.SyncWordLength = 0x40;
	tele_packet.Params.Gfsk.AddrComp = ADDRESSCOMP_FILT_OFF;
	tele_packet.Params.Gfsk.HeaderType = GFSK_PACKET_FIXED_LENGTH;
	tele_packet.Params.Gfsk.PayloadLength = 50 + 2;  // Payload plus CRC
	tele_packet.Params.Gfsk.CrcType = GFSK_CRC_2_BYTES;
	tele_packet.Params.Gfsk.DcFree = WHITENING_ON;
}


void SX1262CreateTpmsRequestMsg(Radio* radio, uint16_t delay_ms, uint8_t tx_freq, int8_t sleep_time, int8_t wait_time)
{
	radio->tpms_tx_msg[0] = (REQUEST_MSG << 4) | tx_freq;
	radio->tpms_tx_msg[1] = delay_ms >> 8;
	radio->tpms_tx_msg[2] = delay_ms;
	radio->tpms_tx_msg[3] = sleep_time;
	radio->tpms_tx_msg[4] = wait_time;
}

void SX1262CreateTpmsCfgrMsg(Radio* radio,  int8_t power, uint16_t freqInMHz)
{
	radio->tpms_tx_msg[0] = (CFGR_MSG << 4) & 0xF0;
	radio->tpms_tx_msg[1] = power;
	radio->tpms_tx_msg[2] = freqInMHz >> 8;
	radio->tpms_tx_msg[3] = freqInMHz;
	radio->tpms_tx_msg[4] = 0x00;
}

RadioState_t SX1262SetFunction(MCU* mcu, Radio* radio, Function_t function)
{
	if ((function == TPMS) && (radio->function != TPMS))
	{
		SX1262SetPacketParams(mcu, radio, &tpms_packet);
		radio->function = TPMS;
		return SX1262SetGfskSyncWord(mcu, radio, TpmsGfskSyncWord, sizeof(TpmsGfskSyncWord));
	}

	else if ((function == TELEMETRY) && (radio->function != TELEMETRY))
	{
		SX1262SetPacketParams(mcu, radio, &tele_packet);
		radio->function = TELEMETRY;
		return SX1262SetGfskSyncWord(mcu, radio, TeleGfskSyncWord, sizeof(TeleGfskSyncWord));
	}

	return RADIO_OK;
}


void TpmsGetMessage(Radio* radio, TelemetryData_t* teleData)
{
	uint8_t received_tpms_id = radio->tpms_rx_msg[0] & 0b11;

	TPMS_data_t* tpmsSensor;

	/* Chooses which TPMS the data are referring to */
	switch (received_tpms_id)
	{
	case 0:
		tpmsSensor = &tpms_fr;
		break;
	case 1:
		tpmsSensor = &tpms_fl;
		break;
	case 2:
		tpmsSensor = &tpms_rr;
		break;
	case 3:
		tpmsSensor = &tpms_rl;
		break;
	default:
		break;
	}


	if (radio->tpmsWrongCRC == true)
	{
		tpmsSensor->wrongCRC++;
		radio->tpmsWrongCRC = false;
	}

	tpmsSensor->rssi = radio->TpmsRssiAvg;
	tpmsSensor->received_msgs++;
	tpmsSensor->time_variation = radio->tpms_variation;
	tpmsSensor->not_received--;

	tpmsSensor->temperature_reg = 0xFFFF00 & ((radio->tpms_rx_msg[1] << 16) | (radio->tpms_rx_msg[2] << 8));
	tpmsSensor->pressure_reg    = 0xFFFF00 & ((radio->tpms_rx_msg[3] << 16) | (radio->tpms_rx_msg[4] << 8));


	/* Temperature Calibration */
	int32_t dT   = tpmsSensor->temperature_reg - ((uint32_t)(tpmsSensor->PROM[5]) << 8);
	int32_t temp = 2000 + ((int64_t)dT * tpmsSensor->PROM[6] >> 23);

	/* Pressure Calibration */
	int64_t off  = ((int64_t)(tpmsSensor->PROM[2]) << 16) + (((int64_t)(tpmsSensor->PROM[4]) * dT) >> 7);
	int64_t sens = ((int64_t)(tpmsSensor->PROM[1]) << 15) + (((int64_t)(tpmsSensor->PROM[3]) * dT) >> 8);

	/* Pressure without second order temperature compensation */
	// int32_t p = ((pressure_reg * sens >> 21) - off) >> 13;

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
	tpmsSensor->temperature = (temp - ti) * 0.01;
	tpmsSensor->pressure    = ((((tpmsSensor->pressure_reg * (sens - sensi))/ 2097152) - (off - offi)) / 8192) * 0.1;

	/* Convert pressure from mBar to PSI and subtract atmospheric pressure */
	tpmsSensor->pressure = (tpmsSensor->pressure - 1013.25) * 0.0145037;

	switch (received_tpms_id)
	{
	case 0:
		teleData->tpms.fr_press = (uint16_t)(tpmsSensor->pressure    * 100);
		teleData->tpms.fr_temp  = (uint16_t)(tpmsSensor->temperature * 100);  /* Gives a resolution of 0.01 deg C */
		break;
	case 1:
		teleData->tpms.fl_press = (uint16_t)(tpmsSensor->pressure    * 100);
		teleData->tpms.fl_temp  = (uint16_t)(tpmsSensor->temperature * 100);
		break;
	case 2:
		teleData->tpms.rr_press = (uint16_t)(tpmsSensor->pressure	 * 100);
		teleData->tpms.rr_temp  = (uint16_t)(tpmsSensor->temperature * 100);
		break;
	case 3:
		teleData->tpms.rl_press = (uint16_t)(tpmsSensor->pressure	 * 100);
		teleData->tpms.rl_temp  = (uint16_t)(tpmsSensor->temperature * 100);
		break;
	default:
		break;
	}
}



