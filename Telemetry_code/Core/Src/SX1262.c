#include "SX1262.h"



/*****************************************************************************
*  @Description	  Sets the Radio chip to Standby Mode. This is a mode where
*                 it awaits for further commands
******************************************************************************/
void SX1262SetStandby(MCU* mcu, Radio* radio, StandbyMode_t standby_mode)
{
	SX1262WriteCommand(mcu, radio, SET_STANDBY, &standby_mode, 4);

#if (FUNCTION == TPMS_RELEASE)
	radio->opMode = standby_mode;
#else
	SX1262GetStatus(mcu, radio);
#endif
}


/*****************************************************************************
*  @Description	  Sets the protocol used for the wireless communication
******************************************************************************/
void SX1262SetPacketType(MCU *mcu, Radio* radio, PacketType_t packet_type)
{
	SX1262WriteCommand(mcu, radio, SET_PACKET_TYPE, &packet_type, 1);
}


/*****************************************************************************
*  @Description	  Sets the wireless communication frequency (430 - 928 MHz)
******************************************************************************/
void SX1262SetRfFrequency(MCU *mcu, Radio* radio, uint32_t rf_frequency)
{
	uint8_t tx_msg[4];
	uint32_t rf_freq;


	SX1262CalibrateImage(mcu, radio, rf_frequency);

	rf_freq = (uint32_t)((double)rf_frequency * FREQ_DIV / XTAL_FREQ); // can change with shifting <<25
	tx_msg[0] = rf_freq >> 24;
	tx_msg[1] = rf_freq >> 16;
	tx_msg[2] = rf_freq >> 8;
	tx_msg[3] = rf_freq;

	SX1262WriteCommand(mcu, radio, SET_RF_FREQUENCY, tx_msg, 4);

	radio->rfFrequency = rf_frequency;
}


/*****************************************************************************
*  @Description	  Sets vital parameters of the Power Amplifier
******************************************************************************/
/* ----------------- Optimal Settings ----------------- *
 * Output power        paDutyCycle        hpMax			*
 * 														*
 * +22 dBm				  0x04			  0x07			*
 * +20 dBm				  0x03			  0x05			*
 * +17 dBm				  0x02			  0x03			*
 * +14 dBm				  0x02			  0x02			*
 */
void SX1262SetPaConfig(MCU* mcu, Radio* radio, int8_t power)
{
//	uint8_t tx_msg[4];
//
//	if (hp_max > 0x07)
//		hp_max = 0x07;
//	if (pa_duty_cycle > 0x04)
//		pa_duty_cycle = 0x04;
//
//	tx_msg[0] = pa_duty_cycle;
//	tx_msg[1] = hp_max;
//	tx_msg[2] = 0x00;
//	tx_msg[3] = 0x01;
//	SX1262WriteCommand(mcu, radio, SET_PA_CONFIG, tx_msg, 4);
//
//	radio->paDutyCycle = pa_duty_cycle;
//	radio->hpMax = hp_max;

	uint8_t pa_duty_cycle;
	uint8_t hp_max;
	uint8_t tx_msg[4];

	if (power <= 14)
	{
		pa_duty_cycle = 0x02;
		hp_max = 0x02;
	}
	else if (power <= 17)
	{
		pa_duty_cycle = 0x02;
		hp_max = 0x03;
	}
	else if (power <= 20)
	{
		pa_duty_cycle = 0x03;
		hp_max = 0x05;
	}
	else
	{
		pa_duty_cycle = 0x04;
		hp_max = 0x07;
	}

	tx_msg[0] = pa_duty_cycle;
	tx_msg[1] = hp_max;
	tx_msg[2] = 0x00;
	tx_msg[3] = 0x01;
	SX1262WriteCommand(mcu, radio, SET_PA_CONFIG, tx_msg, 4);

	radio->paDutyCycle = pa_duty_cycle;
	radio->hpMax = hp_max;
}


/******************************************************************************
*  @Description	  Sets the transmitted signal power in dBm and the PA ramp time
*******************************************************************************/
void SX1262SetTxParams(MCU* mcu, Radio* radio, int8_t power, RampTime_t ramp_time, bool workaround_on)
{
	 uint8_t tx_msg[2];
	 uint8_t reg_data[1];

	 if (workaround_on) {
		 // WORKAROUND BEGIN - Protection of the chip against antenna mismatch - datasheet chapter 15.2
		 SX1262ReadRegister(mcu, radio, TX_CLAMP_CONFIG_REG, reg_data, 1);
		 reg_data[0] = reg_data[0] | 0x1E;
		 SX1262WriteRegister(mcu, radio, TX_CLAMP_CONFIG_REG, reg_data, 1);
	 	 // WORKAROUND END
		 radio->workaround_on = true;
	 }

	 if (power > 22)
		 power = 22;
	 else if (power < -9)
		 power = -9;

	 tx_msg[0] = power;
	 tx_msg[1] = (uint8_t) ramp_time;
	 SX1262WriteCommand(mcu, radio, SET_TX_PARAMS, tx_msg, 2);

	 radio->power = power;
	 radio->rampTime = ramp_time;
}


/*****************************************************************************
*  @Description	  Sets the base address (offset from 0 to 255) for the
*                 transmitted / received payload. Data buffer has 256 Bytes
******************************************************************************/
void SX1262SetBufferBaseAddress(MCU* mcu, Radio* radio, uint8_t tx_base_address, uint8_t rx_base_address)
{
    uint8_t tx_msg[2];

    tx_msg[0] = tx_base_address;
    tx_msg[1] = rx_base_address;
    SX1262WriteCommand(mcu, radio, SET_BUFFER_BASE_ADDRESS, tx_msg, 2);

    radio->txBaseAddress = tx_base_address;
    radio->rxBaseAddress = rx_base_address;
}


/*****************************************************************************
*  @Description	  Sets the packet parameters for the protocol selected before.
*                 If the parameters correspond to a different packet
*                 type, it changes the packet type and then gives the params.
******************************************************************************/
void SX1262SetPacketParams(MCU *mcu, Radio *radio, PacketParams_t *packetParams) {
	uint8_t tx_msg_len;
	uint8_t crcType = 0;
	uint8_t tx_msg[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	/* Check if required configuration corresponds to the stored packet type
	   If not, silently update radio packet type     */
	if (radio->packetType != packetParams->PacketType) {
		SX1262SetPacketType(mcu, radio, packetParams->PacketType);
		radio->packetType = packetParams->PacketType;
	}
	switch (packetParams->PacketType)
	{
	case PACKET_TYPE_GFSK:
		crcType = packetParams->Params.Gfsk.CrcType;
		if (packetParams->Params.Gfsk.SyncWordLength > 0x40)
			packetParams->Params.Gfsk.SyncWordLength = 0x40;

		tx_msg_len = 9;
		tx_msg[0] = packetParams->Params.Gfsk.PreambleLength >> 8;
		tx_msg[1] = packetParams->Params.Gfsk.PreambleLength;
		tx_msg[2] = packetParams->Params.Gfsk.PreambleMinDetect;
		tx_msg[3] = packetParams->Params.Gfsk.SyncWordLength;
		tx_msg[4] = packetParams->Params.Gfsk.AddrComp;
		tx_msg[5] = packetParams->Params.Gfsk.HeaderType;
		tx_msg[6] = packetParams->Params.Gfsk.PayloadLength;
		tx_msg[7] = crcType;
		tx_msg[8] = packetParams->Params.Gfsk.DcFree;
		radio->packetParams.Params.Gfsk.PreambleLength = packetParams->Params.Gfsk.PreambleLength;
		radio->packetParams.Params.Gfsk.PreambleMinDetect = packetParams->Params.Gfsk.PreambleMinDetect;
		radio->packetParams.Params.Gfsk.SyncWordLength = packetParams->Params.Gfsk.SyncWordLength;
		radio->packetParams.Params.Gfsk.AddrComp = packetParams->Params.Gfsk.AddrComp;
		radio->packetParams.Params.Gfsk.HeaderType = packetParams->Params.Gfsk.HeaderType;
		radio->packetParams.Params.Gfsk.PayloadLength = packetParams->Params.Gfsk.PayloadLength;
		radio->packetParams.Params.Gfsk.CrcType = crcType;
		radio->packetParams.Params.Gfsk.DcFree = packetParams->Params.Gfsk.DcFree;
		break;

	case PACKET_TYPE_LORA:
		tx_msg_len = 6;
		tx_msg[0] = packetParams->Params.LoRa.PreambleLength >> 8;
		tx_msg[1] = packetParams->Params.LoRa.PreambleLength;
		tx_msg[2] = packetParams->Params.LoRa.HeaderType;
		tx_msg[3] = packetParams->Params.LoRa.PayloadLength;
		tx_msg[4] = packetParams->Params.LoRa.CrcMode;
		tx_msg[5] = packetParams->Params.LoRa.InvertIQ;
		radio->packetParams.Params.LoRa.PreambleLength = packetParams->Params.LoRa.PreambleLength;
		radio->packetParams.Params.LoRa.HeaderType = packetParams->Params.LoRa.HeaderType;
		radio->packetParams.Params.LoRa.PayloadLength = packetParams->Params.LoRa.PayloadLength;
		radio->packetParams.Params.LoRa.CrcMode = packetParams->Params.LoRa.CrcMode;
		radio->packetParams.Params.LoRa.InvertIQ = packetParams->Params.LoRa.InvertIQ;
		break;

	case PACKET_TYPE_LR_FHSS:
		// Not implemented
		break;
	}

	SX1262WriteCommand(mcu, radio, SET_PACKET_PARAMS, tx_msg, tx_msg_len);
}


/*****************************************************************************
*  @Description	  Sets the modulation parameters for the protocol selected
*                 before. If the parameters correspond to a different packet
*                 type, it changes the packet type and then gives the params.
******************************************************************************/
void SX1262SetModulationParams(MCU* mcu, Radio* radio, ModulationParams_t *modulationParams)
{
    uint8_t tx_msg_len;
    uint32_t bitrate;
    uint8_t tx_msg[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if(radio->packetType != modulationParams->PacketType)
    {
       SX1262SetPacketType(mcu, radio, modulationParams->PacketType);
       radio->packetType = modulationParams->PacketType;
    }


    switch(modulationParams->PacketType)
    {
    case PACKET_TYPE_GFSK:

        tx_msg_len = 8;
        bitrate = (uint32_t)(32 * XTAL_FREQ / modulationParams->Params.Gfsk.BitRate);
        tx_msg[0] = bitrate >> 16;
        tx_msg[1] = bitrate >> 8;
        tx_msg[2] = bitrate;
        tx_msg[3] = modulationParams->Params.Gfsk.ModulationShaping;
        tx_msg[4] = modulationParams->Params.Gfsk.Bandwidth;
        uint32_t fdev = (uint32_t) ((float)(modulationParams->Params.Gfsk.FdevHz) * FREQ_DIV / XTAL_FREQ);
        tx_msg[5] = fdev >> 16;
        tx_msg[6] = fdev >> 8;
        tx_msg[7] = fdev;

        radio->modulationParams.Params.Gfsk.BitRate = modulationParams->Params.Gfsk.BitRate;
        radio->modulationParams.Params.Gfsk.ModulationShaping = modulationParams->Params.Gfsk.ModulationShaping;
        radio->modulationParams.Params.Gfsk.Bandwidth = modulationParams->Params.Gfsk.Bandwidth;
        radio->modulationParams.Params.Gfsk.FdevHz = modulationParams->Params.Gfsk.FdevHz;
        break;

    case PACKET_TYPE_LORA:
        tx_msg_len = 4;
        tx_msg[0] = modulationParams->Params.LoRa.SpreadingFactor;
        tx_msg[1] = modulationParams->Params.LoRa.Bandwidth;
        tx_msg[2] = modulationParams->Params.LoRa.CodingRate;
        tx_msg[3] = modulationParams->Params.LoRa.LowDatarateOptimize;

        radio->modulationParams.Params.LoRa.SpreadingFactor = modulationParams->Params.LoRa.SpreadingFactor;
        radio->modulationParams.Params.LoRa.Bandwidth = modulationParams->Params.LoRa.Bandwidth;
        radio->modulationParams.Params.LoRa.CodingRate = modulationParams->Params.LoRa.CodingRate;
        radio->modulationParams.Params.LoRa.LowDatarateOptimize = modulationParams->Params.LoRa.LowDatarateOptimize;
        break;
    case PACKET_TYPE_LR_FHSS:
    	/* NOT IMPLEMENTED */
        return;
    }

    SX1262WriteCommand(mcu, radio, SET_MODULATION_PARAMS, tx_msg, tx_msg_len);
}


/*****************************************************************************
*  @Description	  Maps the interrupts. irqMask dictates which interurpts will
*                 be created and stored in the device registers. dio_Mask
*                 determines which of these interrupts will create an interrupt
*                 on this specific DIO (set pin).
******************************************************************************/
void SX1262SetDioIrqParams(MCU* mcu, Radio* radio, Irq_t irqMask, Irq_t dio1Mask, Irq_t dio2Mask, Irq_t dio3Mask)
{
    uint8_t tx_msg[8];

    tx_msg[0] = irqMask  >> 8;
    tx_msg[1] = irqMask;
    tx_msg[2] = dio1Mask >> 8;
    tx_msg[3] = dio1Mask;
    tx_msg[4] = dio2Mask >> 8;
    tx_msg[5] = dio2Mask;
    tx_msg[6] = dio3Mask >> 8;
    tx_msg[7] = dio3Mask;

    SX1262WriteCommand(mcu, radio, SET_DIO_IRQ_PARAMS, tx_msg, 8);

    radio->irqMask  = irqMask;
    radio->dio1Mask = dio1Mask;
    radio->dio2Mask = dio2Mask;
    radio->dio3Mask = dio3Mask;
}


/*****************************************************************************
*  @Description	  Sets the syncWord by accessing the register in GFSK
******************************************************************************/
RadioState_t SX1262SetGfskSyncWord(MCU* mcu, Radio* radio, uint8_t *GfskSyncWord, uint8_t len)
{
	return SX1262SafeWriteRegister(mcu, radio, SYNC_WORD_0_REG, GfskSyncWord, len);
}


/***************************************************************************************
*  @Description	  Sets the syncWord by accessing the register in LoRa
*                 LORA_MAC_PRIVATE_SYNCWORD  0x1424   Syncword for Private LoRa networks
*                 LORA_MAC_PUBLIC_SYNCWORD   0x3444   Syncword for Public LoRa networks
****************************************************************************************/
void SX1262SetLoRaSyncWord(MCU* mcu, Radio* radio, uint8_t *LoRasyncWord)
{
	SX1262WriteRegister(mcu, radio, LORA_SYNC_WORD_0_REG, LoRasyncWord, 2);
}



/**********************************************************************************
*  @Description	  Sets the chip in TX mode. The transceiver will transmit
*                 whatever is present in the Buffer starting from the txBufBaseAddr
*                 until the TxLength has been completed. After the Tx it will get
*                 information of the status and return to fallback mode
***********************************************************************************/
void SX1262SetTx(MCU* mcu, Radio* radio, uint16_t timeout_duration /* [msec] */, bool wait)
{

#if ((FUNCTION != TPMS_DEBUG) && (FUNCTION != TPMS_RELEASE))
	uint8_t reg_data;
	if ((radio->packetType == PACKET_TYPE_LORA) && (radio->modulationParams.Params.LoRa.Bandwidth == LORA_BW_500))
	{
		SX1262ReadRegister(mcu, radio, TX_MODULATION_REG, &reg_data, 1);
		reg_data = reg_data & 0xFB;
		SX1262SafeWriteRegister(mcu, radio, TX_MODULATION_REG, &reg_data, 1);
	}
	else
	{
		SX1262ReadRegister(mcu, radio, TX_MODULATION_REG, &reg_data, 1);
		reg_data = reg_data | 0x04;
		SX1262SafeWriteRegister(mcu, radio, TX_MODULATION_REG, &reg_data, 1);
	}
#endif



    uint8_t tx_msg[3];

    uint32_t timeout = timeout_duration * 1000 / 15.625;

    if (timeout > 0xFFFFFF)
    	timeout = 0xFFFFFF;

    tx_msg[0] = timeout >> 16;
    tx_msg[1] = timeout >> 8;
    tx_msg[2] = timeout;

    // function returns when Tx has started
    if (!wait)
    {
    	SX1262WriteCommand(mcu, radio, SET_TX, tx_msg, 3);
#if (FUNCTION != TPMS_RELEASE)
    	SX1262GetStatus(mcu, radio);
    	SX1262GetIrqStatus(mcu, radio);

    	if (radio->irq.txDone != 1)
    		radio->tx_failures++;
#endif
    	SX1262ClearIrqStatus(mcu, radio, 0xFFFF);
    }

    // function returns when Tx has been completed
    else
    {
    	SX1262ClearIrqStatus(mcu, radio, 0xFFFF);
    	SX1262WriteCommand(mcu, radio, SET_TX, tx_msg, 3);
    	SX1262GetStatus(mcu, radio);
    	SX1262GetIrqStatus(mcu, radio);

    	uint32_t current_tick = HAL_GetTick();
    	while(radio->irq.txDone != 1)
    	{
        	SX1262GetIrqStatus(mcu, radio);
        	if (HAL_GetTick() - current_tick > timeout_duration)
        	{
        		radio->tx_failures++;
        		break;
        	}
    	}
    }
}


/*****************************************************************************
*  @Description	  Sets the chip to sleep mode, minimuzing its current consumption
******************************************************************************/
void SX1262SetSleep(MCU* mcu, Radio* radio, SleepParams_t sleepConfig )
{
    uint8_t value = (sleepConfig.startMode << 2) | sleepConfig.rtcMode;

    SX1262WriteCommand(mcu, radio, SET_SLEEP, &value, 1);
    radio->opMode = SLEEP_MODE;
}


/*****************************************************************************
*  @Description	  Sets the chip to Frequency Synthesizer mode. Useful only for
*                 debugging purposes as otherwise it is handled automatically
******************************************************************************/
void SX1262SetFs(MCU* mcu, Radio* radio)
{
    SX1262WriteCommand(mcu, radio, SET_FS, 0, 0);

#if (FUNCTION == TPMS_RELEASE)
    radio->opMode = FS_MODE;
#else
    SX1262GetStatus(mcu, radio);
#endif
}


/*****************************************************************************
*  @Description	  Sets the chip to Rx mode with a given timeout. If the timeout
*                 is over or a message is detected, it gets to the fallbac mode,
*                 except if the timeout is 0xFFFF (continuous RX mode)
******************************************************************************/
RadioState_t SX1262SetRx(MCU* mcu, Radio* radio, uint16_t timeout_duration /* [ms] */)
{
	if ((radio->opMode != RX_MODE) && (radio->opMode != RX_DC_MODE))
		radio->msg_pending = false;

    uint8_t tx_msg[3];

    uint32_t timeout = timeout_duration * 1000 / 15.625;
    if ((timeout_duration == 0xFFFF) || (timeout > 0xFFFFFF))
    	timeout = 0xFFFFFF;  // Rx Continuous Mode

#if ((FUNCTION != TPMS_DEBUG) && (FUNCTION != TPMS_RELEASE))
    uint8_t reg_data = 0x94;
    if(SX1262SafeWriteRegister(mcu, radio, RX_GAIN_REG, &reg_data, 1) != RADIO_OK) // Rx Power saving gain
    {
    	return RADIO_SAFE_WRITE_ERROR;
    }
#endif

    tx_msg[0] = timeout >> 16;
    tx_msg[1] = timeout >> 8;
    tx_msg[2] = timeout;
    SX1262WriteCommand(mcu, radio, SET_RX, tx_msg, 3);

#if ((FUNCTION != TPMS_DEBUG) && (FUNCTION != TPMS_RELEASE))
    SX1262GetStatus(mcu, radio);
#else
    radio->opMode = RX_MODE;
#endif

    return RADIO_OK;
}


/*****************************************************************************
*  @Description	  Sets the chip to Rx mode and writes to a register to increase
*                 sensitivity by ~3dB
******************************************************************************/
RadioState_t SX1262SetRxBoosted(MCU* mcu, Radio* radio, uint32_t timeout_duration)
{
	if ((radio->opMode != RX_MODE) && (radio->opMode != RX_DC_MODE))
		radio->msg_pending = false;

	uint8_t tx_msg[3];

	uint32_t timeout = timeout_duration * 1000 / 15.625;
	if ((timeout_duration == 0xFFFF) || (timeout > 0xFFFFFF))
    	timeout = 0xFFFFFF;  // Rx Continuous Mode

    uint8_t reg_data = 0x96;
    if (SX1262SafeWriteRegister(mcu, radio, RX_GAIN_REG, &reg_data, 1) != RADIO_OK) // max LNA gain, increase current by ~2mA for around ~3dB in sensitivity
    {
    	return RADIO_SAFE_WRITE_ERROR;
    }

    tx_msg[0] = timeout >> 16;
    tx_msg[1] = timeout >> 8;
    tx_msg[2] = timeout;
    SX1262WriteCommand(mcu, radio, SET_RX, tx_msg, 3);

    SX1262GetStatus(mcu, radio);

    return RADIO_OK;
}


/*****************************************************************************
*  @Description	  Clears all the generated interrupts from the chip and from
*                 the radio struct
******************************************************************************/
void SX1262ClearIrqStatus(MCU* mcu, Radio* radio, uint16_t irq)
{
    uint8_t tx_msg[2];

    tx_msg[0] = irq >> 8;
    tx_msg[1] = irq;
    SX1262WriteCommand(mcu, radio, CLEAR_IRQ_STATUS, tx_msg, 2);

    radio->irq_status = 0x0000;

	radio->irq.txDone 			= 0;
	radio->irq.rxDone			= 0;
	radio->irq.preambleDetected = 0;
	radio->irq.syncWordValid 	= 0;
	radio->irq.headerValid		= 0;
	radio->irq.headerError 		= 0;
	radio->irq.crcError 		= 0;
	radio->irq.cadDone 			= 0;
	radio->irq.cadDetected 		= 0;
	radio->irq.timeout 			= 0;
}


/*****************************************************************************
*  @Description	  Returns all the interrupts generated by the chip and updates
*                 the radio struct
******************************************************************************/
void SX1262GetIrqStatus(MCU* mcu, Radio* radio)
{
    SX1262ReadCommand(mcu, radio, GET_IRQ_STATUS, 2);

    uint16_t irq_status = (radio->spi_read_msg[2] << 8) | radio->spi_read_msg[3];

    /* Updates the struct irq */
    if ((irq_status & IRQ_TX_DONE) == IRQ_TX_DONE)
    	radio->irq.txDone = 1;
    else
    	radio->irq.txDone = 0;

    if ((irq_status & IRQ_RX_DONE) == IRQ_RX_DONE)
    	radio->irq.rxDone = 1;
    else
    	radio->irq.rxDone = 0;

    if ((irq_status & IRQ_PREAMBLE_DETECTED) == IRQ_PREAMBLE_DETECTED)
    	radio->irq.preambleDetected = 1;
    else
    	radio->irq.preambleDetected = 0;

    if ((irq_status & IRQ_SYNC_WORD_VALID) == IRQ_SYNC_WORD_VALID)
    	radio->irq.syncWordValid = 1;
    else
    	radio->irq.syncWordValid = 0;

    if ((irq_status & IRQ_HEADER_VALID) == IRQ_HEADER_VALID)
    	radio->irq.headerValid = 1;
    else
    	radio->irq.headerValid = 0;

    if ((irq_status & IRQ_HEADER_ERROR) == IRQ_HEADER_ERROR)
    	radio->irq.headerError = 1;
    else
    	radio->irq.headerError = 0;

    if ((irq_status & IRQ_CRC_ERROR) == IRQ_CRC_ERROR)
    	radio->irq.crcError = 1;
    else
    	radio->irq.crcError = 0;

    if ((irq_status & IRQ_CAD_DONE) == IRQ_CAD_DONE)
    	radio->irq.cadDone = 1;
    else
    	radio->irq.cadDone = 0;

    if ((irq_status & IRQ_CAD_DETECTED) == IRQ_CAD_DETECTED)
    	radio->irq.cadDetected = 1;
    else
    	radio->irq.cadDetected = 0;

    if ((irq_status & IRQ_TIMEOUT) == IRQ_TIMEOUT)
    	radio->irq.timeout = 1;
    else
    	radio->irq.timeout = 0;
}


/*****************************************************************************
*  @Description	  Gets the position of the first byte of the received message
*                 as well as its length in the Buffer
******************************************************************************/
void SX1262GetRxBufferStatus(MCU* mcu, Radio* radio)
{
    SX1262ReadCommand(mcu, radio, GET_RX_BUFFER_STATUS, 2);

    radio->payloadLength = radio->spi_read_msg[2];
    radio->rxStartBufferPointer = radio->spi_read_msg[3];
}


/*****************************************************************************
*  @Description	  Begins the calibration of the parts declared in calibParams
******************************************************************************/
void SX1262Calibrate(MCU* mcu, Radio* radio, CalibrationParams_t calibParam)
{
    uint8_t value = (calibParam.Img << 6) 	   |
    				(calibParam.ADCBulkP << 5) |
					(calibParam.ADCBulkN << 4) |
    		        (calibParam.ADCPulse << 3) |
					(calibParam.PLL << 2) 	   |
					(calibParam.RC13M << 1)    |
					(calibParam.RC64K);

    SX1262WriteCommand(mcu, radio, CALIBRATE, &value, 1);
}


/*****************************************************************************
*  @Description	  Enables / Disables the DIO2 pin to control the RF switch.
*                 If it is enabled, it can generate no other interrupts even
*                 if they are mapped on it
******************************************************************************/
void SX1262SetDio2AsRfSwitchCtrl(MCU* mcu, Radio* radio, bool enable)
{
	uint8_t enable_switch = 0;
	if (enable)
		enable_switch = 1;

    SX1262WriteCommand(mcu, radio, SET_RF_SWITCH_MODE, &enable_switch, 1);
}


/*****************************************************************************
*  @Description	  Enables / Disables the DIO3 pin to supply the TCXO.
*                 If it is enabled, it can generate no interrupts even
*                 if they are mapped on it.
*                 IMAGE CALIBRATION SHOULD BE CALLED AFTER THIS COMMAND
******************************************************************************/
void SX1262SetDio3AsTcxoCtrl(MCU* mcu, Radio* radio, TcxoCtrlVoltage_t tcxoVoltage, uint8_t delay_duration)
{
	if (radio->opMode != STDBY_RC_MODE)
		SX1262SetStandby(mcu, radio, STDBY_RC);

	uint8_t tx_msg[4];

	uint32_t delay = delay_duration << 6;

    if (tcxoVoltage > TCXO_MAX_VOLTAGE) {
    	tcxoVoltage = TCXO_MAX_VOLTAGE;   // Voltage limit for protection of the TCXO
    }

    tx_msg[0] = tcxoVoltage;
    tx_msg[1] = delay >> 16;
    tx_msg[2] = delay >> 8;
    tx_msg[3] = delay;

    SX1262WriteCommand(mcu, radio, SET_TCXO_MODE, tx_msg, 4);
    radio->tcxoVolt = tcxoVoltage;
}


/*****************************************************************************
*  @Description	  It calibrates the frequency band in which the chip is
*                 about to transmit / receive
******************************************************************************/
void SX1262CalibrateImage(MCU* mcu, Radio* radio, uint32_t freq)
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    SX1262WriteCommand(mcu, radio, CALIBRATE_IMAGE, calFreq, 2);
    radio->ImageCalibrated = true;
}


/*****************************************************************************
*  @Description	  Defines whether the timer in RX Duty Cycle mode will be
*                 stopped upon preamble detection
******************************************************************************/
void SX1262StopTimerOnPreamble(MCU* mcu, Radio* radio, bool enable)
{
	SX1262WriteCommand(mcu, radio, STOP_TIMER_ON_PREAMBLE, (uint8_t*)&enable, 1);
}


/*****************************************************************************
*  @Description	  Sets the chip in periodic transition between Rx mode and
*                 sleep mode
******************************************************************************/
void SX1262SetRxDutyCycle(MCU* mcu, Radio* radio, uint16_t rx_period, uint16_t sleep_period /* [ms] */)
{
	uint32_t rxPeriod = rx_period << 6;
	uint32_t sleepPeriod = sleep_period << 6;

	uint8_t tx_msg[6];

	tx_msg[0] = rxPeriod >> 16;
	tx_msg[1] = rxPeriod >> 8;
	tx_msg[2] = rxPeriod;
	tx_msg[3] = sleepPeriod >> 16;
	tx_msg[4] = sleepPeriod >> 8;
	tx_msg[5] = sleepPeriod;

	SX1262WriteCommand(mcu, radio, SET_RX_DUTY_CYCLE, tx_msg, 6 );

	radio->opMode = RX_DC_MODE;
}


/*****************************************************************************
*  @Description	  Sets the chip to Channel Activity Detection mode
******************************************************************************/
void SX1262SetCad(MCU* mcu, Radio* radio)
{
	SX1262WriteCommand(mcu, radio, SET_CAD, 0, 0);
	radio->opMode = CAD_MODE;
}


/*****************************************************************************
*  @Description	  Transmits a continuous sinusoidal wave of specific frequency.
*                 Useful when testing the transmitted signal power on a
*                 spectrum analyzer
******************************************************************************/
void SX1262SetTxContinuousWave(MCU* mcu, Radio* radio)
{
	SX1262WriteCommand(mcu, radio, SET_TX_CONTINUOUS_WAVE, 0, 0);

#if (FUNCTION == TPMS_RELEASE)
	radio->opMode = TX_MODE;
#else
	SX1262GetStatus(mcu, radio);
#endif
}


/*****************************************************************************
*  @Description	  Sends infinitely a preamble for debugging purposes
******************************************************************************/
void SX1262SetTxInfinitePreamble(MCU* mcu, Radio* radio)
{
	SX1262WriteCommand(mcu, radio, SET_TX_INFINITE_PREAMBLE, 0, 0);

#if (FUNCTION == TPMS_RELEASE)
	radio->opMode = TX_MODE;
#else
	SX1262GetStatus(mcu, radio);
#endif
}


/*****************************************************************************
*  @Description	  Used to choose between internal LDO and internal DC-DC.
*                 When using Dc-DC the consumption can be dropped to a half
******************************************************************************/
void SX1262SetRegulatorMode(MCU* mcu, Radio* radio, RegulatorMode_t mode)
{
    SX1262WriteCommand(mcu, radio, SET_REGULATOR_MODE, &mode, 1);

    radio->regMode = mode;
}


/*****************************************************************************
*  @Description	  Dictates in which mode the chip will get to after a packet
*                 transmission / reception
******************************************************************************/
void SX1262SetRxTxFallbackMode(MCU* mcu, Radio* radio, FallbackMode_t fallback_mode)
{
	SX1262WriteCommand(mcu, radio, SET_RX_TX_FALLBACK_MODE, &fallback_mode, 1);

	radio->fallbackMode = fallback_mode;
}


/*****************************************************************************
*  @Description	  Writes the data to be transmitted to the Data Buffer
******************************************************************************/
void SX1262SetPayload(MCU* mcu, Radio* radio, uint8_t *payload, uint8_t size)
{
    SX1262WriteBuffer(mcu, radio, 0x00, payload, size);
}


/*****************************************************************************
*  @Description	  Writes the data to be transmitted to the Data Buffer and
*                 sets the chip to TX mode, in order to start the transmission
*                 wait boolean defines if the function returns when the TX has
*                 started (false) or when it is completed (true)
******************************************************************************/
void SX1262SendPayload(MCU* mcu, Radio* radio, uint8_t *payload, uint8_t size, uint8_t timeout, bool wait)
{
    SX1262SetPayload(mcu, radio, payload, size);
    SX1262SetTx(mcu, radio, timeout, wait);
}


/*****************************************************************************
*  @Description	  Returns the packet type stored in the chip's registers
******************************************************************************/
void SX1262GetPacketType(MCU* mcu, Radio* radio)
{
	SX1262ReadCommand(mcu, radio, GET_PACKET_TYPE, 1);

	radio->packetType = radio->spi_read_msg[2];
}


/*****************************************************************************
*  @Description	  Sets the Channel Activity Detection parameters
******************************************************************************/
void SX1262SetCadParams(MCU* mcu, Radio* radio, CadParams_t cadParams)
{
	uint8_t tx_msg[7];

	uint32_t cad_timeout = cadParams.cadTimeout * 1000 / 15.625;

	tx_msg[0] = cadParams.cadSymbolNum;
	tx_msg[1] = cadParams.cadDetPeak;
	tx_msg[2] = cadParams.cadDetMin;
	tx_msg[3] = cadParams.cadExitMode;
	tx_msg[4] = cad_timeout >> 16;
	tx_msg[5] = cad_timeout >> 8;
	tx_msg[6] = cad_timeout;

	SX1262WriteCommand(mcu, radio, SET_CAD_PARAMS, tx_msg, 7);

	radio->cadParams.cadSymbolNum = cadParams.cadSymbolNum;
	radio->cadParams.cadDetPeak = cadParams.cadDetPeak;
	radio->cadParams.cadDetMin = cadParams.cadDetMin;
	radio->cadParams.cadExitMode = cadParams.cadExitMode;
	radio->cadParams.cadTimeout = cadParams.cadTimeout;
}


/*****************************************************************************
*  @Description	  Sets the number of symbols used by the modem to validate
*                 a successful reception (lock the transceiver).
******************************************************************************/
void SX1262SetLoRaSymbNumTimeout(MCU* mcu, Radio* radio, uint8_t symbNum)
{
	if (symbNum > SX1262_MAX_LORA_SYMB_NUM_TIMEOUT)
		symbNum = SX1262_MAX_LORA_SYMB_NUM_TIMEOUT;

    SX1262WriteCommand(mcu, radio, SET_LORA_SYMB_NUM_TIMEOUT, &symbNum, 1);
}


/*****************************************************************************
*  @Description	  Returns information about the received signal power and
*                 possible errors (according to the protocol used)
******************************************************************************/
void SX1262GetPacketStatus(MCU* mcu, Radio* radio)
{
    SX1262ReadCommand(mcu, radio, GET_PACKET_STATUS, 3);

    switch(radio->function)
    {
    case TELEMETRY:
    	switch(radio->packetType)
    	{
    	case PACKET_TYPE_GFSK:
    		//radio->packetStatus.Gfsk.RxStatus =  radio->spi_read_msg[2];
    		radio->packetStatus.Gfsk.RssiSync = -radio->spi_read_msg[3] >> 1;
    		radio->packetStatus.Gfsk.RssiAvg  = -radio->spi_read_msg[4] >> 1;
    		break;

    	case PACKET_TYPE_LORA:
    		radio->packetStatus.LoRa.RssiPacket       = -radio->spi_read_msg[2] >> 1;
    		radio->packetStatus.LoRa.SnrPacket        = ((int8_t)radio->spi_read_msg[3]) >> 2;   // Returns SNR value [dB] rounded to the nearest integer value
    		radio->packetStatus.LoRa.SignalRssiPacket = -radio->spi_read_msg[4] >> 1;
    		break;

    	default:
    		break;
    	}
    	break;

    case TPMS:
    	radio->TpmsRssiAvg = -radio->spi_read_msg[4] >> 1;
    	break;
    }
}


/*****************************************************************************
*  @Description	  Returns the RSSI (Received Signal Strength Indicator)
*                 detected that specific moment. It doesn't have to be a valid
*                 packet to return the RSSI value
******************************************************************************/
int8_t SX1262GetRssiInst(MCU* mcu, Radio* radio)
{
    SX1262ReadCommand(mcu, radio, GET_RSSI_INST, 1);
    return -radio->spi_read_msg[2] >> 1;
}


/*****************************************************************************
*  @Description	  Returns statistics of the last packets received
******************************************************************************/
void SX1262GetStats(MCU* mcu, Radio* radio)
{
	SX1262ReadCommand(mcu, radio, GET_STATS, 6);

	switch(radio->packetType)
	{
		case PACKET_TYPE_GFSK:
			radio->rxCounter.Gfsk.PacketReceived = (radio->spi_read_msg[2] << 8) | (radio->spi_read_msg[3]);
			radio->rxCounter.Gfsk.CrcError       = (radio->spi_read_msg[4] << 8) | (radio->spi_read_msg[5]);
			radio->rxCounter.Gfsk.LengthError    = (radio->spi_read_msg[6] << 8) | (radio->spi_read_msg[7]);
			break;

		case PACKET_TYPE_LORA:
			radio->rxCounter.LoRa.PacketReceived = (radio->spi_read_msg[2] << 8) | (radio->spi_read_msg[3]);
			radio->rxCounter.LoRa.CrcError       = (radio->spi_read_msg[4] << 8) | (radio->spi_read_msg[5]);
			radio->rxCounter.LoRa.HeaderError    = (radio->spi_read_msg[6] << 8) | (radio->spi_read_msg[7]);
			break;

		case PACKET_TYPE_LR_FHSS:
			// NOT IMPLEMENTED
			break;
	}
}


/*****************************************************************************
*  @Description	  Resets the previous statistics of the last packets received
******************************************************************************/
void SX1262ResetStats(MCU* mcu, Radio* radio)
{
	uint8_t tx_msg[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	SX1262WriteCommand(mcu, radio, RESET_STATS, tx_msg, 6);

	radio->rxCounter.Gfsk.PacketReceived = 0x0000;
	radio->rxCounter.Gfsk.CrcError       = 0x0000;
	radio->rxCounter.Gfsk.LengthError    = 0x0000;

	radio->rxCounter.LoRa.PacketReceived = 0x0000;
	radio->rxCounter.LoRa.CrcError       = 0x0000;
	radio->rxCounter.LoRa.HeaderError    = 0x0000;
}


/*****************************************************************************
*  @Description	  Returns possible device errors during calibration. The most
*                 common is the XoscStartErr and special care must be given to
*                 the soldering of the TCXO as well as the commands order
******************************************************************************/
void SX1262GetDeviceErrors(MCU* mcu, Radio* radio)
{
    SX1262ReadCommand(mcu, radio, GET_DEVICE_ERRORS, 2);

    radio->errors.PaRampErr     = (radio->spi_read_msg[2] & 0x01);
    radio->errors.PllLockErr    = (radio->spi_read_msg[3] & 0x40) >> 6;
    radio->errors.XoscStartErr  = (radio->spi_read_msg[3] & 0x20) >> 5;
    radio->errors.ImgCalibErr   = (radio->spi_read_msg[3] & 0x10) >> 4;
    radio->errors.AdcCalibErr   = (radio->spi_read_msg[3] & 0x08) >> 3;
    radio->errors.PllCalibErr   = (radio->spi_read_msg[3] & 0x04) >> 2;
    radio->errors.Rc13mCalibErr = (radio->spi_read_msg[3] & 0x02) >> 1;
    radio->errors.Rc64kCalibErr = (radio->spi_read_msg[3] & 0x01);
}


/*****************************************************************************
*  @Description	  Clears the aforementioned calibration errors from the chip's
*                 registers as well as the radio struct
******************************************************************************/
void SX1262ClearDeviceErrors(MCU* mcu, Radio* radio)
{
    uint8_t tx_msg[2] = {0x00, 0x00};
    SX1262WriteCommand(mcu, radio, CLEAR_DEVICE_ERRORS, tx_msg, 2);

    radio->errors.PaRampErr     = 0;
    radio->errors.PllLockErr    = 0;
    radio->errors.XoscStartErr  = 0;
    radio->errors.ImgCalibErr   = 0;
    radio->errors.AdcCalibErr   = 0;
    radio->errors.PllCalibErr   = 0;
    radio->errors.Rc13mCalibErr = 0;
    radio->errors.Rc64kCalibErr = 0;
}


/*****************************************************************************
*  @Description	  Returns the operating mode of the chip
******************************************************************************/
void SX1262GetStatus(MCU* mcu, Radio* radio)
{
	SX1262ReadCommand(mcu, radio, GET_STATUS, 1);

	uint8_t chip_mode = (radio->spi_read_msg[1] >> 4) & 0x07;

	switch (chip_mode)
		{
		case 0x02:
			radio->opMode = STDBY_RC_MODE;
			break;
		case 0x03:
			radio->opMode = STDBY_XOSC_MODE;
			break;
		case 0x04:
			radio->opMode = FS_MODE;
			break;
		case 0x05:
			radio->opMode = RX_MODE;
			break;
		case 0x06:
			radio->opMode = TX_MODE;
			break;
		default:
			radio->opMode = UNDEFINED;
		}

#if ((FUNCTION != TPMS_DEBUG) &&(FUNCTION != TPMS_RELEASE))
	if (radio->opMode == TX_MODE)
		HAL_GPIO_WritePin(mcu->radio_tx_led.type, mcu->radio_tx_led.pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(mcu->radio_tx_led.type, mcu->radio_tx_led.pin, GPIO_PIN_RESET);
#endif
}



/********************************************************************************
*  @Description	  Drives the NRESET pin low, so it erases all registers / buffers
*                 and fully resets the chip, before getting it high again
*********************************************************************************/
void SX1262Reset(MCU* mcu)
{
  HAL_GPIO_WritePin(mcu->nreset.type, mcu->nreset.pin, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(mcu->nreset.type, mcu->nreset.pin, GPIO_PIN_SET);

  while(HAL_GPIO_ReadPin(mcu->busy.type, mcu->busy.pin) != GPIO_PIN_RESET);  // Wait until BUSY pin goes LOW
}


/*****************************************************************************
*  @Description	  Sets maximum draw current in the chip for overcurrent
*                 protection purposes
******************************************************************************/
void SX1262SetOCP(MCU* mcu, Radio* radio, uint16_t value /* [mA] */)
{
  uint8_t ocp_value = 0xFF;//(uint8_t)(1.0 * value / 2.5);

  SX1262WriteRegister(mcu, radio, OCP_CONFIGURATION_REG, &ocp_value, 1);
}


/*****************************************************************************
*  @Description	  Gives supply to the RF switch and through the control pin,
*                 it connects the antenna to the Rx path
*                 Control  LOW : Rx
*                 Control HIGH : Tx
******************************************************************************/
void SX1262RfSwitchInit(MCU* mcu)
{
	HAL_GPIO_WritePin(mcu->rf_switch_supp.type, mcu->rf_switch_supp.pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(mcu->rf_switch.type, mcu->rf_switch.pin, GPIO_PIN_RESET);
}


/*****************************************************************************
*  @Description	  Initializes the radio for the LoRa protocol. The user can
*                 define in the arguments the frequency and the power in dBm
******************************************************************************/
RadioState_t SX1262LoRaInit(MCU* mcu, Radio* radio, int8_t power, uint16_t freqInMHz)
{
	SX1262Reset(mcu);
	SX1262RfSwitchInit(mcu);
	SX1262GetStatus(mcu, radio);

	SX1262SetStandby(mcu, radio, STDBY_RC);
	SX1262SetRegulatorMode(mcu, radio, USE_DCDC);
	SX1262SetDio3AsTcxoCtrl(mcu, radio, TCXO_1_8V, TCXO_SETUP_TIME);

	SX1262SetDio2AsRfSwitchCtrl(mcu, radio, true);  // STM32 controls the RF switch through interrupts from DIO2

	SX1262SetPacketType(mcu, radio, PACKET_TYPE_LORA);
	SX1262GetPacketType(mcu, radio);

	SX1262SetPaConfig(mcu, radio, power);
	//SX1262SetOCP(mcu, radio, 30);    // current max 30mA for the whole device
	SX1262SetTxParams(mcu, radio, power, PA_RAMP_200_US, false);  // WORKAROUND OFF
	SX1262SetBufferBaseAddress(mcu, radio, 0, 0);

	ModulationParams_t mod;
	mod.PacketType = PACKET_TYPE_LORA;
	mod.Params.LoRa.SpreadingFactor = LORA_SF12;
	mod.Params.LoRa.LowDatarateOptimize = LOW_DATARATE_OPTIMIZE_ON;
	mod.Params.LoRa.CodingRate = LORA_CR_4_8;
	mod.Params.LoRa.Bandwidth = LORA_BW_125;
	SX1262SetModulationParams(mcu, radio, &mod);

	PacketParams_t packet;
	packet.PacketType = PACKET_TYPE_LORA;
	packet.Params.LoRa.PreambleLength = 12;
	packet.Params.LoRa.PayloadLength = 4;
	packet.Params.LoRa.InvertIQ = IQ_NORMAL;
	packet.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
	packet.Params.LoRa.CrcMode = LORA_CRC_OFF;
	SX1262SetPacketParams(mcu, radio, &packet);

	SX1262SetRfFrequency(mcu, radio, freqInMHz * 1000000);

	//uint8_t LoRasyncWord[2] = {0x14, 0x24};
	//SX1262SetLoRaSyncWord(mcu, radio, LoRasyncWord);

	SX1262SetDioIrqParams(mcu, radio, IRQ_ALL, IRQ_RX_DONE, IRQ_NONE, IRQ_NONE);

 	SX1262ClearDeviceErrors(mcu, radio);

	CalibrationParams_t calib;
	calib.ADCBulkN = 1;
	calib.ADCBulkP = 1;
	calib.ADCPulse = 1;
	calib.Img = 1;
	calib.PLL = 1;
	calib.RC13M = 1;
	calib.RC64K = 1;
	SX1262Calibrate(mcu, radio, calib);

 	SX1262GetDeviceErrors(mcu, radio);
 	SX1262ClearIrqStatus(mcu, radio,  0xFFFF);

 	SX1262SetStandby(mcu, radio, STDBY_XOSC);

 	return RADIO_OK;
}


/*****************************************************************************
*  @Description	  Initializes the radio for the GFSK modulation. The user can
*                 define in the arguments the frequency and the power in dBm
******************************************************************************/
RadioState_t SX1262GfskInit(MCU* mcu, Radio* radio, int8_t power, uint16_t freqInMHz)
{
	SX1262Reset(mcu);
	SX1262RfSwitchInit(mcu);
	SX1262GetStatus(mcu, radio);

	SX1262SetStandby(mcu, radio, STDBY_RC);
	SX1262SetDio3AsTcxoCtrl(mcu, radio, TCXO_1_8V, TCXO_SETUP_TIME);

 	SX1262ClearDeviceErrors(mcu, radio);
	CalibrationParams_t calib;
	calib.ADCBulkN = 1;
	calib.ADCBulkP = 1;
	calib.ADCPulse = 1;
	calib.Img = 1;
	calib.PLL = 1;
	calib.RC13M = 1;
	calib.RC64K = 1;
	SX1262Calibrate(mcu, radio, calib);
 	SX1262GetDeviceErrors(mcu, radio);

	SX1262SetDio2AsRfSwitchCtrl(mcu, radio, true);  // STM32 controls the RF switch through interrupts from DIO2

	SX1262SetStandby(mcu, radio, STDBY_RC);
	SX1262SetRegulatorMode(mcu, radio, USE_DCDC);
	SX1262SetBufferBaseAddress(mcu, radio, 0, 0);

	SX1262SetPaConfig(mcu, radio, power);

	SX1262SetOCP(mcu, radio, 500);

	SX1262SetTxParams(mcu, radio, power, PA_RAMP_200_US, false);  // WORKAROUND OFF

	SX1262SetDioIrqParams(mcu, radio, IRQ_ALL, IRQ_RX_DONE, IRQ_NONE, IRQ_NONE);
 	SX1262ClearIrqStatus(mcu, radio,  0xFFFF);

	SX1262SetRfFrequency(mcu, radio, freqInMHz * 1000000);

	SX1262SetPacketType(mcu, radio, PACKET_TYPE_GFSK);
	SX1262GetPacketType(mcu, radio);

#if ((FUNCTION == TPMS_DEBUG) || (FUNCTION == TPMS_RELEASE))
	uint8_t GfskSyncWord[3] = {0xF0, 0xF0, 0xF0};
	if (SX1262SetGfskSyncWord(mcu, radio, GfskSyncWord, sizeof(GfskSyncWord)) != RADIO_OK)
		return RADIO_SAFE_WRITE_ERROR;
#else
	uint8_t GfskSyncWord[8] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0};
	if (SX1262SetGfskSyncWord(mcu, radio, GfskSyncWord, sizeof(GfskSyncWord)) != RADIO_OK)
		return RADIO_SAFE_WRITE_ERROR;
#endif


//	if (SX1262SetCrcPolynomial(mcu, radio, 0x1021) != RADIO_OK)
//		return RADIO_SAFE_WRITE_ERROR;

	ModulationParams_t mod;
	mod.PacketType = PACKET_TYPE_GFSK;
	mod.Params.Gfsk.BitRate = 60000;
	mod.Params.Gfsk.FdevHz  = 30000;
	mod.Params.Gfsk.ModulationShaping = NO_FILTER; //MOD_SHAPING_G_BT_05;
	mod.Params.Gfsk.Bandwidth = GFSK_BW_156200;
	SX1262SetModulationParams(mcu, radio, &mod);

#if ((FUNCTION == TPMS_DEBUG) || (FUNCTION == TPMS_RELEASE))
	PacketParams_t packet;
	packet.PacketType = PACKET_TYPE_GFSK;
	packet.Params.Gfsk.PreambleLength = 24;
	packet.Params.Gfsk.PreambleMinDetect = PREAMBLE_DETECTOR_08_BITS;
	packet.Params.Gfsk.SyncWordLength = 0x18;
	packet.Params.Gfsk.AddrComp = ADDRESSCOMP_FILT_OFF;
	packet.Params.Gfsk.HeaderType = GFSK_PACKET_FIXED_LENGTH;
	packet.Params.Gfsk.PayloadLength = 5 + 1;  // Payload plus CRC
	packet.Params.Gfsk.CrcType = GFSK_CRC_1_BYTES;
	packet.Params.Gfsk.DcFree = WHITENING_ON;
	SX1262SetPacketParams(mcu, radio, &packet);
	radio->function = TPMS;

	SX1262SetStandby(mcu, radio, STDBY_RC);
	SX1262SetRxTxFallbackMode(mcu, radio, STBY_RC);
#else
	PacketParams_t packet;
	packet.PacketType = PACKET_TYPE_GFSK;
	packet.Params.Gfsk.PreambleLength = 32;
	packet.Params.Gfsk.PreambleMinDetect = PREAMBLE_DETECTOR_32_BITS;
	packet.Params.Gfsk.SyncWordLength = 0x40;
	packet.Params.Gfsk.AddrComp = ADDRESSCOMP_FILT_OFF;
	packet.Params.Gfsk.HeaderType = GFSK_PACKET_FIXED_LENGTH;
	packet.Params.Gfsk.PayloadLength = 50 + 2;  // Payload plus CRC
	packet.Params.Gfsk.CrcType = GFSK_CRC_2_BYTES;
	packet.Params.Gfsk.DcFree = WHITENING_ON;
	SX1262SetPacketParams(mcu, radio, &packet);
	radio->function = TELEMETRY;

 	SX1262SetStandby(mcu, radio, STDBY_XOSC);
 	SX1262SetRxTxFallbackMode(mcu, radio, STBY_XOSC);
#endif

 	SX1262GetStatus(mcu, radio);

 	return RADIO_OK;
}



//RadioState_t SX1262GfskInitTpms(MCU* mcu, Radio* radio, int8_t power, uint16_t freqInMHz)
//{
//	SX1262Reset(mcu);
//	SX1262RfSwitchInit(mcu);
//	SX1262GetStatus(mcu, radio);
//
//	SX1262SetStandby(mcu, radio, STDBY_RC);
//	SX1262SetDio3AsTcxoCtrl(mcu, radio, TCXO_1_8V, TCXO_SETUP_TIME);
//
// 	SX1262ClearDeviceErrors(mcu, radio);
//	CalibrationParams_t calib;
//	calib.ADCBulkN = 1;
//	calib.ADCBulkP = 1;
//	calib.ADCPulse = 1;
//	calib.Img = 1;
//	calib.PLL = 1;
//	calib.RC13M = 1;
//	calib.RC64K = 1;
//	SX1262Calibrate(mcu, radio, calib);
// 	SX1262GetDeviceErrors(mcu, radio);
//
//	SX1262SetDio2AsRfSwitchCtrl(mcu, radio, true);  // STM32 controls the RF switch through interrupts from DIO2
//
//	SX1262SetStandby(mcu, radio, STDBY_RC);
//	SX1262SetRegulatorMode(mcu, radio, USE_DCDC);
//	SX1262SetBufferBaseAddress(mcu, radio, 0, 0);
//
//	SX1262SetPaConfig(mcu, radio, power);
//
//	SX1262SetOCP(mcu, radio, 500);
//
//	SX1262SetTxParams(mcu, radio, power, PA_RAMP_200_US, false);  // WORKAROUND OFF
//
//	SX1262SetDioIrqParams(mcu, radio, IRQ_ALL, IRQ_RX_DONE, IRQ_NONE, IRQ_NONE);
// 	SX1262ClearIrqStatus(mcu, radio,  0xFFFF);
//
//	SX1262SetRfFrequency(mcu, radio, freqInMHz * 1000000);
//
//	SX1262SetPacketType(mcu, radio, PACKET_TYPE_GFSK);
//	SX1262GetPacketType(mcu, radio);
//
//
//
//
////	if (SX1262SetCrcPolynomial(mcu, radio, 0x1021) != RADIO_OK)
////		return RADIO_SAFE_WRITE_ERROR;
//
//	ModulationParams_t mod;
//	mod.PacketType = PACKET_TYPE_GFSK;
//	mod.Params.Gfsk.BitRate = 60000;
//	mod.Params.Gfsk.FdevHz  = 30000;
//	mod.Params.Gfsk.ModulationShaping = NO_FILTER; //MOD_SHAPING_G_BT_05;
//	mod.Params.Gfsk.Bandwidth = GFSK_BW_156200;
//	SX1262SetModulationParams(mcu, radio, &mod);
//
//
//
// 	SX1262GetStatus(mcu, radio);
//
// 	return RADIO_OK;
//}



/*****************************************************************************
*  @Description	  Gets the received packet length and start address and then
*                 reads it from the buffer
******************************************************************************/
RadioState_t SX1262ReceiveMessage(MCU* mcu, Radio* radio)
{
	SX1262GetRxBufferStatus(mcu, radio);

	if (radio->payloadLength != radio->packetParams.Params.Gfsk.PayloadLength)
		return RADIO_PAYLOAD_LENGTH_ERROR;

	uint8_t received_data[radio->payloadLength];
	SX1262ReadBuffer(mcu, radio, radio->rxStartBufferPointer, received_data, radio->payloadLength);

	if (radio->function == TELEMETRY)
	{
		if (radio->irq.crcError == 0)
		{
			for (uint8_t i = 0; i < radio->payloadLength; ++i)
				radio->rx_msg[i] = received_data[i];

			radio->per.received_msgs++;
		}
		else   /* Wrong CRC telemetry message */
		{
			for (uint8_t i = 0; i < radio->payloadLength; ++i)
				radio->wrong_CRC_msg[i] = received_data[i];
		}
	}

	else   /* radio->function == TPMS */
		if (radio->irq.crcError == 0)
		{
			for (uint8_t i = 0; i < radio->payloadLength; ++i)
				radio->tpms_rx_msg[i] = received_data[i];
		}
		else   /* Wrong CRC TPMS message */
		{
			for (uint8_t i = 0; i < radio->payloadLength; ++i)
				radio->tpms_wrong_CRC_msg[i] = received_data[i];
		}

	return RADIO_OK;
}


/*****************************************************************************
*  @Description	  Scans the frequency window from start_freq to stop_freq and
*                 finds the minimum average RSSI, which is probably the
*                 frequency that is least occupied. sampling_time_ms dictates
*                 how long a specific frequency will be scanned to find RSSI avg
******************************************************************************/
uint16_t SX1262FindMostSilentFrequency(MCU* mcu, Radio* radio, uint16_t start_freq, uint16_t stop_freq, uint16_t sampling_time_ms)
{
	float RSSI[stop_freq - start_freq + 1];
	int32_t rssi_sum;
	uint16_t rssi_samples;
	uint32_t sampling_time_start;

	for (uint16_t freq = start_freq; freq <= stop_freq; ++freq)
	{
		SX1262GfskInit(mcu, radio, radio->power, freq);
		SX1262SetRx(mcu, radio, 0xFFFF);
		rssi_sum = 0;
		rssi_samples = 0;
		sampling_time_start = HAL_GetTick();
		while (HAL_GetTick() - sampling_time_start < sampling_time_ms)
		{
			rssi_sum += SX1262GetRssiInst(mcu, radio);
			rssi_samples++;
		}

		RSSI[freq - start_freq] = (float)(rssi_sum) / (float)(rssi_samples);
	}

	float min_rssi = RSSI[0];
	uint16_t min_rssi_freq = start_freq;

	for (uint16_t freq = start_freq; freq <= stop_freq; freq++)
	{
		if (RSSI[freq - start_freq] < min_rssi)
		{
			min_rssi = RSSI[freq - start_freq];
			min_rssi_freq = freq;
		}
	}

	return min_rssi_freq;
}



void SX1262CalculatePacketLoss(Radio* radio)
{
	radio->per.rx_counter = (radio->rx_msg[1] << 24) |
							(radio->rx_msg[2] << 16) |
			                (radio->rx_msg[3] << 8)  |
							(radio->rx_msg[4] << 0);

	if (radio->per.first_reception)
	{
		radio->per.initial_rx_counter = radio->per.rx_counter;
		radio->per.first_reception = false;
		radio->per.received_msgs = 1;
	}
	if (radio->per.received_msgs > radio->per.rx_counter)
	{
		radio->per.received_msgs = radio->per.rx_counter;
		radio->per.initial_rx_counter = radio->per.rx_counter;
	}

	radio->per.packet_loss = 100 - (float)((100.0 * radio->per.received_msgs) / (radio->per.rx_counter - radio->per.initial_rx_counter + 1));
}



/*****************************************************************************
*  @Description	  Sets the CRC polynomial used for the radio transactions. The
*                 default value is 0x1021
******************************************************************************/
RadioState_t SX1262SetCrcPolynomial(MCU* mcu, Radio* radio, uint16_t polynomial)
{
    uint8_t pol[2];

    pol[0] = polynomial >> 8;
    pol[1] = polynomial;

    return SX1262SafeWriteRegister(mcu, radio, CRC_POLYNOMIAL_VALUE_0_REG, pol, 2);
}


/*****************************************************************************
*  @Description	  Defines the behavior of the chip when an interrupt from the
*                 DIO1 pin has been received
******************************************************************************/
void SX1262dio1Interrupt(MCU* mcu, Radio* radio)
{
	SX1262GetIrqStatus(mcu, radio);
	if (radio->irq.rxDone == 1)  /* Message received */
	{
		if (radio->irq.crcError == 0)
		{
#if ((FUNCTION != TPMS_DEBUG) && (FUNCTION != TPMS_RELEASE))
			HAL_GPIO_WritePin(mcu->radio_rx_led.type, mcu->radio_rx_led.pin, GPIO_PIN_SET);
#endif
			SX1262ReceiveMessage(mcu, radio);  /* Message stored in the Radio struct */
			SX1262GetPacketStatus(mcu, radio);
#if ((FUNCTION != TPMS_DEBUG) && (FUNCTION != TPMS_RELEASE))
			HAL_GPIO_WritePin(mcu->radio_rx_led.type, mcu->radio_rx_led.pin, GPIO_PIN_RESET);
#endif
		}
		else
		{
			if (radio->function == TELEMETRY)
				radio->wrongCRC++;
			else
				radio->tpmsWrongCRC = true;

			SX1262ReceiveMessage(mcu, radio);  /* The message is received even if the CRC is wrong, but is stored in a different array */
			SX1262GetPacketStatus(mcu, radio);
		}
	}

	//SX1262ClearIrqStatus(mcu, radio, 0xFFFF);  // Clears both the SX1262 registers and the radio.irq struct
}


/*****************************************************************************
*  @Description	  Defines the behavior of the chip when an interrupt from the
*                 DIO2 pin has been received. This interrupt is generated both
*                 in rising and falling edges of DIO2
******************************************************************************/
void SX1262dio2Interrupt(MCU* mcu, Radio* radio)
{
	if (HAL_GPIO_ReadPin(mcu->dio2.type, mcu->dio2.pin) == GPIO_PIN_SET)            // If the interrupt occured from a rising edge
		HAL_GPIO_WritePin(mcu->rf_switch.type, mcu->rf_switch.pin, GPIO_PIN_SET);   // Connect the antenna to the TX path
	else																			// If the interrupt occured from a falling edge
		HAL_GPIO_WritePin(mcu->rf_switch.type, mcu->rf_switch.pin, GPIO_PIN_RESET); // Connect the antenna to the RX path
}

