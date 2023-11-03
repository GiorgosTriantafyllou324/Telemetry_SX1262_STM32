
#ifndef __SX126x_H__
#define __SX126x_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "SX1262_hal.h"



/* Constants */
#define XTAL_FREQ        		(double) 32000000
#define FREQ_DIV         		(double) 33554432  // 2^25
#define SX1262_MAX_LORA_SYMB_NUM_TIMEOUT 248
#define RADIO_WAKEUP_TIME                3 		   // [ms]    Radio complete Wake-up Time with margin for temperature compensation
#define AUTO_RX_TX_OFFSET                2 		   // Compensation delay for SetAutoTx/Rx functions in 15.625 microseconds
#define CRC_IBM_SEED                     0xFFFF    // LFSR initial value to compute IBM type CRC
#define CRC_CCITT_SEED                   0x1D0F    // LFSR initial value to compute CCIT type CRC
#define CRC_POLYNOMIAL_IBM               0x8005    // Polynomial used to compute IBM CRC
#define CRC_POLYNOMIAL_CCITT             0x1021    // Polynomial used to compute CCIT CRC

#define LORA_MAC_PRIVATE_SYNCWORD        0x1424    // Syncword for Private LoRa networks
#define LORA_MAC_PUBLIC_SYNCWORD         0x3444    // Syncword for Public LoRa networks


/* Commands */
typedef enum
{
	SET_STANDBY 			  = 0x80,
	SET_PACKET_TYPE 		  = 0x8A,
	SET_RF_FREQUENCY 		  = 0x86,
	SET_PA_CONFIG    		  = 0x95,
	SET_TX_PARAMS    		  = 0x8E,
	SET_BUFFER_BASE_ADDRESS   = 0x8F,
	SET_MODULATION_PARAMS     = 0x8B,
	SET_PACKET_PARAMS		  = 0x8C,
	SET_CAD_PARAMS			  = 0x88,
	SET_DIO_IRQ_PARAMS		  = 0x08,
	CLEAR_IRQ_STATUS          = 0x02,
	GET_IRQ_STATUS            = 0x12,
	SET_SLEEP				  = 0x84,
	SET_TX                    = 0x83,
	SET_RX 					  = 0x82,
	SET_FS					  = 0xC1,
	GET_RX_BUFFER_STATUS      = 0x13,
	CALIBRATE                 = 0x89,
	CALIBRATE_IMAGE           = 0x98,
	SET_RF_SWITCH_MODE        = 0x9D,
	SET_TCXO_MODE             = 0x97,
	STOP_TIMER_ON_PREAMBLE    = 0x9F,
	SET_RX_DUTY_CYCLE         = 0x94,
	SET_CAD					  = 0xC5,
	SET_TX_CONTINUOUS_WAVE    = 0xD1,
	SET_TX_INFINITE_PREAMBLE  = 0xD2,
	SET_REGULATOR_MODE		  = 0x96,
	SET_RX_TX_FALLBACK_MODE   = 0x93,
	GET_PACKET_TYPE			  = 0x11,
	SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
	GET_PACKET_STATUS         = 0x14,
	GET_RSSI_INST			  = 0x15,
	GET_STATS				  = 0x10,
	RESET_STATS				  = 0x00,
	GET_STATUS				  = 0xC0,
	GET_DEVICE_ERRORS		  = 0x17,
	CLEAR_DEVICE_ERRORS       = 0x07,
	WRITE_REGISTER            = 0x0D,
	READ_REGISTER    		  = 0x1D,
	WRITE_BUFFER              = 0x0E,
	READ_BUFFER      		  = 0x1E,

}RadioCommand_t;


/* Register Addresses */
typedef enum
{
	HOPPING_ENABLE_REG              = 0x0385,
	PACKET_LENGTH_REG               = 0x0386,
	NB_HOPPING_BLOCKS_REG			= 0x0387,
	NB_SYMBOLS_0_REG				= 0x0388,
	FREQ_0_REG						= 0x038A,
	NB_SYMBOLS_15_REG				= 0x03E2,
	DIOX_OUTPUT_ENABLE_REG			= 0x0580,
	DIOX_INPUT_ENABLE_REG			= 0x0583,
	DIOX_PULLUP_CONTROL_REG			= 0x0584,
	DIOX_PULLDOWN_CONTROL_REG		= 0x0585,
	WHITENING_MSB_REG				= 0x06B8,  // The address of the register holding the first byte defining the whitening seed
	WHITENING_LSB_REG				= 0x06B9,
	CRC_INITIAL_VALUE_0_REG			= 0x06BC,  // The address of the register holding the first byte defining the CRC seed
	CRC_POLYNOMIAL_VALUE_0_REG		= 0x06BE,  // The address of the register holding the first byte defining the CRC polynomial
	SYNC_WORD_0_REG		            = 0x06C0,  // The addresses of the registers holding SyncWords values
	NODE_ADDRESS_REG				= 0x06CD,
	BROADCAST_ADDRESS_REG			= 0x06CE,
	IQ_POLARITY_SETUP_REG			= 0x0736,  // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
	LORA_SYNC_WORD_0_REG			= 0x0740,  // The addresses of the register holding LoRa Modem SyncWord value
	RANDOM_NUMBER_GEN_0_REG			= 0x0819,  // The address of the register giving a 32-bit random number
	TX_MODULATION_REG				= 0x0889,  // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
	RX_GAIN_REG						= 0x08AC,  // The address of the register holding RX Gain value (0x94: power saving, 0x96: rx boosted)
	TX_CLAMP_CONFIG_REG             = 0x08D8,  // WORKAROUND - Better resistance to antenna mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
	OCP_CONFIGURATION_REG			= 0x08E7,  // Set the current max value in the over current protection
	RTC_CONTROL_REG					= 0x0902,  // RTC control
	XTA_TRIM_REG					= 0x0911,  // Change the value on the device internal trimming capacitor
	XTB_TRIM_REG					= 0x0912,
	DIO3_OUTPUT_VOLTAGE_REG			= 0x0920,
	EVENT_MASK_REG					= 0x0944,  // Event clear

/* --------------------------------------------------------------------------

// THE FOLLOWING ARE NOT INCLUDED IN THE DATASHEET

// The address of the register holding the packet configuration
REG_LR_PACKETPARAMS                        = 0x0704,

// The address of the register holding the payload size
REG_LR_PAYLOADLENGTH                       = 0x0702,

// The address of the register holding the re-calculated number of symbols
REG_LR_SYNCH_TIMEOUT                       = 0x0706,

// The address of the register used to disable the LNA
REG_ANA_LNA                                = 0x08E2,

// The address of the register used to disable the mixer
REG_ANA_MIXER                              = 0x08E5,

// Base address of the register retention list
REG_RETENTION_LIST_BASE_ADDRESS            = 0x029F,

----------------------------------------------------------------------------- */

}RadioRegAddress_t;



/* Declares the oscillator in use while in standby mode */
typedef enum
{
    STDBY_RC   = 0x00,
    STDBY_XOSC = 0x01,

}StandbyMode_t;



/* Declares the the communication protocol */
typedef enum
{
    PACKET_TYPE_GFSK    = 0x00,
    PACKET_TYPE_LORA    = 0x01,
    PACKET_TYPE_LR_FHSS = 0x03,

}PacketType_t;



/* Declares the ramping time for the power amplifier */
typedef enum
{
    PA_RAMP_10_US   = 0x00,
    PA_RAMP_20_US   = 0x01,
    PA_RAMP_40_US   = 0x02,
    PA_RAMP_80_US   = 0x03,
    PA_RAMP_200_US  = 0x04,
    PA_RAMP_800_US  = 0x05,
    PA_RAMP_1700_US = 0x06,
    PA_RAMP_3400_US = 0x07,

}RampTime_t;



/* Declares the modulation shaping parameter (Gaussian filter) */
typedef enum
{
    NO_FILTER           = 0x00,
    MOD_SHAPING_G_BT_03 = 0x08,
    MOD_SHAPING_G_BT_05 = 0x09,
    MOD_SHAPING_G_BT_07 = 0x0A,
    MOD_SHAPING_G_BT_1  = 0x0B,

}ModShaping_t;



/* Declares the receiver bandwidth when using GFSK */
typedef enum {

	GFSK_BW_4800   = 0x1F,  // 4.8 kHz
	GFSK_BW_5800   = 0x17,
	GFSK_BW_7300   = 0x0F,
	GFSK_BW_9700   = 0x1E,
	GFSK_BW_11700  = 0x16,
	GFSK_BW_14600  = 0x0E,
	GFSK_BW_19500  = 0x1D,
	GFSK_BW_23400  = 0x15,
	GFSK_BW_29300  = 0x0D,
	GFSK_BW_39000  = 0x1C,
	GFSK_BW_46900  = 0x14,
	GFSK_BW_58600  = 0x0C,
	GFSK_BW_78200  = 0x1B,
	GFSK_BW_93800  = 0x13,
	GFSK_BW_117300 = 0x0B,
	GFSK_BW_156200 = 0x1A,
	GFSK_BW_187200 = 0x12,
	GFSK_BW_234300 = 0x0A,
	GFSK_BW_312000 = 0x19,
	GFSK_BW_373600 = 0x11,
	GFSK_BW_467000 = 0x09,

}GfskBandwidth_t;



/* Declares the possible spreading factor values in LoRa protocol */
typedef enum
{
    LORA_SF5  = 0x05,
    LORA_SF6  = 0x06,
    LORA_SF7  = 0x07,
    LORA_SF8  = 0x08,
    LORA_SF9  = 0x09,
    LORA_SF10 = 0x0A,
    LORA_SF11 = 0x0B,
    LORA_SF12 = 0x0C,

}LoRaSpreadingFactor_t;



/* Declares the receiver bandwidth choices when using LoRa */
typedef enum
{
    LORA_BW_007 = 0x00,   // ~7kHz
    LORA_BW_010 = 0x08,
    LORA_BW_015 = 0x01,
    LORA_BW_020 = 0x09,
    LORA_BW_031 = 0x02,
    LORA_BW_041 = 0x0A,
    LORA_BW_062 = 0x03,
    LORA_BW_125 = 0x04,
    LORA_BW_250 = 0x05,
    LORA_BW_500 = 0x06,

}LoRaBandwidth_t;



/* Represents the coding rate values for LoRa packet type */
typedef enum
{
    LORA_CR_4_5 = 0x01,
    LORA_CR_4_6 = 0x02,
    LORA_CR_4_7 = 0x03,
    LORA_CR_4_8 = 0x04,

}LoRaCodingRate_t;



typedef enum
{
	LOW_DATARATE_OPTIMIZE_OFF = 0x00,
	LOW_DATARATE_OPTIMIZE_ON  = 0x01,

}LoRaLowDataRateOptimize_t;


/* Declares the protocol parameters in the modulation process */
typedef struct
{
    PacketType_t                  PacketType;             // Packet to which the modulation parameters are referring to.
    struct
    {
        struct
        {
            uint32_t                  BitRate;            // [bps]   600bps to 300kbps
            uint32_t                  FdevHz;			  // [Hz]    Determines the frequency distance between 1 and 0 in GFSK. Idreally, Modulation Index = 2 * FdevHz / BitRate = 1
            ModShaping_t              ModulationShaping;  // Dictates whether a Gaussian filter will be used (GFSK) or not (FSK)
            GfskBandwidth_t           Bandwidth;          // Determines the scanning frequency window of the receiver. It must be chosen so that: 2 * FdevHz + BitRate < Bandwidth
        }Gfsk;
        struct
        {
            LoRaSpreadingFactor_t     SpreadingFactor;     // Spreading Factor for the LoRa modulation
            LoRaBandwidth_t           Bandwidth;           // Bandwidth for the LoRa modulation
            LoRaCodingRate_t          CodingRate;          // Coding rate for the LoRa modulation
            LoRaLowDataRateOptimize_t LowDatarateOptimize; // Indicates if the modem uses the low datarate optimization
        }LoRa;
    }Params;                                               // Holds the modulation parameters structure

}ModulationParams_t;



/* Represents the preamble length used to detect the packet on Rx side */
typedef enum
{
    PREAMBLE_DETECTOR_OFF     = 0x00,         // Preamble detection length off
    PREAMBLE_DETECTOR_08_BITS = 0x04,         // Preamble detection length 8 bits
    PREAMBLE_DETECTOR_16_BITS = 0x05,         // Preamble detection length 16 bits
    PREAMBLE_DETECTOR_24_BITS = 0x06,         // Preamble detection length 24 bits
    PREAMBLE_DETECTOR_32_BITS = 0x07,         // Preamble detection length 32 bits

}PreambleDetection_t;



/* Represents the possible combinations of SyncWord correlators activated */
typedef enum
{
    ADDRESSCOMP_FILT_OFF        = 0x00,         // No correlator turned on, i.e. do not search for SyncWord
    ADDRESSCOMP_FILT_NODE       = 0x01,
    ADDRESSCOMP_FILT_NODE_BROAD = 0x02,

}AddressComp_t;



/* Radio GFSK packet length mode */
typedef enum
{
    GFSK_PACKET_FIXED_LENGTH    = 0x00,         // The packet is known on both sides, no header included in the packet
    GFSK_PACKET_VARIABLE_LENGTH = 0x01,         // The packet is on variable size, header included

}GfskPacketLength_t;



/* Represents the CRC length */
typedef enum
{
    GFSK_CRC_OFF          = 0x01,           // No CRC in use
    GFSK_CRC_1_BYTES      = 0x00,
    GFSK_CRC_2_BYTES      = 0x02,
    GFSK_CRC_1_BYTES_INV  = 0x04,
    GFSK_CRC_2_BYTES_INV  = 0x06,

	GFSK_CRC_2_BYTES_IBM  = 0xF1,
    GFSK_CRC_2_BYTES_CCIT = 0xF2,

}CrcType_t;



/* Radio whitening mode activated or deactivated */
typedef enum
{
	WHITENING_OFF = 0x00,
    WHITENING_ON  = 0x01,

}DcFree_t;



/* Holds the Radio lengths mode for the LoRa packet type */
typedef enum
{
    LORA_PACKET_VARIABLE_LENGTH = 0x00,         // The packet is on variable size, header included
    LORA_PACKET_FIXED_LENGTH    = 0x01,         // The packet is known on both sides, no header included in the packet

	/* Synonyms */
	LORA_PACKET_EXPLICIT        = LORA_PACKET_VARIABLE_LENGTH,
    LORA_PACKET_IMPLICIT        = LORA_PACKET_FIXED_LENGTH,

}LoRaPacketLength_t;



/* Represents the CRC mode for LoRa packet type */
typedef enum
{
    LORA_CRC_OFF = 0x00,         // CRC not used
    LORA_CRC_ON  = 0x01,         // CRC activated

}LoRaCrcMode_t;



/* Represents the IQ mode for LoRa packet type */
typedef enum
{
    IQ_NORMAL   = 0x00,
    IQ_INVERTED = 0x01,

}IQMode_t;



/* The type describing the packet parameters for every packet types */
typedef struct
{
    PacketType_t                    PacketType;    // Packet to which the packet parameters are referring to.
    struct
    {
        /* Holds the GFSK packet parameters */
        struct
        {
            uint16_t            PreambleLength;    // The preamble Tx length for GFSK packet type in bit. Number represents pairs of 1's and 0's
            PreambleDetection_t PreambleMinDetect; // The preamble Rx length minimal for GFSK packet type
            uint8_t             SyncWordLength;    // The synchronization word length for GFSK packet type (0x00 --> 0x40)
            AddressComp_t       AddrComp;          // Activated SyncWord correlators
            GfskPacketLength_t  HeaderType;        // If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
            uint8_t             PayloadLength;     // Size of the payload in the GFSK packet (0x00 --> 0xFF)
            CrcType_t           CrcType;           // Size of the CRC block in the GFSK packet
            DcFree_t            DcFree;
        }Gfsk;

        /* Holds the LoRa packet parameters */
        struct
        {
            uint16_t           PreambleLength;     // The preamble length is the number of LoRa symbols in the preamble
            LoRaPacketLength_t HeaderType;         // If the header is explicit, it will be transmitted in the LoRa packet. If the header is implicit, it will not be transmitted
            uint8_t            PayloadLength;      // Size of the payload in the LoRa packet  (0x00 --> 0xFF)
            LoRaCrcMode_t      CrcMode;            // Size of CRC block in LoRa packet
            IQMode_t           InvertIQ;           // Allows to swap IQ for LoRa packet
        }LoRa;
    }Params;                                       // Holds the packet parameters structure

}PacketParams_t;



/* Represents the operating mode the radio is actually running */
typedef enum
{
    STDBY_RC_MODE   = 0,         // The radio is in standby mode with RC oscillator
    STDBY_XOSC_MODE,             // The radio is in standby mode with XOSC oscillator
    FS_MODE,                     // The radio is in frequency synthesis mode
    TX_MODE,                     // The radio is in transmit mode
    RX_MODE,                     // The radio is in receive mode
	UNDEFINED,					 // GetStatus didn't return an expected mode
    RX_DC_MODE,                  // The radio is in receive duty cycle mode
    CAD_MODE,                    // The radio is in channel activity detection mode
    SLEEP_MODE,           		 // The radio is in sleep mode

}OperatingMode_t;



/* Represents the sleep mode configuration */
typedef enum {

	COLD_START = 0,
	WARM_START = 1,

}StartMode;
typedef enum {

	RTC_DISABLED = 0,
	RTC_WAKEUP   = 1,

}RtcMode;
typedef struct
{
	StartMode startMode;
	RtcMode   rtcMode;

}SleepParams_t;



/* Represents the calibration configuration */
typedef struct
{
    uint8_t RC64K    : 1;          // Calibrate RC64K clock
    uint8_t RC13M    : 1;          // Calibrate RC13M clock
    uint8_t PLL      : 1;          // Calibrate PLL
    uint8_t ADCPulse : 1;          // Calibrate ADC Pulse
    uint8_t ADCBulkN : 1;          // Calibrate ADC bulkN
    uint8_t ADCBulkP : 1;          // Calibrate ADC bulkP
    uint8_t Img      : 1;

}CalibrationParams_t;


// bit7: preamble err,  bit6: sync err,  bit5: adrs err,  bit4: crc err,  bit3: length err,  bit2: abort err,  bit1: packet received,  bit0: packet sent.
typedef struct
{


}RxStatus_t;



/* Represents the voltage used to control the TCXO from DIO3 */
typedef enum
{
    TCXO_1_6V  = 0x00,
    TCXO_1_7V  = 0x01,
    TCXO_1_8V  = 0x02,
    TCXO_2_2V  = 0x03,
    TCXO_2_4V  = 0x04,
    TCXO_2_7V  = 0x05,
    TCXO_3_0V  = 0x06,
    TCXO_3_3V  = 0x07,

}TcxoCtrlVoltage_t;



/* Declares the power regulation used to power the device */
typedef enum
{
    USE_LDO  = 0x00, // default
    USE_DCDC = 0x01,

}RegulatorMode_t;



/* Declares the mode of the radio after a transmission / reception */
typedef enum
{
	FS        = 0x40,
	STBY_XOSC = 0x30,
	STBY_RC   = 0x20,

}FallbackMode_t;



/* Represents the number of symbols to be used for channel activity detection operationn */
typedef enum
{
    CAD_01_SYMBOL = 0x00,
    CAD_02_SYMBOL = 0x01,
    CAD_04_SYMBOL = 0x02,
    CAD_08_SYMBOL = 0x03,
    CAD_16_SYMBOL = 0x04,

}CadSymbolsNum_t;



/* Represents the Channel Activity Detection actions after the CAD operation is finished */
typedef enum
{
    CAD_ONLY = 0x00,
    CAD_RX   = 0x01,

}CadExitMode_t;



/* Determines the state at which the radio is operating, since each function has its own packet types */
typedef enum
{
	TELEMETRY = 0,
	TPMS         ,

}Function_t;


/* Includes all the Channel Activity Detection parameters */
typedef struct
{
	uint8_t         cadDetPeak;
	uint8_t         cadDetMin;
	CadExitMode_t   cadExitMode;
	float           cadTimeout;   /* in ms */
	CadSymbolsNum_t cadSymbolNum;

}CadParams_t;



/* Includes the packet status for every packet type */
typedef struct
{
    struct
    {
        RxStatus_t  RxStatus;		   // Returns the status of the packet
        int8_t   RssiSync;             // The RSSI measured on last packet in dBm
        int8_t   RssiAvg;              // The averaged RSSI in dBm
    }Gfsk;
    struct
    {
        int8_t   RssiPacket;           // The RSSI of the last packet in dBm
        int8_t   SnrPacket;            // The SNR of the last packet in dB
        int8_t   SignalRssiPacket;	   // The RSSI after despreading on last packet in dB
    }LoRa;

}PacketStatus_t;



/* Represents the Rx internal counter values when GFSK or LoRa packet type is used */
typedef struct
{
	struct
	{
		uint16_t PacketReceived;
		uint16_t CrcError;		  // Initial name was CrcOk !!
		uint16_t LengthError;
	}Gfsk;
	struct
	{
		uint16_t PacketReceived;
		uint16_t CrcError;		  // Initial name was CrcOk !!
		uint16_t HeaderError;
	}LoRa;

}RxCounter_t;



/* Represents the possible radio system error states */
typedef struct
{
        uint8_t Rc64kCalibErr  : 1;        // RC 64kHz oscillator calibration failed
        uint8_t Rc13mCalibErr  : 1;        // RC 13MHz oscillator calibration failed
        uint8_t PllCalibErr    : 1;        // PLL calibration failed
        uint8_t AdcCalibErr    : 1;        // ADC calibration failed
        uint8_t ImgCalibErr    : 1;        // Image calibration failed
        uint8_t XoscStartErr   : 1;        // XOSC oscillator failed to start
        uint8_t PllLockErr     : 1;        // PLL lock failed
        uint8_t PaRampErr      : 1;        // PA ramp failed

}Error_t;


/* Declares the possible interrupts */
typedef struct
{
	uint8_t txDone			 : 1;
	uint8_t rxDone			 : 1;
	uint8_t preambleDetected : 1;
	uint8_t syncWordValid	 : 1;
	uint8_t headerValid		 : 1;
	uint8_t headerError		 : 1;
	uint8_t crcError		 : 1;
	uint8_t cadDone			 : 1;
	uint8_t cadDetected 	 : 1;
	uint8_t timeout 		 : 1;

}Interrupt_t;



/* SX1262SetDioIrqParams Arguments - Used to set which interrupts will be addressed to a DIO */
typedef enum
{
	IRQ_TIMEOUT             = 0b1000000000,  // Rx or Tx timeout
	IRQ_CAD_DETECTED        = 0b0100000000,  // channel activity detected
	IRQ_CAD_DONE            = 0b0010000000,  // channel activity detection finished
	IRQ_CRC_ERROR           = 0b0001000000,  // wrong CRC received
	IRQ_HEADER_ERROR        = 0b0000100000,  // LoRa header CRC error
	IRQ_HEADER_VALID        = 0b0000010000,  // valid LoRa header received
	IRQ_SYNC_WORD_VALID     = 0b0000001000,  // valid sync word detected
	IRQ_PREAMBLE_DETECTED   = 0b0000000100,  // preamble detected
	IRQ_RX_DONE             = 0b0000000010,  // packet received
	IRQ_TX_DONE             = 0b0000000001,  // packet transmission completed
	IRQ_ALL                 = 0b1111111111,  // all interrupts
	IRQ_NONE                = 0b0000000000,  // no interrupts

}Irq_t;


/* Stores variables for the calculation of the packet_loss - percentage of lost messages */
typedef struct PER {

	uint32_t tx_counter;
	uint32_t received_msgs;
	uint32_t rx_counter;
	uint32_t initial_rx_counter;

	bool first_reception;

	float packet_loss;

}PER;



/* This is the basic struct containing all configurations of the radio */
typedef struct Radio {

	uint8_t            spi_read_msg[READ_MSG_LEN];  // Message read from the MISO line of the SX1262

	uint8_t 		   rx_msg[MAX_RX_BYTES];        // Message received from the radio
	uint8_t            tx_msg[MAX_RX_BYTES];        // Message about to be transmitted from the radio
	uint8_t            wrong_CRC_msg[MAX_RX_BYTES]; // Messages received from telemetry with wrong CRC are stored here
	bool               msg_pending;                 // True if a new message has been received and not processed yet

	uint8_t            tpms_tx_msg[TPMS_MSG_BYTES + 1];
	uint8_t            tpms_rx_msg[TPMS_MSG_BYTES + 1];
	uint8_t            tpms_wrong_CRC_msg[TPMS_MSG_BYTES + 1]; // Messages received from TPMS with wrong CRC are stored here
	int8_t			   TpmsRssiAvg;                            // RSSI value of the last TPMS message received
	bool               tpmsWrongCRC;					       // Becomes true when a wrong CRC message has been received from a TPMS and false when it is registered in the TPMS struct
	int32_t            tpms_variation;                         // Stores the time difference between excpected message arrival and actual arrival from TPMS

	PacketType_t       packetType;
	bool 			   ImageCalibrated;
	uint32_t           rfFrequency;
	uint8_t            paDutyCycle;
	uint8_t            hpMax;
	int8_t             power;
	RampTime_t         rampTime;
	bool               workaround_on;
	uint8_t            txBaseAddress;
	uint8_t 		   rxBaseAddress;
	ModulationParams_t modulationParams;
	PacketParams_t     packetParams;
	PacketStatus_t     packetStatus;
	SleepParams_t	   sleepParams;
	CadParams_t		   cadParams;
	RxCounter_t        rxCounter;

	uint16_t           irqMask;
	uint16_t           dio1Mask;
	uint16_t           dio2Mask;
	uint16_t           dio3Mask;
	uint16_t		   irq_status;
	Interrupt_t        irq;

	OperatingMode_t    opMode;
	TcxoCtrlVoltage_t  tcxoVolt;
	RegulatorMode_t    regMode;
	FallbackMode_t     fallbackMode;

	Error_t            errors;
	uint16_t		   wrongCRC;
	uint16_t           tx_failures;   // Incremented every time the txDone flag is not set after every transmission

	uint8_t payloadLength;
	uint8_t rxStartBufferPointer;

	PER        per;
	Function_t function;

	/* Timing variables */
	uint16_t tele_tx_time;
	uint16_t tpms_tx_time;
	uint16_t tele_rx_time;
	uint16_t tpms_rx_time;
	uint16_t tx_interval;
	uint16_t rx_interval;  // time in ms since consecutive receptions

}Radio;


/* -------------- Higher Level Functions ----------- */
void SX1262Reset(MCU* mcu);
RadioState_t SX1262LoRaInit(MCU* mcu, Radio* radio, int8_t power, uint16_t freqInMHz);
RadioState_t SX1262GfskInit(MCU* mcu, Radio* radio, int8_t power, uint16_t freqInMHz);
void Radio_struct_init(Radio* radio);

/* -------------------- Setters -------------------- */
void SX1262SetStandby(MCU* mcu, Radio* radio, StandbyMode_t standby_mode);
void SX1262SetPacketType(MCU *mcu, Radio* radio, PacketType_t packet_type);
void SX1262SetRfFrequency(MCU *mcu, Radio* radio, uint32_t rf_frequency);
void SX1262SetPaConfig(MCU* mcu, Radio* radio, int8_t power);
void SX1262SetTxParams(MCU* mcu, Radio* radio, int8_t power, RampTime_t ramp_time, bool workaround_on);
void SX1262SetBufferBaseAddress(MCU* mcu, Radio* radio, uint8_t tx_base_address, uint8_t rx_base_address);
void SX1262SetPacketParams(MCU *mcu, Radio *radio, PacketParams_t *packetParams);
void SX1262SetModulationParams(MCU* mcu, Radio* radio, ModulationParams_t *modulationParams);
void SX1262SetDioIrqParams(MCU* mcu, Radio* radio, Irq_t irqMask, Irq_t dio1Mask, Irq_t dio2Mask, Irq_t dio3Mask);
RadioState_t SX1262SetGfskSyncWord(MCU* mcu, Radio* radio, uint8_t *GfsksyncWord, uint8_t len);
void SX1262SetLoRaSyncWord(MCU* mcu, Radio* radio, uint8_t *LoRasyncWord);
void SX1262SetTx(MCU* mcu, Radio* radio, uint16_t timeout_duration /* in msec */, bool wait);
void SX1262SetSleep(MCU* mcu, Radio* radio, SleepParams_t sleepConfig );
void SX1262SetFs(MCU* mcu, Radio* radio);
RadioState_t SX1262SetRx(MCU* mcu, Radio* radio, uint16_t timeout_duration /* in ms */);
RadioState_t SX1262SetRxBoosted(MCU* mcu, Radio* radio, uint32_t timeout_duration);
void SX1262SetDio2AsRfSwitchCtrl(MCU* mcu, Radio* radio, bool enable);
void SX1262SetDio3AsTcxoCtrl(MCU* mcu, Radio* radio, TcxoCtrlVoltage_t tcxoVoltage, uint8_t delay_duration);
void SX1262SetRxDutyCycle(MCU* mcu, Radio* radio, uint16_t rx_period, uint16_t sleep_period /* in ms */);
void SX1262SetCad(MCU* mcu, Radio* radio);
void SX1262SetTxContinuousWave (MCU* mcu, Radio* radio);
void SX1262SetTxInfinitePreamble(MCU* mcu, Radio* radio);
void SX1262SetRegulatorMode(MCU* mcu, Radio* radio, RegulatorMode_t mode);
void SX1262SetRxTxFallbackMode(MCU* mcu, Radio* radio, FallbackMode_t fallback_mode);
void SX1262SetPayload(MCU* mcu, Radio* radio, uint8_t *payload, uint8_t size);
void SX1262SetCadParams(MCU* mcu, Radio* radio, CadParams_t cadParams);
void SX1262SetLoRaSymbNumTimeout(MCU* mcu, Radio* radio, uint8_t symbNum);
void SX1262SetOCP(MCU* mcu, Radio* radio, uint16_t value /* [mA] */);
RadioState_t SX1262SetCrcPolynomial(MCU* mcu, Radio* radio, uint16_t polynomial);

/* --------------------- Getters ------------------------- */
void SX1262GetRxBufferStatus(MCU* mcu, Radio* radio);
void SX1262GetPacketType(MCU* mcu, Radio* radio);
void SX1262GetPacketStatus(MCU* mcu, Radio* radio);
int8_t SX1262GetRssiInst(MCU* mcu, Radio* radio);
void SX1262GetStats(MCU* mcu, Radio* radio);
void SX1262GetStatus(MCU* mcu, Radio* radio);
void SX1262GetDeviceErrors(MCU* mcu, Radio* radio);
void SX1262GetIrqStatus(MCU* mcu, Radio* radio);

/* ---------------------- Others ------------------------- */
void SX1262ResetStats(MCU* mcu, Radio* radio);
void SX1262StopTimerOnPreamble(MCU* mcu, Radio* radio, bool enable);
void SX1262Calibrate(MCU* mcu, Radio* radio, CalibrationParams_t calibParam);
void SX1262CalibrateImage(MCU* mcu, Radio* radio, uint32_t freq);
void SX1262ClearIrqStatus(MCU* mcu, Radio* radio, uint16_t irq);
void SX1262ClearDeviceErrors(MCU* mcu, Radio* radio);
void SX1262SendPayload(MCU* mcu, Radio* radio, uint8_t *payload, uint8_t size, uint8_t timeout, bool wait);
void SX1262CalculatePacketLoss(Radio* radio);
void SX1262CreateTeleTxMsg(Radio* radio, uint8_t id);
RadioState_t SX1262ReceiveMessage(MCU* mcu, Radio* radio);
uint16_t SX1262FindMostSilentFrequency(MCU* mcu, Radio* radio, uint16_t start_freq, uint16_t stop_freq, uint16_t sampling_time_ms);

/* --------------------- Privates ------------------------- */
uint32_t SX126xConvertFreqInHzToPllStep(uint32_t freqInHz);  // MIGHT BE USELESS

/* --------------------- Callbacks ------------------------- */
void SX1262dio1Interrupt(MCU* mcu, Radio* radio);
void SX1262dio2Interrupt(MCU* mcu, Radio* radio);






#endif
