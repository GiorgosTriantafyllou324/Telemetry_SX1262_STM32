#ifndef INC_TELEMSGS_USB_H_
#define INC_TELEMSGS_USB_H_


#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include "cJSON.h"
#include "SX1262.h"

#define RFU          0xFF;  // Reserved for future use
#define PACKETS_NUM  3      // Number of different telemetry packets


typedef enum
{
	AS_UNDEFINED  = 0x00,
	AS_OFF        = 0x01,
	AS_READY      = 0x02,
	AS_DRIVING    = 0x04,
	AS_FINISHED   = 0x08,
	AS_EMERGENCY  = 0x10,

}Autonomous_state_t;


typedef enum
{
	UNDEFINED_MISSION = 0x00,
	ACCELERATION      = 0x01,
	SKIDPAD           = 0x02,
	AUTOCROSS         = 0x04,
	TRACKDRIVE        = 0x08,
	EBS_TEST          = 0x10,
	INSPECTION        = 0x20,
	MANUAL            = 0x40,

}Mission_t;


/* Stores all the data transmitted / received from telemetry */
typedef struct
{
	/* Infrared Sensors */
	struct
	{
		uint16_t infrared_fr,
		         infrared_fl,
				 infrared_rr,
				 infrared_rl;
	}ir;

	/* Inverter */
	struct
	{
		// Flags
		uint8_t inv_enable 						: 1,  // ok
				inv_ok 							: 1,
				curr_lim_reached 				: 1,
				curr_lim_igbt_temp 				: 1,
				curr_lim_motor_temp 			: 1,
                curr_peak_val_warn 				: 1,
				defective_param 				: 1,
				hardware_fault 					: 1,
				faulty_safety_circuit 			: 1,
		        can_time_out_exceeded 			: 1,
				bad_encoder_signal 				: 1,
				no_power_supply_voltage 		: 1,
				overvoltage					 	: 1,
				overcurrent 					: 1,
				current_measure_fault 			: 1,
				ballast_circuit_overload 		: 1,
				faulty_run_signal_emi 			: 1,
				inactive_rfe 					: 1,
				power_supply_missing_or_too_low : 1,
				output_voltage_limit_reached 	: 1,
				overcurrent_200 				: 1,
				ballast_circuit_overload_87	    : 1;
	}inv;

	/* Accumulator Board */
	struct
	{
		// Flags
		uint8_t over_60v_dclink : 1, // ok
		        air_m_state 	: 1, // ok
		        air_m_supp 		: 1, // ok
				air_p_state 	: 1, // ok
				air_p_supp 		: 1, // ok
		        imd_ok 			: 1, // ok
				ams_ok 			: 1, // ok
				precharge_state : 1, // ok
				ts_active 		: 1, // ok
				vicor_overtemp  : 1;

		uint8_t max_cell_temp,        // ok
		        min_cell_temp,        // ok
				dcdc_temp,
				max_humidity,         // ok
				bms_error_code,       // ok
				last_bms_error_code,  // ok
		        tsac_error_code,      // ok
				last_tsac_error_code, // ok
				max_soc,              // ok
				min_soc,			  // ok
				error_position,       // ok
				max_cell_voltage_pos, // ok
				min_cell_voltage_pos; // ok

		uint16_t total_voltage_vs,    // ok
				 max_cell_voltage,    // ok
		         min_cell_voltage,    // ok
				 imd_isolation_kOhms; // ok

		int16_t wh_consumed,  // ok
				ah_consumed;  // ok

		 int16_t accu_current; // ok

	}accu;

	/* Power Distribution Unit / LV-BMS */
	struct
	{
		// Flags
		uint8_t undervoltage : 1,
				overvoltage  : 1,
				overtemp     : 1,
				overcurrent  : 1,
				dac1_enabled : 1,
				dac2_enabled : 1,
				pi_enabled   : 1;

		uint8_t max_cell_temp;       // ok

		uint16_t max_cell_voltage,   // ok
				 min_cell_voltage,   // ok
				 tdk1_current,       // ok
		         tdk2_current,       // ok
				 wh_consumed,        // ok
				 distribution_ratio; // ok

	}pdu;

	/* ASB Board */
	struct
	{
		// Flags
		uint8_t manual_begin		  : 1, // ok
				manual_asms_off		  : 1, // ok
				manual_mission_error  : 1, // ok
				manual_asms_off_error : 1, // ok
				right_sd_closed		  : 1, // ok
				left_sd_closed		  : 1, // ok
				vcu_alive			  : 1, // ok
				accu_alive			  : 1; // ok
	}asb;

	/* Tire Pressure Sensor */
	struct
	{
		uint16_t fr_press,  // ok
				 fr_temp,   // ok
				 fl_press,  // ok
				 fl_temp,   // ok
				 rr_press,  // ok
				 rr_temp,   // ok
				 rl_press,  // ok
				 rl_temp;   // ok

	}tpms;

	/* ECU Board */
	struct
	{
		// Flags
		uint8_t asb_ok	       : 1, 	 // ok
				bspd_ok	       : 1, 	 // ok
				sd_closed	   : 1,      // ok
				precharge_done : 1,      // ok
				r2d_done	   : 1,      // ok
				mission_pc_error   : 1,
				air_m_error        : 1,
				air_p_error        : 1,
				pc_pressed_error   : 1,
				imd_ams_bspd_error : 1,
				airs_opened_error  : 1,
				apps_deviaton		  : 1,
				apps_implausibility	  : 1,
				brake_deviation 	  : 1,
				brake_implausibility  : 1,
				software_bspd_enabled : 1,
				power_limiter_active  : 1;


		uint8_t brake_front, // ok
				brake_rear;  // ok

		//Autonomous_state_t as_status; // ok

		int16_t requested_torque, // ok
		 	 	actual_torque,    // ok
				motor_rpm;        // ok

		uint16_t apps1,    // ok
				 apps2,    // ok
				 hall_fr,
				 hall_fl,
				 hall_rr,  // ok
				 hall_rl;  // ok
				 // steering_linear;

	}vcu;

	/* Dashboard */
	struct
	{
		// FLAGS
		uint8_t r2d_pressed       : 1, // ok
				aux2_pressed      : 1, // ok
				regen_on	      : 1, // ok
				aero_fans_on      : 1, // ok
				pc_combo	      : 1, // ok
				cockpit_sd_closed : 1, // ok
				bots_sd_closed	  : 1, // ok
				inertia_sd_closed : 1, // ok
				mission_locked    : 1; // ok

		uint8_t power_limiter, // ok
				traction_def;  // ok

		Mission_t mission;     // ok

	}dash;

	/* Cooling (Rear) Control Unit */
	struct
	{
		// Flags
		uint8_t pumps_on				  : 1, // ok
				motor_temp_too_high		  : 1,
				device_temp_too_high 	  : 1,
				motor_temp_limit_reached  : 1,
				device_temp_limit_reached : 1,
				cooling_fan_on            : 1;

		uint8_t igbt_temp,   // ok
				motor_temp,  // ok
				water_temp0,
				water_temp1;

		uint16_t linear_rl,
				 linear_rr;

	}ccu;

	/* PLEX SDM-500 Display */
	struct
	{
		uint16_t accel_long,
				 accel_lat,
				 gps_speed;

	}plex;

}TelemetryData_t;








/* Functions processing received data / data about to be transmitted */
void process_can1_rx_msg(TelemetryData_t* teleData, uint32_t can_id, uint8_t* RxData);
void process_can2_rx_msg(TelemetryData_t* teleData, uint32_t can_id, uint8_t* RxData);

void CreateTeleTxMsg(Radio* radio, uint8_t id, TelemetryData_t* teleData);
void process_rx_packet(Radio* radio, TelemetryData_t* teleData);



/* Functions sending received data thru USB to the GUI */
cJSON* Make_Packet(cJSON* packet,const char* name, double value);
USBD_StatusTypeDef Transmit_USB(Radio* radio, TelemetryData_t* teleData, uint8_t id);






#endif /* INC_TELEMSGS_USB_H_ */
