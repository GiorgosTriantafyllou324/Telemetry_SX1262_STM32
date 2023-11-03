#include <TeleMsgs_USB.h>


/* Primary CAN bus */
void process_can1_rx_msg(TelemetryData_t* teleData, uint32_t can_id, uint8_t* RxData)
{
	switch(can_id)
	{

	case 0x181:         /* INVERTER RX ID - VCU FLAGS */
		switch(RxData[0])
		{
		/* VCU-generated flags */
		case 0x27:
			teleData->vcu.apps_deviaton 		= (RxData[1] >> 0) & 0x01;
			teleData->vcu.apps_implausibility	= (RxData[1] >> 1) & 0x01;
			teleData->vcu.brake_deviation 		= (RxData[1] >> 2) & 0x01;
			teleData->vcu.brake_implausibility  = (RxData[1] >> 3) & 0x01;
			teleData->vcu.software_bspd_enabled = (RxData[1] >> 4) & 0x01;
			teleData->vcu.power_limiter_active  = (RxData[1] >> 5) & 0x01;
			break;

		/* Inverter Status */
		case 0x40:
			teleData->inv.inv_enable 		  = (RxData[1] >> 0) & 0x01;
			teleData->inv.inv_ok 			  = (RxData[1] >> 1) & 0x01;
			teleData->inv.curr_lim_reached 	  = (RxData[4] >> 5) & 0x01;
			teleData->inv.curr_lim_igbt_temp  = (RxData[5] >> 0) & 0x01;
			teleData->inv.curr_lim_motor_temp = (RxData[5] >> 2) & 0x01;
			teleData->inv.curr_peak_val_warn  = (RxData[5] >> 4) & 0x01;
			break;

		/* Inverter Error Message */
		case 0x8F:
			teleData->inv.defective_param 		  = (RxData[1] >> 0) & 0x01;
			teleData->inv.hardware_fault 		  = (RxData[1] >> 1) & 0x01;
			teleData->inv.faulty_safety_circuit   = (RxData[1] >> 2) & 0x01;
			teleData->inv.can_time_out_exceeded   = (RxData[1] >> 3) & 0x01;
			teleData->inv.bad_encoder_signal 	  = (RxData[1] >> 4) & 0x01;
			teleData->inv.no_power_supply_voltage = (RxData[1] >> 5) & 0x01;

			teleData->inv.overvoltage 			   = (RxData[2] >> 0) & 0x01;
			teleData->inv.overcurrent 			   = (RxData[2] >> 1) & 0x01;
			teleData->inv.current_measure_fault    = (RxData[2] >> 6) & 0x01;
			teleData->inv.ballast_circuit_overload = (RxData[2] >> 7) & 0x01;

			teleData->inv.faulty_run_signal_emi 		  = (RxData[3] >> 1) & 0x01;
			teleData->inv.inactive_rfe 					  = (RxData[3] >> 2) & 0x01;
			teleData->inv.power_supply_missing_or_too_low = (RxData[3] >> 5) & 0x01;

			teleData->inv.output_voltage_limit_reached = (RxData[4] >> 0) & 0x01;
			teleData->inv.overcurrent_200 			   = (RxData[4] >> 1) & 0x01;
			teleData->inv.ballast_circuit_overload_87  = (RxData[4] >> 7) & 0x01;
			break;

		default:
			break;
		}
		break;

	case 0x300:
		teleData->accu.bms_error_code       = RxData[0];
		teleData->accu.last_bms_error_code  = RxData[1];
		teleData->accu.tsac_error_code      = RxData[2];
		teleData->accu.last_tsac_error_code = RxData[3];
		teleData->accu.error_position       = RxData[4];
		break;

	case 0x301:
		teleData->accu.total_voltage_vs = (RxData[0] << 8) | RxData[1];
		teleData->accu.accu_current     = (RxData[2] << 8) | RxData[3];
		teleData->accu.wh_consumed      = (RxData[4] << 8) | RxData[5];
		teleData->accu.ah_consumed      = (RxData[6] << 8) | RxData[7];
		break;

	case 0x302:
		teleData->accu.max_cell_voltage		= (RxData[0] << 8) | RxData[1];
		teleData->accu.max_cell_voltage_pos =  RxData[2];
		teleData->accu.min_cell_voltage 	= (RxData[3] << 8) | RxData[4];
		teleData->accu.min_cell_voltage_pos =  RxData[5];
		teleData->accu.max_soc              =  RxData[6];
		teleData->accu.min_soc              =  RxData[7];
		break;

	case 0x303:
		teleData->accu.max_cell_temp = RxData[0];
		teleData->accu.min_cell_temp = RxData[2];
		break;

	case 0x304:
		teleData->accu.over_60v_dclink =   (RxData[0] >> 0) & 0x01;
		teleData->accu.air_m_state     =   (RxData[0] >> 1) & 0x01;
		teleData->accu.air_m_supp      =   (RxData[0] >> 2) & 0x01;
		teleData->accu.air_p_state     =   (RxData[0] >> 3) & 0x01;
		teleData->accu.air_p_supp      =   (RxData[0] >> 4) & 0x01;
		teleData->accu.imd_ok          = !((RxData[0] >> 5) & 0x01);
		teleData->accu.ams_ok          = !((RxData[0] >> 6) & 0x01);
		teleData->accu.precharge_state =   (RxData[0] >> 7) & 0x01;
		teleData->accu.ts_active       =   (RxData[1] >> 0) & 0x01;
		teleData->accu.vicor_overtemp  =   (RxData[1] >> 1) & 0x01;

		teleData->accu.dcdc_temp    = RxData[2];
		teleData->accu.max_humidity = RxData[3];

		teleData->accu.imd_isolation_kOhms = (RxData[5] << 8) | RxData[6];
		break;

	case 0x305:
		teleData->vcu.hall_fl = (RxData[0] << 8) | RxData[1];
		teleData->vcu.hall_fr = (RxData[2] << 8) | RxData[3];
		teleData->vcu.hall_rl = (RxData[4] << 8) | RxData[5];
		teleData->vcu.hall_rr = (RxData[6] << 8) | RxData[7];
		break;

	case 0x306:
		teleData->vcu.actual_torque    = (RxData[0] << 8) | RxData[1];
		teleData->vcu.requested_torque = (RxData[2] << 8) | RxData[3];
		teleData->vcu.motor_rpm		   = (RxData[4] << 8) | RxData[5];
		break;

	case 0x307:
		teleData->vcu.asb_ok     = !((RxData[0] >> 2) & 0x01);
		teleData->vcu.bspd_ok    = !((RxData[0] >> 3) & 0x01);
		teleData->vcu.sd_closed  =   (RxData[0] >> 4) & 0x01;

		teleData->vcu.apps1 	  = RxData[1];
		teleData->vcu.apps2 	  = RxData[2];
		teleData->vcu.brake_front = RxData[4];
		teleData->vcu.brake_rear  = RxData[5];
		break;

	case 0x308:
		teleData->vcu.precharge_done     = (RxData[0] >> 0) & 0x01;
		teleData->vcu.r2d_done           = (RxData[0] >> 1) & 0x01;
		teleData->vcu.mission_pc_error   = (RxData[1] >> 0) & 0x01;
		teleData->vcu.air_m_error        = (RxData[1] >> 1) & 0x01;
		teleData->vcu.air_p_error        = (RxData[1] >> 2) & 0x01;
		teleData->vcu.pc_pressed_error   = (RxData[1] >> 3) & 0x01;
		teleData->vcu.imd_ams_bspd_error = (RxData[1] >> 4) & 0x01;
		teleData->vcu.airs_opened_error  = (RxData[1] >> 5) & 0x01;
		break;

	case 0x309:
		teleData->dash.r2d_pressed  = (RxData[0] >> 0) & 0x01;
		teleData->dash.pc_combo     = (RxData[0] >> 5) & 0x01;
		break;

	case 0x310:
		teleData->dash.mission 		  =  RxData[0]       & 0x7F; // Excude the mission lock bit
		teleData->dash.mission_locked = (RxData[0] >> 7) & 0x01;
		break;

	case 0x311:
		teleData->pdu.max_cell_voltage = (RxData[0] << 8) | RxData[1];
		teleData->pdu.min_cell_voltage = (RxData[2] << 8) | RxData[3];
		teleData->pdu.max_cell_temp    = RxData[4];
		teleData->pdu.tdk1_current 		  = (RxData[6] << 8) | RxData[7];
		break;

	case 0x312:
		teleData->pdu.tdk2_current = (RxData[0] << 8) | RxData[1];
		teleData->pdu.wh_consumed  = (RxData[2] << 8) | RxData[3];

		teleData->pdu.undervoltage = (RxData[4] >> 0) & 0x01;
		teleData->pdu.overvoltage  = (RxData[4] >> 1) & 0x01;
		teleData->pdu.overtemp     = (RxData[4] >> 2) & 0x01;
		teleData->pdu.overcurrent  = (RxData[4] >> 3) & 0x01;
		teleData->pdu.dac1_enabled = (RxData[4] >> 4) & 0x01;
		teleData->pdu.dac2_enabled = (RxData[4] >> 5) & 0x01;
		teleData->pdu.pi_enabled   = (RxData[4] >> 6) & 0x01;

		teleData->pdu.distribution_ratio = (RxData[5] << 8) | RxData[6];
		break;

	case 0x313:
		teleData->dash.power_limiter =  RxData[0];
		teleData->dash.traction_def  =  RxData[1];

		teleData->dash.aux2_pressed       = (RxData[2] >> 0) & 0x01;
		teleData->dash.regen_on           = (RxData[2] >> 1) & 0x01;
		teleData->dash.aero_fans_on       = (RxData[2] >> 2) & 0x01;
		teleData->dash.cockpit_sd_closed  = (RxData[2] >> 3) & 0x01;
		teleData->dash.bots_sd_closed     = (RxData[2] >> 4) & 0x01;
		teleData->dash.inertia_sd_closed  = (RxData[2] >> 5) & 0x01;
		break;

	case 0x403:
		teleData->asb.vcu_alive  = !(RxData[0] == 0);
		teleData->asb.accu_alive = !(RxData[1] == 0);
		break;
	case 0x404:

		/* Manual Errors */
		teleData->asb.manual_begin							 = (RxData[5] >> 0) & 0x01;
		teleData->asb.manual_asms_off						 = (RxData[5] >> 1) & 0x01;
		teleData->asb.manual_mission_error					 = (RxData[5] >> 4) & 0x01;
		teleData->asb.manual_asms_off_error 				 = (RxData[5] >> 5) & 0x01;
		teleData->asb.right_sd_closed = (RxData[6] >> 2) & 0x01;
		teleData->asb.left_sd_closed  = (RxData[6] >> 3) & 0x01;
		break;

	default:
		break;
	}
}


/* Sensory CAN bus */
void process_can2_rx_msg(TelemetryData_t* teleData, uint32_t can_id, uint8_t* RxData)
{
	switch(can_id)
	{
	case 0x4D0:

		teleData->ccu.water_temp0    = RxData[0];
		teleData->ccu.water_temp1    = RxData[1];
		teleData->ccu.cooling_fan_on = RxData[2];

		teleData->ccu.pumps_on					= (RxData[3] >> 0) & 0x01;
		teleData->ccu.motor_temp_too_high 		= (RxData[3] >> 1) & 0x01;
		teleData->ccu.device_temp_too_high 		= (RxData[3] >> 2) & 0x01;
		teleData->ccu.motor_temp_limit_reached  = (RxData[3] >> 3) & 0x01;
		teleData->ccu.device_temp_limit_reached = (RxData[3] >> 4) & 0x01;

		teleData->ccu.igbt_temp   = RxData[4];
		teleData->ccu.motor_temp  = RxData[5];
		break;

	case 0x4E0:
		/* Average of the 4 channels */
		teleData->ir.infrared_fl = (((RxData[0] << 8) | RxData[1]) +
				                    ((RxData[2] << 8) | RxData[3]) +
								    ((RxData[4] << 8) | RxData[5]) +
				                    ((RxData[6] << 8) | RxData[7])) / 4;
		break;

	case 0x4E1:
		/* Average of the 4 channels */
		teleData->ir.infrared_fr = (((RxData[0] << 8) | RxData[1]) +
				       	   	        ((RxData[2] << 8) | RxData[3]) +
								    ((RxData[4] << 8) | RxData[5]) +
								    ((RxData[6] << 8) | RxData[7])) / 4;
		break;

	case 0x4E2:
		/* Average of the 4 channels */
		teleData->ir.infrared_rl = (((RxData[0] << 8) | RxData[1]) +
				       	   	        ((RxData[2] << 8) | RxData[3]) +
								    ((RxData[4] << 8) | RxData[5]) +
								    ((RxData[6] << 8) | RxData[7])) / 4;
		break;

	case 0x4E3:
		/* Average of the 4 channels */
		teleData->ir.infrared_rr = (((RxData[0] << 8) | RxData[1]) +
				       	   	        ((RxData[2] << 8) | RxData[3]) +
								    ((RxData[4] << 8) | RxData[5]) +
								    ((RxData[6] << 8) | RxData[7])) / 4;
		break;

	case 0x400:

		teleData->plex.accel_long = (RxData[0] << 8) | RxData[1];
		teleData->plex.accel_lat  = (RxData[2] << 8) | RxData[3];
		teleData->plex.gps_speed  = (RxData[4] << 8) | RxData[5];
		break;

	case 0x4D1:

		teleData->ccu.linear_rl = RxData[0] * 100 + RxData[1];
		teleData->ccu.linear_rr = RxData[2] * 100 + RxData[3];
		break;

	default:
		break;
	}
}



void CreateTeleTxMsg(Radio* radio, uint8_t id, TelemetryData_t* teleData)
{
	radio->per.tx_counter++;

	radio->tx_msg[0] = id;
	radio->tx_msg[1] = radio->per.tx_counter >> 24;
	radio->tx_msg[2] = radio->per.tx_counter >> 16;
	radio->tx_msg[3] = radio->per.tx_counter >> 8;
	radio->tx_msg[4] = radio->per.tx_counter;

	switch(id)
	{
	case 1:
		radio->tx_msg[5] = (teleData->accu.over_60v_dclink << 7) |
						   (teleData->accu.air_m_state     << 6) |
						   (teleData->accu.air_m_supp      << 5) |
						   (teleData->accu.air_p_state     << 4) |
						   (teleData->accu.air_p_supp      << 3) |
						   (teleData->accu.precharge_state << 2) |
						   (teleData->accu.ts_active       << 1) |
						   (teleData->accu.vicor_overtemp  << 0);
		radio->tx_msg[6]  = teleData->accu.max_cell_temp;
		radio->tx_msg[7]  = teleData->accu.min_cell_temp;
		radio->tx_msg[8]  = teleData->accu.dcdc_temp;
		radio->tx_msg[9]  = teleData->accu.max_humidity;
		radio->tx_msg[10] = teleData->accu.bms_error_code;
		radio->tx_msg[11] = teleData->accu.tsac_error_code;
		radio->tx_msg[12] = teleData->accu.max_soc;
		radio->tx_msg[13] = teleData->accu.total_voltage_vs >> 8;
		radio->tx_msg[14] = teleData->accu.total_voltage_vs;
		radio->tx_msg[15] = teleData->accu.accu_current >> 8;
		radio->tx_msg[16] = teleData->accu.accu_current;
		radio->tx_msg[17] = teleData->accu.max_cell_voltage >> 8;
		radio->tx_msg[18] = teleData->accu.max_cell_voltage;
		radio->tx_msg[19] = teleData->accu.min_cell_voltage >> 8;
		radio->tx_msg[20] = teleData->accu.min_cell_voltage;
		radio->tx_msg[21] = teleData->accu.wh_consumed >> 8;
		radio->tx_msg[22] = teleData->accu.wh_consumed;
		radio->tx_msg[23] = teleData->accu.ah_consumed >> 8;
		radio->tx_msg[24] = teleData->accu.ah_consumed;
		radio->tx_msg[25] = teleData->accu.error_position;
		radio->tx_msg[26] = teleData->accu.max_cell_voltage_pos;
		radio->tx_msg[27] = teleData->accu.min_cell_voltage_pos;

		radio->tx_msg[28] = teleData->pdu.max_cell_temp;
		radio->tx_msg[29] = teleData->pdu.max_cell_voltage >> 8;
		radio->tx_msg[30] = teleData->pdu.max_cell_voltage;
		radio->tx_msg[31] = teleData->pdu.min_cell_voltage >> 8;
		radio->tx_msg[32] = teleData->pdu.min_cell_voltage;
		radio->tx_msg[33] = teleData->pdu.tdk1_current >> 8;
		radio->tx_msg[34] = teleData->pdu.tdk1_current;
		radio->tx_msg[35] = teleData->pdu.tdk2_current >> 8;
		radio->tx_msg[36] = teleData->pdu.tdk2_current;
		radio->tx_msg[37] = teleData->pdu.wh_consumed >> 8;
		radio->tx_msg[38] = teleData->pdu.wh_consumed;
		radio->tx_msg[39] = teleData->accu.last_bms_error_code;
		radio->tx_msg[40] = teleData->accu.last_tsac_error_code;

		radio->tx_msg[41] = (teleData->inv.inv_enable          << 7) |
							(teleData->inv.inv_ok              << 6) |
							(teleData->inv.curr_lim_reached    << 5) |
							(teleData->inv.curr_lim_igbt_temp  << 4) |
							(teleData->inv.curr_lim_motor_temp << 3) |
							(teleData->inv.curr_peak_val_warn  << 2) |
							(teleData->inv.defective_param 	   << 1) |
							(teleData->inv.hardware_fault      << 0);
		radio->tx_msg[42] = (teleData->inv.faulty_safety_circuit    << 7) |
							(teleData->inv.can_time_out_exceeded    << 6) |
							(teleData->inv.bad_encoder_signal   	<< 5) |
							(teleData->inv.no_power_supply_voltage  << 4) |
							(teleData->inv.overvoltage 				<< 3) |
							(teleData->inv.overcurrent  			<< 2) |
							(teleData->inv.current_measure_fault    << 1) |
							(teleData->inv.ballast_circuit_overload << 0);
		radio->tx_msg[43] = (teleData->inv.faulty_run_signal_emi           << 5) |
							(teleData->inv.inactive_rfe              	   << 4) |
							(teleData->inv.power_supply_missing_or_too_low << 3) |
							(teleData->inv.output_voltage_limit_reached    << 2) |
							(teleData->inv.overcurrent_200 				   << 1) |
							(teleData->inv.ballast_circuit_overload_87     << 0);
		radio->tx_msg[44] = (teleData->pdu.pi_enabled   << 6) |
							(teleData->pdu.dac2_enabled << 5) |
							(teleData->pdu.dac1_enabled << 4) |
							(teleData->pdu.overcurrent  << 3) |
							(teleData->pdu.overtemp     << 2) |
							(teleData->pdu.overvoltage  << 1) |
							(teleData->pdu.undervoltage << 0);
		radio->tx_msg[45] =  teleData->pdu.distribution_ratio >> 8;
		radio->tx_msg[46] =  teleData->pdu.distribution_ratio;

		radio->tx_msg[47] = teleData->accu.min_soc;

		radio->tx_msg[48] = (teleData->vcu.mission_pc_error   << 7) |
							(teleData->vcu.air_m_error		  << 6) |
							(teleData->vcu.air_p_error 		  << 5) |
							(teleData->vcu.pc_pressed_error   << 4) |
							(teleData->vcu.imd_ams_bspd_error << 3) |
							(teleData->vcu.airs_opened_error  << 2) |
							(teleData->vcu.precharge_done     << 1) |
							(teleData->vcu.r2d_done           << 0);

		radio->tx_msg[49] = (teleData->vcu.power_limiter_active  << 5) |
							(teleData->vcu.software_bspd_enabled << 4) |
							(teleData->vcu.brake_implausibility  << 3) |
							(teleData->vcu.brake_deviation       << 2) |
							(teleData->vcu.apps_implausibility   << 1) |
							(teleData->vcu.apps_deviaton         << 0);

		break;

	case 2:
		radio->tx_msg[5]  = RFU;
		radio->tx_msg[6]  = teleData->vcu.brake_front;
		radio->tx_msg[7]  = teleData->vcu.brake_rear;
		radio->tx_msg[8]  = teleData->vcu.actual_torque >> 8;
		radio->tx_msg[9]  = teleData->vcu.actual_torque;
		radio->tx_msg[10] = teleData->vcu.requested_torque >> 8;
		radio->tx_msg[11] = teleData->vcu.requested_torque;
		radio->tx_msg[12] = teleData->vcu.motor_rpm >> 8;
		radio->tx_msg[13] = teleData->vcu.motor_rpm;
		radio->tx_msg[14] = teleData->vcu.apps1 >> 8;
		radio->tx_msg[15] = teleData->vcu.apps1;
		radio->tx_msg[16] = teleData->vcu.apps2 >> 8;
		radio->tx_msg[17] = teleData->vcu.apps2;
		radio->tx_msg[18] = teleData->vcu.hall_fr >> 8;
		radio->tx_msg[19] = teleData->vcu.hall_fr;
		radio->tx_msg[20] = teleData->vcu.hall_fl >> 8;
		radio->tx_msg[21] = teleData->vcu.hall_fl;
		radio->tx_msg[22] = teleData->vcu.hall_rr >> 8;
		radio->tx_msg[23] = teleData->vcu.hall_rr;
		radio->tx_msg[24] = teleData->vcu.hall_rl >> 8;
		radio->tx_msg[25] = teleData->vcu.hall_rl;
		radio->tx_msg[26] = teleData->accu.imd_isolation_kOhms >> 8;
		radio->tx_msg[27] = teleData->accu.imd_isolation_kOhms;
		radio->tx_msg[28] = RFU;
		radio->tx_msg[29] = RFU;

		radio->tx_msg[30] = (teleData->dash.r2d_pressed     << 7) |
							(teleData->dash.pc_combo        << 6) |
							(teleData->dash.aux2_pressed    << 5) |
							(teleData->dash.regen_on        << 4) |
							(teleData->dash.aero_fans_on    << 3);

		radio->tx_msg[31] =  teleData->dash.power_limiter;
		radio->tx_msg[32] =  teleData->dash.traction_def;
		radio->tx_msg[33] =  teleData->dash.mission;
		radio->tx_msg[34] = (teleData->dash.mission_locked << 0) & 0x01;
		radio->tx_msg[35] =  RFU;

		radio->tx_msg[36] = (teleData->ccu.cooling_fan_on			 << 5) |
							(teleData->ccu.pumps_on 				 << 4) |
							(teleData->ccu.motor_temp_too_high 		 << 3) |
							(teleData->ccu.device_temp_too_high		 << 2) |
							(teleData->ccu.motor_temp_limit_reached  << 1) |
							(teleData->ccu.device_temp_limit_reached << 0);
		radio->tx_msg[37] =  teleData->ccu.igbt_temp;
		radio->tx_msg[38] =  teleData->ccu.motor_temp;
		radio->tx_msg[39] =  RFU;

		radio->tx_msg[40] = teleData->plex.accel_long >> 8;
		radio->tx_msg[41] = teleData->plex.accel_long;
		radio->tx_msg[42] = teleData->plex.accel_lat >> 8;
		radio->tx_msg[43] = teleData->plex.accel_lat;
		radio->tx_msg[44] = teleData->plex.gps_speed >> 8;
		radio->tx_msg[45] = teleData->plex.gps_speed;
		radio->tx_msg[46] = teleData->ccu.water_temp0;
		radio->tx_msg[47] = teleData->ccu.water_temp1;
		radio->tx_msg[48] = RFU;
		radio->tx_msg[49] = RFU;
		break;

	case 3:
		radio->tx_msg[5]  = teleData->ir.infrared_fr >> 8;
		radio->tx_msg[6]  = teleData->ir.infrared_fr;
		radio->tx_msg[7]  = teleData->ir.infrared_fl >> 8;
		radio->tx_msg[8]  = teleData->ir.infrared_fl;
		radio->tx_msg[9]  = teleData->ir.infrared_rr >> 8;
		radio->tx_msg[10] = teleData->ir.infrared_rr;
		radio->tx_msg[11] = teleData->ir.infrared_rl >> 8;
		radio->tx_msg[12] = teleData->ir.infrared_rl;

		radio->tx_msg[13] = teleData->ccu.linear_rr >> 8;
		radio->tx_msg[14] = teleData->ccu.linear_rr;
		radio->tx_msg[15] = teleData->ccu.linear_rl >> 8;
		radio->tx_msg[16] = teleData->ccu.linear_rl;

		radio->tx_msg[17] = (teleData->asb.manual_begin 		 << 5) |
							(teleData->asb.manual_asms_off 		 << 4) |
							(teleData->asb.manual_mission_error	 << 3) |
							(teleData->asb.manual_asms_off_error << 2) |
							(teleData->asb.vcu_alive	 		 << 1) |
							(teleData->asb.accu_alive	 		 << 0);

		radio->tx_msg[18] = teleData->tpms.fr_temp >> 8;
		radio->tx_msg[19] = teleData->tpms.fr_temp;
		radio->tx_msg[20] = teleData->tpms.fr_press >> 8;
		radio->tx_msg[21] = teleData->tpms.fr_press;
		radio->tx_msg[22] = teleData->tpms.fl_temp >> 8;
		radio->tx_msg[23] = teleData->tpms.fl_temp;
		radio->tx_msg[24] = teleData->tpms.fl_press >> 8;
		radio->tx_msg[25] = teleData->tpms.fl_press;
		radio->tx_msg[26] = teleData->tpms.rr_temp >> 8;
		radio->tx_msg[27] = teleData->tpms.rr_temp;
		radio->tx_msg[28] = teleData->tpms.rr_press >> 8;
		radio->tx_msg[29] = teleData->tpms.rr_press;
		radio->tx_msg[30] = teleData->tpms.rl_temp >> 8;
		radio->tx_msg[31] = teleData->tpms.rl_temp;
		radio->tx_msg[32] = teleData->tpms.rl_press >> 8;
		radio->tx_msg[33] = teleData->tpms.rl_press;

		/* ShutDown Nodes Monitoring */
		radio->tx_msg[34] = (teleData->vcu.bspd_ok  		  << 7) |
							(teleData->vcu.asb_ok   		  << 6) |
							(teleData->accu.imd_ok  		  << 4) |
							(teleData->accu.ams_ok 			  << 3) |
							(teleData->dash.bots_sd_closed    << 2) |
							(teleData->dash.cockpit_sd_closed << 1) |
							(teleData->dash.inertia_sd_closed << 0);

		radio->tx_msg[35] = (teleData->vcu.sd_closed       	<< 4) |
							(teleData->asb.right_sd_closed 	<< 3) |
							(teleData->asb.left_sd_closed 	<< 2) |
							(teleData->accu.air_m_state   	<< 1) |
							(teleData->accu.air_p_state   	<< 0);

		break;
	}
}



void process_rx_packet(Radio* radio, TelemetryData_t* teleData)
{
	switch(radio->rx_msg[0])
	{
	case 1:
		teleData->accu.over_60v_dclink = (radio->rx_msg[5] >> 7) & 0x01;
		teleData->accu.air_m_state     = (radio->rx_msg[5] >> 6) & 0x01;
		teleData->accu.air_m_supp      = (radio->rx_msg[5] >> 5) & 0x01;
		teleData->accu.air_p_state     = (radio->rx_msg[5] >> 4) & 0x01;
		teleData->accu.air_p_supp      = (radio->rx_msg[5] >> 3) & 0x01;
		teleData->accu.precharge_state = (radio->rx_msg[5] >> 2) & 0x01;
		teleData->accu.ts_active       = (radio->rx_msg[5] >> 1) & 0x01;
		teleData->accu.vicor_overtemp  = (radio->rx_msg[5] >> 0) & 0x01;

		teleData->accu.max_cell_temp    	=  radio->rx_msg[6];
		teleData->accu.min_cell_temp    	=  radio->rx_msg[7];
		teleData->accu.dcdc_temp        	=  radio->rx_msg[8];
		teleData->accu.max_humidity     	=  radio->rx_msg[9];
		teleData->accu.bms_error_code   	=  radio->rx_msg[10];
		teleData->accu.tsac_error_code  	=  radio->rx_msg[11];
		teleData->accu.max_soc       	=  radio->rx_msg[12];
		teleData->accu.total_voltage_vs 	= (radio->rx_msg[13] << 8) | radio->rx_msg[14];
		teleData->accu.accu_current     	= (radio->rx_msg[15] << 8) | radio->rx_msg[16];
		teleData->accu.max_cell_voltage 	= (radio->rx_msg[17] << 8) | radio->rx_msg[18];
		teleData->accu.min_cell_voltage 	= (radio->rx_msg[19] << 8) | radio->rx_msg[20];
		teleData->accu.wh_consumed      	= (radio->rx_msg[21] << 8) | radio->rx_msg[22];
		teleData->accu.ah_consumed          = (radio->rx_msg[23] << 8) | radio->rx_msg[24];
		teleData->accu.error_position       =  radio->rx_msg[25];
		teleData->accu.max_cell_voltage_pos =  radio->rx_msg[26];
		teleData->accu.min_cell_voltage_pos =  radio->rx_msg[27];

		teleData->pdu.max_cell_temp    =  radio->rx_msg[28];
		teleData->pdu.max_cell_voltage = (radio->rx_msg[29] << 8) | radio->rx_msg[30];
		teleData->pdu.min_cell_voltage = (radio->rx_msg[31] << 8) | radio->rx_msg[32];
		teleData->pdu.tdk1_current     = (radio->rx_msg[33] << 8) | radio->rx_msg[34];
		teleData->pdu.tdk2_current     = (radio->rx_msg[35] << 8) | radio->rx_msg[36];
		teleData->pdu.wh_consumed      = (radio->rx_msg[37] << 8) | radio->rx_msg[38];

		teleData->accu.last_bms_error_code  = radio->rx_msg[39];
		teleData->accu.last_tsac_error_code = radio->rx_msg[40];

		teleData->inv.inv_enable          = (radio->rx_msg[41] >> 7) & 0x01;
		teleData->inv.inv_ok              = (radio->rx_msg[41] >> 6) & 0x01;
		teleData->inv.curr_lim_reached    = (radio->rx_msg[41] >> 5) & 0x01;
		teleData->inv.curr_lim_igbt_temp  = (radio->rx_msg[41] >> 4) & 0x01;
		teleData->inv.curr_lim_motor_temp = (radio->rx_msg[41] >> 3) & 0x01;
		teleData->inv.curr_peak_val_warn  = (radio->rx_msg[41] >> 2) & 0x01;
		teleData->inv.defective_param     = (radio->rx_msg[41] >> 1) & 0x01;
		teleData->inv.hardware_fault      = (radio->rx_msg[41] >> 0) & 0x01;

		teleData->inv.faulty_safety_circuit    = (radio->rx_msg[42] >> 7) & 0x01;
		teleData->inv.can_time_out_exceeded    = (radio->rx_msg[42] >> 6) & 0x01;
		teleData->inv.bad_encoder_signal 	   = (radio->rx_msg[42] >> 5) & 0x01;
		teleData->inv.no_power_supply_voltage  = (radio->rx_msg[42] >> 4) & 0x01;
		teleData->inv.overvoltage 			   = (radio->rx_msg[42] >> 3) & 0x01;
		teleData->inv.overcurrent 			   = (radio->rx_msg[42] >> 2) & 0x01;
		teleData->inv.current_measure_fault    = (radio->rx_msg[42] >> 1) & 0x01;
		teleData->inv.ballast_circuit_overload = (radio->rx_msg[42] >> 0) & 0x01;

		teleData->inv.faulty_run_signal_emi			  = (radio->rx_msg[43] >> 5) & 0x01;
		teleData->inv.inactive_rfe					  = (radio->rx_msg[43] >> 4) & 0x01;
		teleData->inv.power_supply_missing_or_too_low = (radio->rx_msg[43] >> 3) & 0x01;
		teleData->inv.output_voltage_limit_reached    = (radio->rx_msg[43] >> 2) & 0x01;
		teleData->inv.overcurrent_200 				  = (radio->rx_msg[43] >> 1) & 0x01;
		teleData->inv.ballast_circuit_overload_87	  = (radio->rx_msg[43] >> 0) & 0x01;

		teleData->pdu.pi_enabled   = (radio->rx_msg[44] >> 6) & 0x01;
		teleData->pdu.dac2_enabled = (radio->rx_msg[44] >> 5) & 0x01;
		teleData->pdu.dac1_enabled = (radio->rx_msg[44] >> 4) & 0x01;
		teleData->pdu.overcurrent  = (radio->rx_msg[44] >> 3) & 0x01;
		teleData->pdu.overtemp 	   = (radio->rx_msg[44] >> 2) & 0x01;
		teleData->pdu.overvoltage  = (radio->rx_msg[44] >> 1) & 0x01;
		teleData->pdu.undervoltage = (radio->rx_msg[44] >> 0) & 0x01;
		teleData->pdu.distribution_ratio = (radio->rx_msg[45] << 8) | radio->rx_msg[46];

		teleData->accu.min_soc = radio->rx_msg[47];

		/* latches the error from VCU and requires power-cycling to clear */
		if (teleData->vcu.mission_pc_error == 0)
			teleData->vcu.mission_pc_error = (radio->rx_msg[48] >> 7) & 0x01;

		if (teleData->vcu.air_m_error == 0)
			teleData->vcu.air_m_error = (radio->rx_msg[48] >> 6) & 0x01;

		if (teleData->vcu.air_p_error == 0)
			teleData->vcu.air_p_error = (radio->rx_msg[48] >> 5) & 0x01;

		if (teleData->vcu.pc_pressed_error == 0)
			teleData->vcu.pc_pressed_error = (radio->rx_msg[48] >> 4) & 0x01;

		if (teleData->vcu.imd_ams_bspd_error == 0)
			teleData->vcu.imd_ams_bspd_error = (radio->rx_msg[48] >> 3) & 0x01;

		if (teleData->vcu.airs_opened_error == 0)
			teleData->vcu.airs_opened_error = (radio->rx_msg[48] >> 2) & 0x01;

		teleData->vcu.precharge_done     = (radio->rx_msg[48] >> 1) & 0x01;
		teleData->vcu.r2d_done 		     = (radio->rx_msg[48] >> 0) & 0x01;

		teleData->vcu.power_limiter_active  = (radio->rx_msg[49] >> 5) & 0x01;
		teleData->vcu.software_bspd_enabled = (radio->rx_msg[49] >> 4) & 0x01;
		teleData->vcu.brake_implausibility  = (radio->rx_msg[49] >> 3) & 0x01;
		teleData->vcu.brake_deviation       = (radio->rx_msg[49] >> 2) & 0x01;
		teleData->vcu.apps_implausibility   = (radio->rx_msg[49] >> 1) & 0x01;
		teleData->vcu.apps_deviaton 	    = (radio->rx_msg[49] >> 0) & 0x01;
		break;

	case 2:
		teleData->vcu.brake_front	   =  radio->rx_msg[6];
		teleData->vcu.brake_rear 	   =  radio->rx_msg[7];
		teleData->vcu.actual_torque    = (radio->rx_msg[8]  << 8) | radio->rx_msg[9];
		teleData->vcu.requested_torque = (radio->rx_msg[10] << 8) | radio->rx_msg[11];
		teleData->vcu.motor_rpm 	   = (radio->rx_msg[12] << 8) | radio->rx_msg[13];
		teleData->vcu.apps1    		   = (radio->rx_msg[14] << 8) | radio->rx_msg[15];
		teleData->vcu.apps2   		   = (radio->rx_msg[16] << 8) | radio->rx_msg[17];
		teleData->vcu.hall_fr 		   = (radio->rx_msg[18] << 8) | radio->rx_msg[19];
		teleData->vcu.hall_fl 		   = (radio->rx_msg[20] << 8) | radio->rx_msg[21];
		teleData->vcu.hall_rr 		   = (radio->rx_msg[22] << 8) | radio->rx_msg[23];
		teleData->vcu.hall_rl 		   = (radio->rx_msg[24] << 8) | radio->rx_msg[25];

		teleData->accu.imd_isolation_kOhms = (radio->rx_msg[26] << 8) | radio->rx_msg[27];

		teleData->dash.r2d_pressed       = (radio->rx_msg[30] >> 7) & 1;
		teleData->dash.pc_combo          = (radio->rx_msg[30] >> 6) & 1;
		teleData->dash.aux2_pressed      = (radio->rx_msg[30] >> 5) & 1;
		teleData->dash.regen_on          = (radio->rx_msg[30] >> 4) & 1;
		teleData->dash.aero_fans_on      = (radio->rx_msg[30] >> 3) & 1;
		teleData->dash.power_limiter   =  radio->rx_msg[31];
		teleData->dash.traction_def    =  radio->rx_msg[32];
		teleData->dash.mission         =  radio->rx_msg[33];
		teleData->dash.mission_locked  = (radio->rx_msg[34] >> 0) & 0x01;

		teleData->ccu.cooling_fan_on			= (radio->rx_msg[36] >> 5) & 0x01;
		teleData->ccu.pumps_on 					= (radio->rx_msg[36] >> 4) & 0x01;
		teleData->ccu.motor_temp_too_high       = (radio->rx_msg[36] >> 3) & 0x01;
		teleData->ccu.device_temp_too_high      = (radio->rx_msg[36] >> 2) & 0x01;
		teleData->ccu.motor_temp_limit_reached  = (radio->rx_msg[36] >> 1) & 0x01;
		teleData->ccu.device_temp_limit_reached = (radio->rx_msg[36] >> 0) & 0x01;
		teleData->ccu.igbt_temp 				=  radio->rx_msg[37];
		teleData->ccu.motor_temp 				=  radio->rx_msg[38];

		teleData->plex.accel_long = (radio->rx_msg[40] << 8) | radio->rx_msg[41];
		teleData->plex.accel_lat  = (radio->rx_msg[42] << 8) | radio->rx_msg[43];
		teleData->plex.gps_speed  = (radio->rx_msg[44] << 8) | radio->rx_msg[45];
		teleData->ccu.water_temp0 =  radio->rx_msg[46];
		teleData->ccu.water_temp1 =  radio->rx_msg[47];
		break;

	case 3:
		teleData->ir.infrared_fr = (radio->rx_msg[5]  << 8) | radio->rx_msg[6];
		teleData->ir.infrared_fl = (radio->rx_msg[7]  << 8) | radio->rx_msg[8];
		teleData->ir.infrared_rr = (radio->rx_msg[9]  << 8) | radio->rx_msg[10];
		teleData->ir.infrared_rl = (radio->rx_msg[11] << 8) | radio->rx_msg[12];

		teleData->ccu.linear_rr = (radio->rx_msg[13] << 8) | radio->rx_msg[14];
		teleData->ccu.linear_rl = (radio->rx_msg[15] << 8) | radio->rx_msg[16];

		teleData->asb.manual_begin 			= (radio->rx_msg[17] >> 5) & 0x01;
		teleData->asb.manual_asms_off 		= (radio->rx_msg[17] >> 4) & 0x01;
		teleData->asb.manual_mission_error  = (radio->rx_msg[17] >> 3) & 0x01;
		teleData->asb.manual_asms_off_error = (radio->rx_msg[17] >> 2) & 0x01;
		teleData->asb.vcu_alive 			= (radio->rx_msg[17] >> 1) & 0x01;
		teleData->asb.accu_alive 			= (radio->rx_msg[17] >> 0) & 0x01;

		teleData->tpms.fr_temp 	= (radio->rx_msg[18] << 8) | radio->rx_msg[19];
		teleData->tpms.fr_press	= (radio->rx_msg[20] << 8) | radio->rx_msg[21];
		teleData->tpms.fl_temp	= (radio->rx_msg[22] << 8) | radio->rx_msg[23];
		teleData->tpms.fl_press	= (radio->rx_msg[24] << 8) | radio->rx_msg[25];
		teleData->tpms.rr_temp	= (radio->rx_msg[26] << 8) | radio->rx_msg[27];
		teleData->tpms.rr_press = (radio->rx_msg[28] << 8) | radio->rx_msg[29];
		teleData->tpms.rl_temp  = (radio->rx_msg[30] << 8) | radio->rx_msg[31];
		teleData->tpms.rl_press = (radio->rx_msg[32] << 8) | radio->rx_msg[33];

		teleData->vcu.bspd_ok	         = (radio->rx_msg[34] >> 7) & 1;
		teleData->vcu.asb_ok 	         = (radio->rx_msg[34] >> 6) & 1;
		teleData->accu.imd_ok  	         = (radio->rx_msg[34] >> 4) & 1;
		teleData->accu.ams_ok  	         = (radio->rx_msg[34] >> 3) & 1;
		teleData->dash.bots_sd_closed    = (radio->rx_msg[34] >> 2) & 1;
		teleData->dash.cockpit_sd_closed = (radio->rx_msg[34] >> 1) & 1;
		teleData->dash.inertia_sd_closed = (radio->rx_msg[34] >> 0) & 1;

		teleData->vcu.sd_closed		     = (radio->rx_msg[35] >> 4) & 1;
		teleData->asb.right_sd_closed    = (radio->rx_msg[35] >> 3) & 1;
		teleData->asb.left_sd_closed     = (radio->rx_msg[35] >> 2) & 1;
		teleData->accu.air_m_state       = (radio->rx_msg[35] >> 1) & 1;
		teleData->accu.air_p_state	     = (radio->rx_msg[35] >> 0) & 1;
		break;
	}
}



cJSON* Make_Packet(cJSON* packet,const char* name, double value)
{
    char value_name[60];
    cJSON *value_Json = NULL;
    value_Json = cJSON_CreateNumber((double)(value));
    if (value_Json == NULL) return packet;
    sprintf(value_name, name);
    cJSON_AddItemToObject(packet, value_name, value_Json);

    return packet;
}



USBD_StatusTypeDef Transmit_USB (Radio* radio, TelemetryData_t* teleData, uint8_t id)
{
    USBD_StatusTypeDef usb_result;
    char *string = NULL;
    cJSON *Json_packet = NULL;
    Json_packet = cJSON_CreateObject();
    if (Json_packet == NULL)
        goto end;

    switch(id)
    {
    case 1:
        Json_packet = Make_Packet(Json_packet, "accu_over_60v_dclink", teleData->accu.over_60v_dclink);
        Json_packet = Make_Packet(Json_packet, "accu_air_m_state",     teleData->accu.air_m_state);
        Json_packet = Make_Packet(Json_packet, "accu_air_m_supp",	   teleData->accu.air_m_supp);
        Json_packet = Make_Packet(Json_packet, "accu_air_p_state",     teleData->accu.air_p_state);
        Json_packet = Make_Packet(Json_packet, "accu_air_p_supp", 	   teleData->accu.air_p_supp);
        Json_packet = Make_Packet(Json_packet, "accu_precharge_state", teleData->accu.precharge_state);
        Json_packet = Make_Packet(Json_packet, "accu_ts_active", 	   teleData->accu.ts_active);
        Json_packet = Make_Packet(Json_packet, "accu_vicor_overtemp",  teleData->accu.vicor_overtemp);
        Json_packet = Make_Packet(Json_packet, "accu_max_cell_temp",   (double)(teleData->accu.max_cell_temp / 2.0));
        Json_packet = Make_Packet(Json_packet, "accu_min_cell_temp",   (double)(teleData->accu.min_cell_temp / 2.0));
        Json_packet = Make_Packet(Json_packet, "accu_dcdc_temp",       (double)(teleData->accu.dcdc_temp / 2.0));
        Json_packet = Make_Packet(Json_packet, "accu_max_humidity", 	teleData->accu.max_humidity);
        Json_packet = Make_Packet(Json_packet, "accu_bms_error_code",   teleData->accu.bms_error_code);
        Json_packet = Make_Packet(Json_packet, "accu_tsac_error_code",  teleData->accu.tsac_error_code);
        Json_packet = Make_Packet(Json_packet, "accu_max_soc",	 	    teleData->accu.max_soc);
        Json_packet = Make_Packet(Json_packet, "accu_total_voltage_vs", (double)(teleData->accu.total_voltage_vs / 100.0));
        Json_packet = Make_Packet(Json_packet, "accu_accu_current",     (double)(teleData->accu.accu_current     / 100.0));
        Json_packet = Make_Packet(Json_packet, "accu_max_cell_voltage", (double)(teleData->accu.max_cell_voltage / 10000.0));
        Json_packet = Make_Packet(Json_packet, "accu_min_cell_voltage", (double)(teleData->accu.min_cell_voltage / 10000.0));
        Json_packet = Make_Packet(Json_packet, "accu_wh_consumed", 	    teleData->accu.wh_consumed);
        Json_packet = Make_Packet(Json_packet, "accu_ah_consumed",      (double)(teleData->accu.ah_consumed / 1000.0));

        Json_packet = Make_Packet(Json_packet, "accu_error_position",   	teleData->accu.error_position);
        Json_packet = Make_Packet(Json_packet, "accu_min_cell_voltage_pos", teleData->accu.min_cell_voltage_pos);
        Json_packet = Make_Packet(Json_packet, "accu_max_cell_voltage_pos", teleData->accu.max_cell_voltage_pos);

        // NEW
        Json_packet = Make_Packet(Json_packet, "accu_last_bms_error_code",  teleData->accu.last_bms_error_code);
        Json_packet = Make_Packet(Json_packet, "accu_last_tsac_error_code", teleData->accu.last_tsac_error_code);
        Json_packet = Make_Packet(Json_packet, "accu_power",                (double)((teleData->accu.total_voltage_vs / 100.0) * (teleData->accu.accu_current / 100.0) / 1000.0));

        Json_packet = Make_Packet(Json_packet, "pdu_max_cell_temp",    teleData->pdu.max_cell_temp);
        Json_packet = Make_Packet(Json_packet, "pdu_max_cell_voltage", (double)(teleData->pdu.max_cell_voltage / 10000.0));
        Json_packet = Make_Packet(Json_packet, "pdu_min_cell_voltage", (double)(teleData->pdu.min_cell_voltage / 10000.0));
        Json_packet = Make_Packet(Json_packet, "pdu_tdk1_current",     (double)(teleData->pdu.tdk1_current / 1000.0));
        Json_packet = Make_Packet(Json_packet, "pdu_tdk2_current",     (double)(teleData->pdu.tdk2_current / 1000.0));
        Json_packet = Make_Packet(Json_packet, "pdu_wh_consumed",      (double)(teleData->pdu.wh_consumed  / 100.0));
        Json_packet = Make_Packet(Json_packet, "pdu_max_cell_temp",    teleData->pdu.max_cell_temp);
        Json_packet = Make_Packet(Json_packet, "pdu_pi_enabled",   		 teleData->pdu.pi_enabled);
        Json_packet = Make_Packet(Json_packet, "pdu_dac2_enabled", 		 teleData->pdu.dac2_enabled);
        Json_packet = Make_Packet(Json_packet, "pdu_dac1_enabled", 		 teleData->pdu.dac1_enabled);
        Json_packet = Make_Packet(Json_packet, "pdu_overcurrent",  		 teleData->pdu.overcurrent);
        Json_packet = Make_Packet(Json_packet, "pdu_overtemp", 	   		 teleData->pdu.overtemp);
        Json_packet = Make_Packet(Json_packet, "pdu_overvoltage",  		 teleData->pdu.overvoltage);
        Json_packet = Make_Packet(Json_packet, "pdu_undervoltage", 		 teleData->pdu.undervoltage);
        Json_packet = Make_Packet(Json_packet, "pdu_distribution_ratio", (double)(teleData->pdu.distribution_ratio / 10000.0));

        Json_packet = Make_Packet(Json_packet, "accu_min_soc", teleData->accu.min_soc);

        Json_packet = Make_Packet(Json_packet, "inv_inv_enable",          			  teleData->inv.inv_enable);
        Json_packet = Make_Packet(Json_packet, "inv_inv_ok",              	 		  teleData->inv.inv_ok);
        Json_packet = Make_Packet(Json_packet, "inv_curr_lim_reached",    	 		  teleData->inv.curr_lim_reached);
        Json_packet = Make_Packet(Json_packet, "inv_curr_lim_igbt_temp",  	 		  teleData->inv.curr_lim_igbt_temp);
        Json_packet = Make_Packet(Json_packet, "inv_curr_lim_motor_temp", 	 		  teleData->inv.curr_lim_motor_temp);
        Json_packet = Make_Packet(Json_packet, "inv_curr_peak_val_warn",  	 		  teleData->inv.curr_peak_val_warn);
        Json_packet = Make_Packet(Json_packet, "inv_defective_param",     	 		  teleData->inv.defective_param);
        Json_packet = Make_Packet(Json_packet, "inv_hardware_fault",          		  teleData->inv.hardware_fault);
        Json_packet = Make_Packet(Json_packet, "inv_faulty_safety_circuit",   		  teleData->inv.faulty_safety_circuit);
        Json_packet = Make_Packet(Json_packet, "inv_can_time_out_exceeded",   		  teleData->inv.can_time_out_exceeded);
        Json_packet = Make_Packet(Json_packet, "inv_bad_encoder_signal",      		  teleData->inv.bad_encoder_signal);
        Json_packet = Make_Packet(Json_packet, "inv_no_power_supply_voltage", 		  teleData->inv.no_power_supply_voltage);
        Json_packet = Make_Packet(Json_packet, "inv_overvoltage", 			 		  teleData->inv.overvoltage);
        Json_packet = Make_Packet(Json_packet, "inv_overcurrent", 			 		  teleData->inv.overcurrent);
        Json_packet = Make_Packet(Json_packet, "inv_current_measure_fault",   		  teleData->inv.current_measure_fault);
        Json_packet = Make_Packet(Json_packet, "inv_ballast_circuit_overload", 		  teleData->inv.ballast_circuit_overload);
        Json_packet = Make_Packet(Json_packet, "inv_faulty_run_signal_emi", 	      teleData->inv.faulty_run_signal_emi);
        Json_packet = Make_Packet(Json_packet, "inv_inactive_rfe", 					  teleData->inv.inactive_rfe);
        Json_packet = Make_Packet(Json_packet, "inv_power_supply_missing_or_too_low", teleData->inv.power_supply_missing_or_too_low);
        Json_packet = Make_Packet(Json_packet, "inv_output_voltage_limit_reached",    teleData->inv.output_voltage_limit_reached);
        Json_packet = Make_Packet(Json_packet, "inv_overcurrent_200", 				  teleData->inv.overcurrent_200);
        Json_packet = Make_Packet(Json_packet, "inv_ballast_circuit_overload_87", 	  teleData->inv.ballast_circuit_overload_87);

        Json_packet = Make_Packet(Json_packet, "vcu_mission_pc_error",   teleData->vcu.mission_pc_error);
        Json_packet = Make_Packet(Json_packet, "vcu_air_m_error",   	 teleData->vcu.air_m_error);
        Json_packet = Make_Packet(Json_packet, "vcu_air_p_error",   	 teleData->vcu.air_p_error);
        Json_packet = Make_Packet(Json_packet, "vcu_pc_pressed_error",   teleData->vcu.pc_pressed_error);
        Json_packet = Make_Packet(Json_packet, "vcu_imd_asb_bspd_error", teleData->vcu.imd_ams_bspd_error);
        Json_packet = Make_Packet(Json_packet, "vcu_airs_opened_error",  teleData->vcu.airs_opened_error);
        Json_packet = Make_Packet(Json_packet, "vcu_precharge_done",     teleData->vcu.precharge_done);
        Json_packet = Make_Packet(Json_packet, "vcu_r2d_done",           teleData->vcu.r2d_done);
        Json_packet = Make_Packet(Json_packet, "vcu_power_limiter_active",  teleData->vcu.power_limiter_active);
        Json_packet = Make_Packet(Json_packet, "vcu_software_bspd_enabled", teleData->vcu.software_bspd_enabled);
        Json_packet = Make_Packet(Json_packet, "vcu_brake_implausibility",  teleData->vcu.brake_implausibility);
        Json_packet = Make_Packet(Json_packet, "vcu_brake_deviation",       teleData->vcu.brake_deviation);
        Json_packet = Make_Packet(Json_packet, "vcu_apps_implausibility",   teleData->vcu.apps_implausibility);
        Json_packet = Make_Packet(Json_packet, "vcu_apps_deviaton",         teleData->vcu.apps_deviaton);
        break;

    case 2:
        //Json_packet = Make_Packet(Json_packet, "vcu_as_status", 	   teleData->vcu.as_status);
        Json_packet = Make_Packet(Json_packet, "vcu_brake_front", 	   teleData->vcu.brake_front);
        Json_packet = Make_Packet(Json_packet, "vcu_brake_rear", 	   teleData->vcu.brake_rear);
        Json_packet = Make_Packet(Json_packet, "vcu_actual_torque",    teleData->vcu.actual_torque);
        Json_packet = Make_Packet(Json_packet, "vcu_requested_torque", teleData->vcu.requested_torque);
        Json_packet = Make_Packet(Json_packet, "vcu_motor_rpm", 	   teleData->vcu.motor_rpm);
        Json_packet = Make_Packet(Json_packet, "vcu_apps1", 		   teleData->vcu.apps1);
        Json_packet = Make_Packet(Json_packet, "vcu_apps2",			   teleData->vcu.apps2);
        Json_packet = Make_Packet(Json_packet, "vcu_hall_fr",		   teleData->vcu.hall_fr);
        Json_packet = Make_Packet(Json_packet, "vcu_hall_fl",		   teleData->vcu.hall_fl);
        Json_packet = Make_Packet(Json_packet, "vcu_hall_rr",		   teleData->vcu.hall_rr);
        Json_packet = Make_Packet(Json_packet, "vcu_hall_rl",		   teleData->vcu.hall_rl);
        Json_packet = Make_Packet(Json_packet, "accu_imd_isolation_kOhms", teleData->accu.imd_isolation_kOhms);
        Json_packet = Make_Packet(Json_packet, "dash_r2d_pressed",	  	   teleData->dash.r2d_pressed);
        Json_packet = Make_Packet(Json_packet, "dash_aux2_pressed",   	   teleData->dash.aux2_pressed);
        Json_packet = Make_Packet(Json_packet, "dash_regen_on", 	  	   teleData->dash.regen_on);
        Json_packet = Make_Packet(Json_packet, "dash_aero_fans_on",   	   teleData->dash.aero_fans_on);
        Json_packet = Make_Packet(Json_packet, "dash_pc_combo",    		   teleData->dash.pc_combo);
        Json_packet = Make_Packet(Json_packet, "dash_mission_locked",      teleData->dash.mission_locked);
        Json_packet = Make_Packet(Json_packet, "dash_power_limiter",       teleData->dash.power_limiter);
        Json_packet = Make_Packet(Json_packet, "dash_traction_def",        teleData->dash.traction_def);
        Json_packet = Make_Packet(Json_packet, "dash_mission", 		       teleData->dash.mission);
        Json_packet = Make_Packet(Json_packet, "ccu_cooling_fan_on",       teleData->ccu.cooling_fan_on);
        Json_packet = Make_Packet(Json_packet, "ccu_pumps_on",	 	 		    teleData->ccu.pumps_on);
        Json_packet = Make_Packet(Json_packet, "ccu_motor_temp_too_high", 	    teleData->ccu.motor_temp_too_high);
        Json_packet = Make_Packet(Json_packet, "ccu_device_temp_too_high",	    teleData->ccu.device_temp_too_high);
        Json_packet = Make_Packet(Json_packet, "ccu_motor_temp_limit_reached",  teleData->ccu.motor_temp_limit_reached);
        Json_packet = Make_Packet(Json_packet, "ccu_device_temp_limit_reached", teleData->ccu.device_temp_limit_reached);
        Json_packet = Make_Packet(Json_packet, "ccu_igbt_temp", 			    teleData->ccu.igbt_temp);
        Json_packet = Make_Packet(Json_packet, "ccu_motor_temp",				teleData->ccu.motor_temp);
        Json_packet = Make_Packet(Json_packet, "ccu_water_temp0",	       		teleData->ccu.water_temp0);
        Json_packet = Make_Packet(Json_packet, "ccu_water_temp1",	       		teleData->ccu.water_temp1);
        Json_packet = Make_Packet(Json_packet, "dash_mission", 		       		teleData->dash.mission);
        Json_packet = Make_Packet(Json_packet, "plex_accel_long",			    teleData->plex.accel_long);
        Json_packet = Make_Packet(Json_packet, "plex_accel_lat",				teleData->plex.accel_lat);
        Json_packet = Make_Packet(Json_packet, "plex_gps_speed",				teleData->plex.gps_speed);
        break;

    case 3:
        Json_packet = Make_Packet(Json_packet, "ir_infrared_fr", teleData->ir.infrared_fr);
        Json_packet = Make_Packet(Json_packet, "ir_infrared_fl", teleData->ir.infrared_fl);
        Json_packet = Make_Packet(Json_packet, "ir_infrared_rr", teleData->ir.infrared_rr);
        Json_packet = Make_Packet(Json_packet, "ir_infrared_rl", teleData->ir.infrared_rl);
        Json_packet = Make_Packet(Json_packet, "ccu_linear_rr",   (double)(teleData->ccu.linear_rr / 100.0));
        Json_packet = Make_Packet(Json_packet, "ccu_linear_rl",   (double)(teleData->ccu.linear_rl / 100.0));
        Json_packet = Make_Packet(Json_packet, "ccu_water_temp0", teleData->ccu.water_temp0);
        Json_packet = Make_Packet(Json_packet, "ccu_water_temp1", teleData->ccu.water_temp1);
        Json_packet = Make_Packet(Json_packet, "asb_manual_begin",          teleData->asb.manual_begin);
        Json_packet = Make_Packet(Json_packet, "asb_manual_asms_off",       teleData->asb.manual_asms_off);
        Json_packet = Make_Packet(Json_packet, "asb_manual_mission_error",  teleData->asb.manual_mission_error);
        Json_packet = Make_Packet(Json_packet, "asb_manual_asms_off_error", teleData->asb.manual_asms_off_error);
        Json_packet = Make_Packet(Json_packet, "asb_vcu_alive",			    teleData->asb.vcu_alive);
        Json_packet = Make_Packet(Json_packet, "asb_accu_alive",            teleData->asb.accu_alive);
        Json_packet = Make_Packet(Json_packet, "tpms_fr_temp",  (double)(teleData->tpms.fr_temp  / 100.0));
        Json_packet = Make_Packet(Json_packet, "tpms_fr_press", (double)(teleData->tpms.fr_press / 100.0));
        Json_packet = Make_Packet(Json_packet, "tpms_fl_temp",  (double)(teleData->tpms.fl_temp  / 100.0));
        Json_packet = Make_Packet(Json_packet, "tpms_fr_press", (double)(teleData->tpms.fl_press / 100.0));
        Json_packet = Make_Packet(Json_packet, "tpms_rr_temp",  (double)(teleData->tpms.rr_temp  / 100.0));
        Json_packet = Make_Packet(Json_packet, "tpms_fr_press", (double)(teleData->tpms.rr_press / 100.0));
        Json_packet = Make_Packet(Json_packet, "tpms_rl_temp",  (double)(teleData->tpms.rl_temp  / 100.0));
        Json_packet = Make_Packet(Json_packet, "tpms_fr_press", (double)(teleData->tpms.rl_press / 100.0));
        Json_packet = Make_Packet(Json_packet, "vcu_bspd_ok", teleData->vcu.bspd_ok);
        Json_packet = Make_Packet(Json_packet, "vcu_asb_ok",  teleData->vcu.asb_ok);
        Json_packet = Make_Packet(Json_packet, "accu_imd_ok",  teleData->accu.imd_ok);
        Json_packet = Make_Packet(Json_packet, "accu_ams_ok",  teleData->accu.ams_ok);
        Json_packet = Make_Packet(Json_packet, "dash_bots_sd_closed",    teleData->dash.bots_sd_closed);
        Json_packet = Make_Packet(Json_packet, "dash_cockpit_sd_closed", teleData->dash.cockpit_sd_closed);
        Json_packet = Make_Packet(Json_packet, "dash_inertia_sd_closed", teleData->dash.inertia_sd_closed);
        Json_packet = Make_Packet(Json_packet, "vcu_sd_closed",    	    teleData->vcu.sd_closed);
        Json_packet = Make_Packet(Json_packet, "asb_right_sd_closed", teleData->asb.right_sd_closed);
        Json_packet = Make_Packet(Json_packet, "asb_left_sd_closed",  teleData->asb.left_sd_closed);
        Json_packet = Make_Packet(Json_packet, "accu_air_m_state", teleData->accu.air_m_state);
        Json_packet = Make_Packet(Json_packet, "accu_air_p_state", teleData->accu.air_p_state);
        break;
    }

    Json_packet = Make_Packet(Json_packet, "radio_rssi",        radio->packetStatus.Gfsk.RssiAvg);
    Json_packet = Make_Packet(Json_packet, "radio_packet_loss", radio->per.packet_loss);
    Json_packet = Make_Packet(Json_packet, "radio_wrong_crc",   radio->wrongCRC);

    string = cJSON_PrintUnformatted(Json_packet);
    char newline[] = "\r\n";
    if (string == NULL)
    {
        cJSON_free(string);
    }
    else{
        usb_result = CDC_Transmit_FS((uint8_t*)string, strlen(string));
        cJSON_free(string);
        HAL_Delay(2);
        usb_result = CDC_Transmit_FS((uint8_t*)newline, strlen(newline));
    }
    end:
    cJSON_Delete(Json_packet);

    return usb_result;
}
