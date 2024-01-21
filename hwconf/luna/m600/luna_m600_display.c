/*
	Copyright 2020 Marcos Chaparro	mchaparro@powerdesigns.ca
	Copyright 2021 Maximiliano Cordoba	mcordoba@powerdesigns.ca

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "conf_general.h"
#ifdef HW_HAS_LUNA_CANBUS_DISPLAY
#include "hw.h"
#include "luna_m600_display.h"
#include "app.h"
#include "ch.h"
#include "hal.h"
#include "packet.h"
#include "commands.h"
#include "mc_interface.h"
#include "utils.h"
#include <math.h>
#include <string.h>
#include "comm_can.h"
#include "shutdown.h"
#include "datatypes.h"
#include "timeout.h"

#include "mcpwm_foc.h" // for encoder angle error

#define LUNA_TORQUE_SENSOR_MINIMUM_TORQUE	0x02EE
#define LUNA_TORQUE_SENSOR_MAXIMUM_TORQUE	0x0C80

typedef enum {
	PAS_LEVEL_0 = 0x00,
	PAS_LEVEL_1 = 0x01,
	PAS_LEVEL_2 = 0x0B,
	PAS_LEVEL_3 = 0x0C,
	PAS_LEVEL_4 = 0x0D,
	PAS_LEVEL_5 = 0x02,
	PAS_LEVEL_6 = 0x15,
	PAS_LEVEL_7 = 0x16,
	PAS_LEVEL_8 = 0x17,
	PAS_LEVEL_9 = 0x03,
	PAS_LEVEL_WALK = 0x06,
} LUNA_PAS_LEVEL;

typedef enum {
	LUNA_HMI_LEVEL_ID = 0x03106300,
	LUNA_TORQUE_SENSOR_ID = 0x01F83100,
	LUNA_HMI_RQST_HW_LEN_ID = 0x13116000,
	LUNA_HMI_RQST_HW_DATA_ID = 0x13126000,
	LUNA_HMI_RQST_SW_LEN_ID = 0x13116001,
	LUNA_HMI_RQST_SW_DATA_ID = 0x13126001,
	LUNA_HMI_RQST_SN_LEN_ID = 0x13116003,
	LUNA_HMI_RQST_SN_DATA_ID = 0x13126003,
} LUNA_CAN_IDs;

typedef enum {
	LUNA_HMI_LEVEL_ID_BYTES = 4,
	LUNA_TORQUE_SENSOR_ID_BYTES = 4,
} LUNA_CAN_IDs_BYTES;

typedef enum {
	LUNA_LIGHT_MODE_OFF = 0,
	LUNA_LIGHT_MODE_ON = 1,
	LUNA_LIGHT_MODE_TURNING_OFF = 2,
	LUNA_LIGHT_MODE_TURNING_ON = 3,
} LUNA_LIGHT_CONTROL_MODE;

typedef enum {
	LUNA_ERROR_NONE = 0x00,
	LUNA_ERROR_BRAKES = 0x03,
	LUNA_ERROR_THROTTLE = 0x05,
	LUNA_ERROR_UNDER_VOLTAGE = 0x06,
	LUNA_ERROR_HIGH_VOLTAGE = 0x07,
	LUNA_ERROR_ENCODER = 0x08,
	LUNA_ERROR_MOTOR_OVERTEMP = 0x10,
	LUNA_ERROR_MOSFET_OVERTEMP = 0x11,
	LUNA_ERROR_CURRENT_SENSOR = 0x12,
	LUNA_ERROR_BATTERY_TEMPERATURE = 0x13,
	LUNA_ERROR_WHEEL_SPEED_DETECTION = 0x21,
	LUNA_ERROR_BMS_COMMUNICATION = 0x22,
	LUNA_ERROR_TORQUE_SENSOR = 0x25,
	LUNA_ERROR_SPEED_SENSOR = 0x26,
	LUNA_ERROR_COMMUNICATION = 0x30,
} LUNA_ERROR_CODES;

typedef enum{
	STATE_BATTERY_RANGE,
	STATE_SPEED_CURRENT_VOLTAGE,
	STATE_REFRESH,
	STATE_SPEED_LIMIT_WHEEL_SIZE,
	STATE_CALORIES,
	STATE_FAULTS,
} can_display_process_states_t;

typedef struct{
	uint32_t interval_ms;
	uint32_t time_since_ms;
} state_schedule_t;

typedef struct{
	LUNA_LIGHT_CONTROL_MODE light_mode;
	LUNA_PAS_LEVEL pas_level;
	LUNA_ERROR_CODES error_code;
	bool torque_sensor_is_active;
	float pedal_torque_norm;
	uint8_t assist_code;
} luna_settings_t;

static volatile luna_settings_t luna_settings =	{	.light_mode = LUNA_LIGHT_MODE_OFF,
													.pas_level = PAS_LEVEL_0,
													.error_code = LUNA_ERROR_NONE,
													.torque_sensor_is_active = false,
													.pedal_torque_norm = 0.0
												};
static volatile bool display_thread_is_running = false;
static volatile bool display_uart_is_running = false;

// Threads
static THD_WORKING_AREA(display_process_thread_wa, 1024);
static THD_FUNCTION(display_process_thread, arg);

static bool check_light_mode(uint8_t light_mode);
static bool check_assist_level(uint8_t assist_code);
static void set_assist_level(uint8_t assist_code);

/**
 * Initialize the display and torque sensor for M600 drive units
 */
void luna_canbus_start(void) {
	if (!display_thread_is_running) {
		chThdCreateStatic(display_process_thread_wa, sizeof(display_process_thread_wa),
				NORMALPRIO, display_process_thread, NULL);
		display_thread_is_running = true;
	}
}

/**
 * Get torque applied to the crank arms
 *
 * @return
 * 0.0 for no torque applied, 1.0 for maximum torque applied
 */
float luna_get_pedal_torque(void){
	return luna_settings.pedal_torque_norm;
}

/**
 * Get the current Pedal Assist level
 *
 * @return
 * Assist level from 0 (min) to 9 (max power). 
 */
LUNA_PAS_LEVEL luna_canbus_get_pas_level(void){
	return luna_settings.pas_level;
}

/**
 * checks if the light mode is valid
 *
 * @param light_mode
 * Parameter to check
 *
 * @return
 * true if valid, false if not.
 */
static bool check_light_mode(uint8_t light_mode) {
	bool ret = false;

	switch (light_mode) {
		case LUNA_LIGHT_MODE_OFF:
		case LUNA_LIGHT_MODE_ON:
		case LUNA_LIGHT_MODE_TURNING_OFF:
		case LUNA_LIGHT_MODE_TURNING_ON:
			ret = true;
			break;
		default:
			break;
	}
	return ret;
}

/**
 * Check if the assist code corresponds to a current level
 *
 * @param assist_code
 * Parameter to check
 *
 * @return
 * true if valid, false if not.
 */
static bool check_assist_level(uint8_t assist_code) {
	bool ret = false;

	switch (assist_code) {
		case PAS_LEVEL_0:
		case PAS_LEVEL_1:
		case PAS_LEVEL_2:
		case PAS_LEVEL_3:
		case PAS_LEVEL_4:
		case PAS_LEVEL_5:
		case PAS_LEVEL_6:
		case PAS_LEVEL_7:
		case PAS_LEVEL_8:
		case PAS_LEVEL_9:
		case PAS_LEVEL_WALK:
			ret = true;
			break;
		default:
			break;
	}

	return ret;
}

/**
 * Set the PAS level according to the assist code
 *
 * @param assist_code
 * Assist level to apply
 */
static void set_assist_level(uint8_t assist_code) {
	float current_scale;
	volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();
	
	luna_settings.assist_code = assist_code;

	// DPC245 has the following mapping
	// W	PAS_LEVEL_WALK
	// 0	PAS_LEVEL_0
	// E	PAS_LEVEL_2
	// T	PAS_LEVEL_4
	// S	PAS_LEVEL_6
	// S+	PAS_LEVEL_8
	// B	PAS_LEVEL_9
	switch (assist_code) {
		case PAS_LEVEL_0: 		current_scale = 0.0; break;
		case PAS_LEVEL_1: 		current_scale = 1.0 / 9.0; break;
		case PAS_LEVEL_2: 		current_scale = 2.0 / 9.0; break;
		case PAS_LEVEL_3: 		current_scale = 2.5 / 9.0; break;
		case PAS_LEVEL_4: 		current_scale = 3.0 / 9.0; break;
		case PAS_LEVEL_5: 		current_scale = 3.5 / 9.0; break;
		case PAS_LEVEL_6: 		current_scale = 4.0 / 9.0; break;
		case PAS_LEVEL_7: 		current_scale = 5.0 / 9.0; break;
		case PAS_LEVEL_8: 		current_scale = 6.0 / 9.0; break;
		case PAS_LEVEL_9: 		current_scale = 1.0; break;
		case PAS_LEVEL_WALK:	current_scale = 1.0; break;
		default: return;
	}

	if( hw_m600_has_fixed_throttle_level() ) {
		mcconf->l_current_max_scale = 1.0;
		app_pas_set_current_sub_scaling(current_scale);
	} else {
		mcconf->l_current_max_scale = current_scale;
	}

	// In level 0, both PAS and throttle should be disabled
	if(current_scale == 0.0) {
		mcconf->l_current_max_scale = current_scale;
	}
}

float distance;
float time_since_display_change=0;
uint16_t distance_display;

uint32_t delay_between_torque_sensor_message = 0;
uint32_t luna_get_torque_dt(void){
	return delay_between_torque_sensor_message;
}

/**
 * State machine for display functions and error handling  
 *
 * @param dt_ms
 * Time since last call to this function
 */
static void can_bus_display_process(uint32_t dt_ms, state_schedule_t * state_schedule, uint8_t n_states){
	static uint8_t can_tx_buffer[8];
	volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();

	//check if torque sensor is active or not
	if( luna_settings.torque_sensor_is_active ){
		luna_settings.torque_sensor_is_active = false;
		delay_between_torque_sensor_message = 0;
		if(	luna_settings.error_code == LUNA_ERROR_TORQUE_SENSOR){
			luna_settings.error_code = LUNA_ERROR_NONE;
		}
	} else {
		//fault if sensor data stops for >500ms, but also allow 3sec for sensor to boot
		delay_between_torque_sensor_message += dt_ms;
		float uptime = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		if( delay_between_torque_sensor_message > 500 && uptime > 3.0 ){
			delay_between_torque_sensor_message = 0;
			luna_settings.error_code = LUNA_ERROR_TORQUE_SENSOR;
		}
	}

	if( luna_settings.error_code == LUNA_ERROR_NONE ){
		//check if vesc has got any faults
		mc_fault_code vesc_fault = mc_interface_get_fault();
		switch( vesc_fault ){
			case FAULT_CODE_NONE:{
				break;
			}
			case FAULT_CODE_ABS_OVER_CURRENT:
			case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1:
			case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2:
			case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3:
			case FAULT_CODE_UNBALANCED_CURRENTS:{
				luna_settings.error_code = LUNA_ERROR_CURRENT_SENSOR;
				break;
			}
			case FAULT_CODE_OVER_TEMP_FET: {
				luna_settings.error_code = LUNA_ERROR_MOSFET_OVERTEMP;
				break;
			}
			case FAULT_CODE_OVER_TEMP_MOTOR:{
				luna_settings.error_code = LUNA_ERROR_MOTOR_OVERTEMP;
				break;
			}
			case FAULT_CODE_OVER_VOLTAGE:{
				luna_settings.error_code = LUNA_ERROR_HIGH_VOLTAGE;
				break;
			}
			case FAULT_CODE_UNDER_VOLTAGE:{
				luna_settings.error_code = LUNA_ERROR_UNDER_VOLTAGE;
				break;
			}
			case FAULT_CODE_ENCODER_SPI:
			case FAULT_CODE_ENCODER_NO_MAGNET:{
				luna_settings.error_code = LUNA_ERROR_ENCODER;
				break;
			}
			default:
				;//luna_settings.error_code = LUNA_ERROR_COMMUNICATION;
		}
	}

	if( mc_interface_get_configuration()->foc_encoder_offset == 400.0 ) {
		luna_settings.error_code = LUNA_ERROR_ENCODER;
	}

	// increment state counters
	for(int i = 0; i < n_states; ++i) {
		state_schedule[i].time_since_ms += dt_ms;
	}

	// select most overdue state (or none)
	int32_t state_selected = -1;
	int32_t past_due_ms = -1;
	for(int i = 0; i < n_states; ++i) {
		int32_t state_past_due_ms = state_schedule[i].time_since_ms - state_schedule[i].interval_ms;
		if (state_past_due_ms > past_due_ms) {
			state_selected = i;
			past_due_ms = state_past_due_ms;
		}
	}
	state_schedule[state_selected].time_since_ms = 0;

	// execute state
	memset(can_tx_buffer, 0, 8);
	switch( state_selected ){
		case -1:{
			break;
		}
		case STATE_BATTERY_RANGE:{
			float wh_left;
			float battery_level = mc_interface_get_battery_level(&wh_left) * 100.0;
			utils_truncate_number(&battery_level, 0.0, 100.0);

			static float last_distance = 0.0;
			float distance_abs = mc_interface_get_distance_abs();
			distance = distance_abs - last_distance;
			
			// what is sent is 1 / 10 of the value in meters
			distance_display = (uint16_t) (distance / 10.0);

			static uint16_t time_at_last_distance_change = 0;
			static uint16_t last_display_distance = 0;

			// if distance_display doesn't change for 5 seconds, we have to reset it to zero
			if ( distance_display != last_display_distance ) {
				time_at_last_distance_change = chVTGetSystemTimeX() / (float) CH_CFG_ST_FREQUENCY;
			} else {
				float current_time = chVTGetSystemTimeX() / (float) CH_CFG_ST_FREQUENCY;

				time_since_display_change = current_time - time_at_last_distance_change;//for debug
				if( current_time - time_at_last_distance_change > 5.0 ){
					time_at_last_distance_change = chVTGetSystemTimeX() / (float) CH_CFG_ST_FREQUENCY;
					last_distance = distance_abs;
					distance = 0;
					distance_display = 0;
				}
			}

			last_display_distance = distance_display;
			if( distance_display < 5 ) {
				//make sure the first zero is not skipped. Apparently it's important and needs 4 consecutive zeroes
				//(or maybe its 1 full second of zeroes)
				distance_display = 0;
			}

			static float distance_old = 0;
			if( distance_abs != distance_old ) {
			//	commands_printf("time_since_display_change: %.2f",(double)time_since_display_change);
			//	commands_printf("display_distance: %d",distance_display);
			//	commands_printf("distance: %.2f",(double)distance);
			//	commands_printf("odometer: %.2f\n", (double)distance_abs);
				distance_old = distance_abs;
			}
			// If the distance value exceeds 1000 meters, it overflows back to 0 meter
			if( distance > 1020 ) {
				last_distance += distance;
			}

			can_tx_buffer[0] = (uint8_t) battery_level;
			can_tx_buffer[1] = (uint8_t)(distance_display & 0x00ff);
			//TODO: support RANGE parameter. 0x1FF = 511 sets RANGE as 5.11 km or 3 miles.
			float range = 57.92; // 57.92 km = 35.99 mi
			uint16_t range_display = (uint16_t) (range * 100);
			can_tx_buffer[6] = (uint8_t) (range_display & 0x00);
			can_tx_buffer[7] = (uint8_t) ((range_display >> 8 ) & 0x00ff);
			comm_can_transmit_eid(0x02F83200, can_tx_buffer, 8);
			break;
		}
		case STATE_SPEED_CURRENT_VOLTAGE:{
#ifdef HW_HAS_WHEEL_SPEED_SENSOR
			//float wheel_rpm = hw_get_wheel_rpm();
			//float wheelsize_in_meters = mcconf->si_wheel_diameter / 1000.0;
			//float speed_km_h = (uint16_t)(wheel_rpm * wheelsize_in_meters * 60.0 / 1000.0);

			//uint16_t speed_display = (uint16_t)(speed_km_h * 100.0);//the display needs [km_h * 100]
			uint16_t speed_display = (uint16_t) (mc_interface_get_speed() * 3600.0 / 1000.0 * 100.0);	//the display needs [km_h * 100]

			can_tx_buffer[0] = (uint8_t) (speed_display & 0x00ff);
			can_tx_buffer[1] = (uint8_t) ((speed_display >> 8 ) & 0x00ff);
#endif
			float current = mc_interface_get_tot_current_in_filtered();
			uint16_t current_display = (uint16_t) (current * 100.0);
			current = hw_get_pedal_torque(); // use pedal torque for DPC245 right bar
			current_display = (uint16_t) (current * 1000.0);
			
			float voltage = mc_interface_get_input_voltage_filtered();
			uint16_t voltage_display = voltage * 100;

			can_tx_buffer[2] = (uint8_t) (current_display & 0x00ff);
			can_tx_buffer[3] = (uint8_t) ((current_display >> 8 ) & 0x00ff);
			can_tx_buffer[4] = (uint8_t) (voltage_display & 0x00ff);
			can_tx_buffer[5] = (uint8_t) ((voltage_display >> 8 ) & 0x00ff);
			can_tx_buffer[6] = (uint8_t) (mc_interface_temp_fet_filtered() - 40.0);		// 10°C = 10+40=50(32Hex) = 32
			can_tx_buffer[7] = (uint8_t) (mc_interface_temp_motor_filtered() - 40.0);	// 20°C = 20+40=60(3CHex) = 3C
			comm_can_transmit_eid(0x02F83201, can_tx_buffer, 8);
			break;
		}
		case STATE_REFRESH:{
			comm_can_transmit_eid(0x02FF1203, can_tx_buffer, 1);
			break;
		}
		case STATE_SPEED_LIMIT_WHEEL_SIZE:{
			//"speed limit" parameter [kmh *100]
			float speed_limit = 32.2;// dummy 32.2km/h (20mph)
			uint16_t speed_limit_display = speed_limit * 100;
			// Arbitrary wheel size definitions from the display
			uint16_t wheelsize_display;
			if( mcconf->si_wheel_diameter <= 0.7 ){
				wheelsize_display = 416; //26"
			} else if( mcconf->si_wheel_diameter >= 0.75 ){
				wheelsize_display = 464; //29"
			} else {
				wheelsize_display = 437; //27.5"
			}

			can_tx_buffer[0] = (uint8_t) (speed_limit_display & 0x00ff);
			can_tx_buffer[1] = (uint8_t) ((speed_limit_display >> 8) & 0x00ff);
			can_tx_buffer[2] = (uint8_t) (wheelsize_display & 0x00ff);
			can_tx_buffer[3] = (uint8_t) ((wheelsize_display >> 8) & 0x00ff);
			can_tx_buffer[4] = 182;
			can_tx_buffer[5] = 8;
			comm_can_transmit_eid(0x02F83203, can_tx_buffer, 6);
			break;
		}
		case STATE_CALORIES:{
			//TODO: support "KCAL" parameter in [kmh *100]
			//conversion rate: KCAL = value * 0.621368.
			//For example: 0xFFFF = 65535 sets KCAL as 40722
			comm_can_transmit_eid(0x02F83205, can_tx_buffer, 2);
			break;
		}
		case STATE_FAULTS:{
			can_tx_buffer[0] = luna_settings.error_code;
			comm_can_transmit_eid(0x02FF1200, can_tx_buffer, 1);
			break;
		}
	}
}

static bool can_bus_rx_callback(uint32_t id, uint8_t *data, uint8_t len) {
	static int tx_length = 0;
	bool used_data = false;
	LUNA_CAN_IDs cmd_id = id;

	switch(cmd_id){
		case LUNA_HMI_LEVEL_ID:{
			if( len == LUNA_HMI_LEVEL_ID_BYTES){
				if( data != NULL ){
					used_data = true;

					if( check_assist_level(data[1]) ){
						luna_settings.pas_level = data[1];
						set_assist_level(luna_settings.pas_level);
					}

					if( check_light_mode(data[2]) ){
						luna_settings.light_mode = data[2];
					}
				}
			}
			break;
		}
		case LUNA_TORQUE_SENSOR_ID:{
			if( len == LUNA_TORQUE_SENSOR_ID_BYTES ){
				if( data != NULL ){
					used_data = true;
					luna_settings.torque_sensor_is_active = true;
					uint16_t torque_raw = (((uint16_t)data[1] << 8 ) & 0xff00) | ((uint16_t)data[0] & 0x00ff);
					float torque_norm = ((float)torque_raw - LUNA_TORQUE_SENSOR_MINIMUM_TORQUE) / (LUNA_TORQUE_SENSOR_MAXIMUM_TORQUE - LUNA_TORQUE_SENSOR_MINIMUM_TORQUE);
					utils_truncate_number(&torque_norm, 0, 1);
					luna_settings.pedal_torque_norm = torque_norm;
				}
			}
			break;
		}
		case LUNA_HMI_RQST_HW_LEN_ID:{
			if( len == 0 ){
				tx_length = 7;
				uint8_t tx_data[1];
				tx_data[0] = tx_length;
				comm_can_transmit_eid(0x029C6000, tx_data, 1);
			}
			break;
		}
		case LUNA_HMI_RQST_HW_DATA_ID:{
			if( len == 0 && tx_length > 0 ){
				uint8_t tx_data[8];
				tx_data[0] = 'T';
				tx_data[1] = 'E';
				tx_data[2] = 'S';
				tx_data[3] = 'T';
				tx_data[4] = ' ';
				tx_data[5] = 'H';
				tx_data[6] = 'W';
				comm_can_transmit_eid(0x029E0000, tx_data, tx_length);
			}
			break;
		}
		case LUNA_HMI_RQST_SW_LEN_ID:{
			if( len == 0 ){
				tx_length = 7;
				uint8_t tx_data[1];
				tx_data[0] = tx_length;
				comm_can_transmit_eid(0x029C6001, tx_data, 1);
			}
			break;
		}
		case LUNA_HMI_RQST_SW_DATA_ID:{
			if( len == 0 && tx_length > 0 ){
				uint8_t tx_data[8];
				tx_data[0] = 'T';
				tx_data[1] = 'E';
				tx_data[2] = 'S';
				tx_data[3] = 'T';
				tx_data[4] = ' ';
				tx_data[5] = 'S';
				tx_data[6] = 'W';
				comm_can_transmit_eid(0x029E0000, tx_data, tx_length);
			}
			break;
		}
		case LUNA_HMI_RQST_SN_LEN_ID:{
			if( len == 0 ){
				tx_length = 8;
				uint8_t tx_data[1];
				tx_data[0] = tx_length;
				comm_can_transmit_eid(0x029C6003, tx_data, 1);
			}
			break;
		}
		case LUNA_HMI_RQST_SN_DATA_ID:{
			if( len == 0 && tx_length > 0 ){
				uint8_t tx_data[8];
				tx_data[0] = '0';
				tx_data[1] = '0';
				tx_data[2] = '0';
				tx_data[3] = '0';
				tx_data[4] = '0';
				tx_data[5] = '0';
				tx_data[6] = '0';
				tx_data[7] = '1';
				comm_can_transmit_eid(0x029E0000, tx_data, tx_length);
			}
			break;
		}
	}

	return used_data;
}

// If user long-presses the (-) button, walk mode is engaged.
// Controller will target a specific RPM and hold it until the button is released
// The speed control must have a slow ramp, the allowed current is TBD.
bool luna_display_walk_mode_long_pressed(void) {
	static systime_t time_last_button_down = 0;
	const float long_press_time_seconds = 0.5;	// bafang uses 2 sec, lets make it more responsive

	if( time_last_button_down == 0) {
		time_last_button_down = chVTGetSystemTimeX();
	}

	if(hw_luna_m600_minus_button_down() == false) {
		time_last_button_down = chVTGetSystemTimeX();
	}

	return (UTILS_AGE_S(time_last_button_down) > long_press_time_seconds);
}

// lets try using the sensorless observer to check if the encoder
// offset has been set correctly.
float encoder_error(void) {
	static float angle_diff_filtered = 0.0;
	float angle_diff = 0.0;
	
	// some batches have only 2 current sensors, so better rely only in
	// the phase voltage tracker which runs with no modulation and is
	// accurate at mid-high rpm
	if(/*mc_interface_get_state() != MC_STATE_OFF &&*/ mc_interface_get_duty_cycle_now() > 0.4) {
		angle_diff = utils_angle_difference(mcpwm_foc_get_phase_encoder(), mcpwm_foc_get_phase_observer());
	}

	UTILS_LP_FAST(angle_diff_filtered, angle_diff, 0.01);
	return angle_diff_filtered;
}

static THD_FUNCTION(display_process_thread, arg) {
	(void)arg;
	chRegSetThreadName("Luna CANbus display");

	event_listener_t el;
	chEvtRegisterMaskWithFlags(&HW_UART_DEV.event, &el, EVENT_MASK(0), CHN_INPUT_AVAILABLE);

	// Set default power level
	set_assist_level(PAS_LEVEL_1);

	comm_can_set_eid_rx_callback( can_bus_rx_callback );

	uint8_t n_states = 6;
	state_schedule_t state_schedule[n_states];
	state_schedule[STATE_BATTERY_RANGE]			.time_since_ms	= 0;
	state_schedule[STATE_BATTERY_RANGE]			.interval_ms	= 1500;
	state_schedule[STATE_SPEED_CURRENT_VOLTAGE]	.time_since_ms	= 0;
	state_schedule[STATE_SPEED_CURRENT_VOLTAGE]	.interval_ms	= 200;
	state_schedule[STATE_REFRESH]				.time_since_ms	= 0;
	state_schedule[STATE_REFRESH]				.interval_ms 	= 100;
	state_schedule[STATE_SPEED_LIMIT_WHEEL_SIZE].time_since_ms	= 0;
	state_schedule[STATE_SPEED_LIMIT_WHEEL_SIZE].interval_ms 	= 2000;
	state_schedule[STATE_CALORIES]				.time_since_ms 	= 0;
	state_schedule[STATE_CALORIES]				.interval_ms 	= 3000;
	state_schedule[STATE_FAULTS]				.time_since_ms 	= 0;
	state_schedule[STATE_FAULTS]				.interval_ms 	= 400;

	for(;;) {
		float uptime = chVTGetSystemTime() / (float) CH_CFG_ST_FREQUENCY;
		
		uint32_t dt_ms = 5;
		chThdSleepMilliseconds(dt_ms);
		can_bus_display_process(dt_ms, state_schedule, n_states);

		static bool encoder_recovery_done = false;
		if(encoder_recovery_done) {
			// default motor config is set to an invalid 400° encoder offset. Make clear
			// to the users that they need to run the encoder offset detection
			if(mc_interface_get_configuration()->foc_encoder_offset == 400.0 /*|| fabsf(hw_get_encoder_error()) > 10.0*/) {
				;//mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, false, false);
			}
		} else {
			if (uptime > 1.0 && !encoder_recovery_done) {
				// recover encoder offset across fw updates
				hw_recover_encoder_offset();
				encoder_recovery_done = true;
			}
		}
		// when PAS level set to 0, the system would shut down after 10 minutes of non-assisted pedaling
		// so we force it to stay ON if there is pedal activity
		if(luna_get_pedal_torque() > 0.25) {
			shutdown_reset_timer();
		}

		if (luna_settings.assist_code == PAS_LEVEL_WALK) {
			//disable ADC & PAS apps for 50 millisec
			app_disable_output(50);

			if(luna_display_walk_mode_long_pressed()) {
				//send speed command. Slow 500rpm/sec ramp
				mc_interface_set_pid_speed(2500.0);
				timeout_reset();
			} else {
				//quickly release the motor
				mc_interface_release_motor();
			}
		}
	}
}
#endif
