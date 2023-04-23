/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se
	Copyright 2020 Marcos Chaparro	mchaparro@powerdesigns.ca

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>
#include <stdio.h>
#include "commands.h"
#include "terminal.h"

// Settings
#define PEDAL_INPUT_TIMEOUT				0.2
#define MAX_MS_WITHOUT_CADENCE			2500
#define MIN_MS_WITHOUT_POWER			500

// Threads
static THD_FUNCTION(pas_thread, arg);
static THD_WORKING_AREA(pas_thread_wa, 512);

// Private variables
static volatile pas_config config;
static volatile float sub_scaling = 1;
static volatile float output_current_rel = 0;
static volatile float ms_without_power = 0;
static volatile float max_pulse_period = 0;
static volatile float min_pedal_period = 0;
static volatile float direction_conf = 0;
static volatile float pedal_rpm = 0;
static volatile bool primary_output = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
float pedal_torque;
float pedal_torque_filter;
float output = 0;
systime_t thread_ticks = 0, thread_t0 = 0, thread_t1 = 0; // used to get more consistent loop rate

// Debug values
static int debug_sample_field, debug_sample_count, debug_sample_index;
static int debug_experiment_1, debug_experiment_2, debug_experiment_3, debug_experiment_4, debug_experiment_5, debug_experiment_6;

// Function Prototypes
static void terminal_sample(int argc, const char **argv);
static void terminal_experiment(int argc, const char **argv);
static float debug_get_field(int index);
static void debug_sample(void);
static void debug_experiment(void);

/**
 * Configure and initialize PAS application
 *
 * @param conf
 * App config
 */
void app_pas_configure(pas_config *conf) {
	config = *conf;
	ms_without_power = 0.0;
	output_current_rel = 0.0;

	// a period longer than this should immediately reduce power to zero
	max_pulse_period = 1.0 / ((config.pedal_rpm_start / 60.0) * config.magnets) * 1.2;

	// if pedal spins at x3 the end rpm, assume its beyond limits
	min_pedal_period = 1.0 / ((config.pedal_rpm_end * 3.0 / 60.0));

	(config.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);
}

/**
 * Start PAS thread
 *
 * @param is_primary_output
 * True when PAS app takes direct control of the current target,
 * false when PAS app shares control with the ADC app for current command
 */
void app_pas_start(bool is_primary_output) {
	stop_now = false;

	// Register terminal commands
	terminal_register_command_callback(
		"pas_sample",
		"Output real time values to the terminal",
		"[Field Number: 1-pedTrq, 2-pedTrqFlt, 3-output, 4-pedRpm, 5-threadTicks, 6-trqDt, 7-shutdownFlt] [Sample Count]",
		terminal_sample);
	terminal_register_command_callback(
		"pas_plot",
		"Output real time values to the experiments graph",
		"[Field Number] [Plot 1-6]",
		terminal_experiment);

	chThdCreateStatic(pas_thread_wa, sizeof(pas_thread_wa), NORMALPRIO, pas_thread, NULL);

	primary_output = is_primary_output;
}

bool app_pas_is_running(void) {
	return is_running;
}

void app_pas_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(2);
	}

	if (primary_output == true) {
		mc_interface_set_current_rel(0.0);
	}
	else {
		output_current_rel = 0.0;
	}
}

void app_pas_set_current_sub_scaling(float current_sub_scaling) {
	sub_scaling = current_sub_scaling;
}

float app_pas_get_current_target_rel(void) {
	return output_current_rel;
}

float app_pas_get_pedal_rpm(void) {
	return pedal_rpm;
}

void pas_event_handler(void) {
#ifdef HW_PAS1_PORT
	const int8_t QEM[] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; // Quadrature Encoder Matrix
	int8_t direction_qem;
	uint8_t new_state;
	static uint8_t old_state = 0;
	static float old_timestamp = 0;
	static float inactivity_time = 0;
	static float period_filtered = 0;
	static int32_t correct_direction_counter = 0;

	uint8_t PAS1_level = palReadPad(HW_PAS1_PORT, HW_PAS1_PIN);
	uint8_t PAS2_level = palReadPad(HW_PAS2_PORT, HW_PAS2_PIN);

	new_state = PAS2_level * 2 + PAS1_level;
	direction_qem = (float) QEM[old_state * 4 + new_state];
	old_state = new_state;

	// Require several quadrature events in the right direction to prevent vibrations from
	// engging PAS
	int8_t direction = (direction_conf * direction_qem);
	
	switch(direction) {
		case 1: correct_direction_counter++; break;
		case -1:correct_direction_counter = 0; break;
	}

	const float timestamp = (float) chVTGetSystemTime() / CH_CFG_ST_FREQUENCY;

	// sensors are poorly placed, so use only one rising edge as reference
	if( (new_state == 3) && (correct_direction_counter >= 4) ) {
		float period = (timestamp - old_timestamp) * config.magnets;
		old_timestamp = timestamp;

		UTILS_LP_FAST(period_filtered, period, 1.0); // 1.0 is unfiltered

		if(period_filtered < min_pedal_period) { //can't be that short, abort
			return;
		}
		pedal_rpm = 60.0 / period_filtered;
		pedal_rpm *= (direction_conf * (float) direction_qem);
		inactivity_time = 0.0;
	}
	else {
		inactivity_time += 1.0 / config.update_rate_hz;

		//if no pedal activity, set RPM as zero
		if(inactivity_time > max_pulse_period) {
			pedal_rpm = 0.0;
		}
	}
#endif
}

static THD_FUNCTION(pas_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_PAS");

#ifdef HW_PAS1_PORT
	palSetPadMode(HW_PAS1_PORT, HW_PAS1_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_PAS2_PORT, HW_PAS2_PIN, PAL_MODE_INPUT_PULLUP);
#endif

	is_running = true;

	for(;;) {
		// Sleep for a time according to the specified rate
		thread_ticks = thread_t1 - thread_t0;
		const systime_t nominal_sleep_ticks = CH_CFG_ST_FREQUENCY / config.update_rate_hz;
		systime_t sleep_ticks = nominal_sleep_ticks - thread_ticks;
		sleep_ticks = MIN(sleep_ticks, nominal_sleep_ticks); // Limit max to nominal
		sleep_ticks = MAX(sleep_ticks, nominal_sleep_ticks / 2); // Limit min to nominal / 2
		chThdSleep(sleep_ticks);

		if (stop_now) {
			is_running = false;
			return;
		}

		thread_t0 = chVTGetSystemTime();

		pas_event_handler(); // this could happen inside an ISR instead of being polled

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			ms_without_power = 0;
		}

		if (app_is_output_disabled()) {
			continue;
		}

		switch (config.ctrl_type) {
			case PAS_CTRL_TYPE_NONE:
				output = 0.0;
				break;
			case PAS_CTRL_TYPE_CADENCE:
				// Map pedal rpm to assist level

				// NOTE: If the limits are the same a numerical instability is approached, so in that case
				// just use on/off control (which is what setting the limits to the same value essentially means).
				if (config.pedal_rpm_end > (config.pedal_rpm_start + 1.0)) {
					output = utils_map(pedal_rpm, config.pedal_rpm_start, config.pedal_rpm_end, 0.0, config.current_scaling * sub_scaling);
					utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);
				} else {
					if (pedal_rpm > config.pedal_rpm_end) {
						output = config.current_scaling * sub_scaling;
					} else {
						output = 0.0;
					}
				}
				break;

#ifdef HW_HAS_PAS_TORQUE_SENSOR
			case PAS_CTRL_TYPE_TORQUE:
			{
				pedal_torque = hw_get_pedal_torque();
				float dt = 1.0 / config.update_rate_hz; // update_rate_hz is int
				float pedal_cadence_rpm = 70; // 1.1s or 70 rpm works well for long climbs as well as short technical sections
				//float pedal_cadence_rpm = 180.0 - 120.0 / 1.2 * config.ramp_time_pos; // Maps (1.5..0.3 s) to (30..150 rpm)
				float lpf_hz = pedal_cadence_rpm / 60 / 1.5;
				float rc = 1.0 / (2.0 * M_PI * lpf_hz);
				float lpf_constant = dt / (dt + rc);
				UTILS_LP_FAST(pedal_torque_filter, pedal_torque, lpf_constant);
				output = utils_throttle_curve(pedal_torque_filter, -3.5, 0.0, 2) * config.current_scaling * sub_scaling;
				utils_truncate_number(&output, 0.0, config.current_scaling * sub_scaling);
			}
			/* fall through */
			case PAS_CTRL_TYPE_TORQUE_WITH_CADENCE_TIMEOUT:
			{
				// disable assistance if torque has been sensed for >5sec without any pedal movement. Prevents
				// motor overtemps when the rider is just resting on the pedals
				static float ms_without_cadence_or_torque = 0.0;
				if(output == 0.0 || pedal_rpm > 0) {
					ms_without_cadence_or_torque = 0.0;
				} else {
					ms_without_cadence_or_torque += 1000.0 / config.update_rate_hz;
					if(ms_without_cadence_or_torque > MAX_MS_WITHOUT_CADENCE) {
						output = 0.0;
					}
				}
			}
#endif
			default:
				break;
		}

		// Apply ramping
		static systime_t last_time = 0;
		static float output_ramp = 0.0;
		float ramp_time = fabsf(output) > fabsf(output_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&output_ramp, output, ramp_step);
			utils_truncate_number(&output_ramp, 0.0, config.current_scaling * sub_scaling);

			last_time = chVTGetSystemTime();
			output = output_ramp;
		}

		if (output < 0.001) {
			ms_without_power += 1000.0 / config.update_rate_hz;
		}

		// Safe start is enabled if the output has not been zero for long enough
		if (ms_without_power < MIN_MS_WITHOUT_POWER) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			output_current_rel = 0.0;
			continue;
		}

		// Reset timeout
		timeout_reset();

		if (primary_output == true) {
			mc_interface_set_current_rel(output);
		}
		else {
			output_current_rel = output;
		}

		// Debug outputs
		debug_sample();
		debug_experiment();

		thread_t1 = chVTGetSystemTime();
	}
}

// Terminal commands
static void terminal_sample(int argc, const char **argv) {
	if (argc == 3) {
		debug_sample_field = 0;
		debug_sample_count = 0;
		sscanf(argv[1], "%d", &debug_sample_field);
		sscanf(argv[2], "%d", &debug_sample_count);
		debug_sample_index = 0;
	} else {
		commands_printf("This command requires two arguments.\n");
	}
}

static void terminal_experiment(int argc, const char **argv) {
	static bool initialized = false;
	if (argc == 3) {
		int field = 0;
		int graph = 1;
		sscanf(argv[1], "%d", &field);
		sscanf(argv[2], "%d", &graph);
		switch(graph){
			case (1):
				debug_experiment_1 = field;
				break;
			case (2):
				debug_experiment_2 = field;
				break;
			case (3):
				debug_experiment_3 = field;
				break;
			case (4):
				debug_experiment_4 = field;
				break;
			case (5):
				debug_experiment_5 = field;
				break;
			case (6):
				debug_experiment_6 = field;
				break;
		}
		if( initialized ){
			commands_init_plot("Milliseconds", "PAS Debug");
			commands_plot_add_graph("1");
			commands_plot_add_graph("2");
			commands_plot_add_graph("3");
			commands_plot_add_graph("4");
			commands_plot_add_graph("5");
			commands_plot_add_graph("6");
			initialized = true;
		}
	} else {
		commands_printf("This command requires two arguments.\n");
	}
}

// Debug functions
static float debug_get_field(int index) {
	switch(index){
		case(1):
			return pedal_torque;
		case(2):
			return pedal_torque_filter;
		case(3):
			return output;
		case(4):
			return pedal_rpm;
		case(5):
			return thread_ticks;
		case(6):
			return hw_get_torque_dt();
		case(7):
			return hw_luna_m600_shutdown_button_value();

		default:
			return 0;
	}
}

static void debug_sample() {
	if(debug_sample_index < debug_sample_count){
		commands_printf("%f", (double) debug_get_field(debug_sample_field));
		++debug_sample_index;
	}
}

static void debug_experiment() {
	if(debug_experiment_1 != 0) {
		commands_plot_set_graph(0);
		commands_send_plot_points(ST2MS(thread_t0), debug_get_field(debug_experiment_1));
	}
	if(debug_experiment_2 != 0) {
		commands_plot_set_graph(1);
		commands_send_plot_points(ST2MS(thread_t0), debug_get_field(debug_experiment_2));
	}
	if(debug_experiment_3 != 0) {
		commands_plot_set_graph(2);
		commands_send_plot_points(ST2MS(thread_t0), debug_get_field(debug_experiment_3));
	}
	if(debug_experiment_4 != 0) {
		commands_plot_set_graph(3);
		commands_send_plot_points(ST2MS(thread_t0), debug_get_field(debug_experiment_4));
	}
	if(debug_experiment_5 != 0) {
		commands_plot_set_graph(4);
		commands_send_plot_points(ST2MS(thread_t0), debug_get_field(debug_experiment_5));
	}
	if(debug_experiment_6 != 0) {
		commands_plot_set_graph(5);
		commands_send_plot_points(ST2MS(thread_t0), debug_get_field(debug_experiment_6));
	}
}
