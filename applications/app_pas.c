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
#include "digital_filter.h"
#include <stdio.h>
#include "commands.h"
#include "terminal.h"
#include "buffer.h"

// Settings
#define PEDAL_INPUT_TIMEOUT				0.2
#define MAX_MS_WITHOUT_CADENCE			2500
#define MIN_MS_WITHOUT_POWER			500

// Function pointers
static void(* volatile send_func_sample)(unsigned char *data, unsigned int len) = 0;

// Threads
static THD_FUNCTION(pas_thread, arg);
static THD_WORKING_AREA(pas_thread_wa, 512);
static THD_WORKING_AREA(sample_send_thread_wa, 512);
static THD_FUNCTION(sample_send_thread, arg);
static thread_t *sample_send_tp;

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
float pedal_torque_prev1 = 0;
float pedal_torque_prev2 = 0;
float pedal_torque_filter;
float pedal_torque_filter_prev1 = 0;
float pedal_torque_filter_prev2 = 0;
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

// Sampling variables
#define ADC_SAMPLE_MAX_LEN		2000
__attribute__((section(".ram4"))) static volatile int16_t m_debug1_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_debug2_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_debug3_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_debug4_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_debug5_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_debug6_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_debug7_samples[ADC_SAMPLE_MAX_LEN];

static volatile int m_sample_len;
static volatile int m_sample_int;
static volatile bool m_sample_raw;
static volatile debug_sampling_mode m_sample_mode;
static volatile debug_sampling_mode m_sample_mode_last;
static volatile int m_sample_now;
static volatile int m_sample_trigger;

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
		"[Field Number: 1-pedTrq, 2-pedTrqFlt, 3-pedRpm, 4-output, 5-threadTicks] [Sample Count]",
		terminal_sample);
	terminal_register_command_callback(
		"pas_plot",
		"Output real time values to the experiments graph",
		"[Field Number] [Plot 1-6]",
		terminal_experiment);

	m_sample_len = 1000;
	m_sample_int = 1;
	m_sample_now = 0;
	m_sample_raw = false;
	m_sample_trigger = 0;
	m_sample_mode = DEBUG_SAMPLING_OFF;
	m_sample_mode_last = DEBUG_SAMPLING_OFF;

	chThdCreateStatic(pas_thread_wa, sizeof(pas_thread_wa), NORMALPRIO, pas_thread, NULL);
	chThdCreateStatic(sample_send_thread_wa, sizeof(sample_send_thread_wa), NORMALPRIO - 1, sample_send_thread, NULL);

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

		if (sleep_ticks > nominal_sleep_ticks) {
			sleep_ticks = nominal_sleep_ticks; // maximum
		}
		if (sleep_ticks <= 0) {
			sleep_ticks = 1; // minimum, 1 tick sleep to not block the other threads
		}
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
				const float PAS_UPDATE_RATE_HZ = 100.0f;
				const float LOW_PASS_FILTER_HZ = PAS_UPDATE_RATE_HZ / 4;
				pedal_torque = hw_get_PAS_torque();
				pedal_torque_filter = filter_bw2(config.update_rate_hz, LOW_PASS_FILTER_HZ, pedal_torque, &pedal_torque_prev1, &pedal_torque_prev2, &pedal_torque_filter_prev1, &pedal_torque_filter_prev2);
				pedal_torque_prev2 = pedal_torque_prev1;
				pedal_torque_prev1 = pedal_torque;
				pedal_torque_filter_prev2 = pedal_torque_filter_prev1;
				pedal_torque_filter_prev1 = pedal_torque_filter;
				output = utils_throttle_curve(pedal_torque_filter, -3.0, 0.0, 2) * config.current_scaling * sub_scaling;
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
					ms_without_cadence_or_torque += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
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

		bool sample = false;

		switch (m_sample_mode) {
		case DEBUG_SAMPLING_NOW:
			if (m_sample_now == m_sample_len) {
				m_sample_mode = DEBUG_SAMPLING_OFF;
				m_sample_mode_last = DEBUG_SAMPLING_NOW;
				chSysLockFromISR();
				chEvtSignalI(sample_send_tp, (eventmask_t) 1);
				chSysUnlockFromISR();
			} else {
				sample = true;
			}
			break;

		case DEBUG_SAMPLING_TRIGGER_START:
		case DEBUG_SAMPLING_TRIGGER_START_NOSEND: {
			sample = true;

			int sample_last = -1;
			if (m_sample_trigger >= 0) {
				sample_last = m_sample_trigger - m_sample_len;
				if (sample_last < 0) {
					sample_last += ADC_SAMPLE_MAX_LEN;
				}
			}

			if (m_sample_now == sample_last) {
				m_sample_mode_last = m_sample_mode;
				sample = false;

				if (m_sample_mode == DEBUG_SAMPLING_TRIGGER_START) {
					chSysLockFromISR();
					chEvtSignalI(sample_send_tp, (eventmask_t) 1);
					chSysUnlockFromISR();
				}

				m_sample_mode = DEBUG_SAMPLING_OFF;
			}

			if (config.current_scaling >= 0.3 && m_sample_trigger < 0) { // trigger on PAS5
				m_sample_trigger = m_sample_now;
			}
		} break;

		default:
			break;
		}

		if (sample) {
			static int a = 0;
			if (++a >= m_sample_int) {
				a = 0;

				if (m_sample_now >= ADC_SAMPLE_MAX_LEN) {
					m_sample_now = 0;
				}

				m_debug1_samples[m_sample_now] = (int16_t) (10000 * pedal_torque);
				m_debug2_samples[m_sample_now] = (int16_t) (10000 * pedal_torque_filter);
				m_debug3_samples[m_sample_now] = (int16_t) (10000 * output);
				m_debug4_samples[m_sample_now] = (int16_t) (10000 * pedal_rpm);
				m_debug5_samples[m_sample_now] = (int16_t) (10000 * sin(2 * 3.14 * m_sample_now / 500));
				m_debug6_samples[m_sample_now] = 0;
				m_debug7_samples[m_sample_now] = 0;

				++m_sample_now;
			}
		}

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
		commands_init_plot("Milliseconds", "PAS Debug");
		commands_plot_add_graph("1");
		commands_plot_add_graph("2");
		commands_plot_add_graph("3");
		commands_plot_add_graph("4");
		commands_plot_add_graph("5");
		commands_plot_add_graph("6");
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
			return pedal_rpm;
		case(4):
			return output;
		case(5):
			return thread_ticks;

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

float mc_interface_get_last_sample_adc_isr_duration(void) {
	return 0;
}

void mc_interface_sample_print_data(debug_sampling_mode mode, uint16_t len, uint8_t decimation, bool raw, 
		void(*reply_func)(unsigned char *data, unsigned int len)) {

	if (len > ADC_SAMPLE_MAX_LEN) {
		len = ADC_SAMPLE_MAX_LEN;
	}

	if (mode == DEBUG_SAMPLING_SEND_LAST_SAMPLES) {
		chEvtSignal(sample_send_tp, (eventmask_t) 1);
	} else {
		m_sample_trigger = -1;
		m_sample_now = 0;
		m_sample_len = len;
		m_sample_int = decimation;
		m_sample_mode = mode;
		m_sample_raw = raw;
		send_func_sample = reply_func;
	}
}

static THD_FUNCTION(sample_send_thread, arg) {
	(void)arg;

	chRegSetThreadName("SampleSender");
	sample_send_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		int len = 0;
		int offset = 0;

		switch (m_sample_mode_last) {
		case DEBUG_SAMPLING_NOW:
		case DEBUG_SAMPLING_START:
			len = m_sample_len;
			break;

		case DEBUG_SAMPLING_TRIGGER_START:
		case DEBUG_SAMPLING_TRIGGER_FAULT:
		case DEBUG_SAMPLING_TRIGGER_START_NOSEND:
		case DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND:
			len = ADC_SAMPLE_MAX_LEN;
			offset = m_sample_trigger - m_sample_len;
			break;

		default:
			break;
		}

		for (int i = 0;i < len;i++) {
			uint8_t buffer[40];
			int32_t index = 0;
			int ind_samp = i + offset;

			while (ind_samp >= ADC_SAMPLE_MAX_LEN) {
				ind_samp -= ADC_SAMPLE_MAX_LEN;
			}

			while (ind_samp < 0) {
				ind_samp += ADC_SAMPLE_MAX_LEN;
			}

			buffer[index++] = COMM_SAMPLE_PRINT;

			buffer_append_float32_auto(buffer, (float) m_debug6_samples[ind_samp] / 10000, &index);	// curr0
			buffer_append_float32_auto(buffer, (float) m_debug7_samples[ind_samp] / 10000, &index);	// curr1
			buffer_append_float32_auto(buffer, (float) m_debug1_samples[ind_samp] / 10000, &index);	// ph1
			buffer_append_float32_auto(buffer, (float) m_debug2_samples[ind_samp] / 10000, &index);	// ph2
			buffer_append_float32_auto(buffer, (float) m_debug3_samples[ind_samp] / 10000, &index);	// ph3
			buffer_append_float32_auto(buffer, (float) m_debug4_samples[ind_samp] / 10000, &index);	// vzero
			buffer_append_float32_auto(buffer, (float) m_debug5_samples[ind_samp] / 10000, &index);	// curr_fir
			buffer_append_float32_auto(buffer, (float) 500, &index);								// f_sw
			buffer[index++] = 0;																	// status
			buffer[index++] = 0;																	// phase

			send_func_sample(buffer, index);
		}
	}
}
