/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "app_regen_types.h"
#include "app_regen_conf.h"

// Some useful includes
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

// App settings
#define FILTER_SAMPLES					5

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void load_default_config(custom_config_type* conf);
static void load_stored_config(custom_config_type* conf);

static void terminal_set_speed(int argc, const char **argv);
static void terminal_config(int argc, const char **argv);
static void terminal_clutch(int argc, const char **argv);
static void terminal_log(int argc, const char **argv);
static void terminal_cmd_enable_plot(int argc, const char **argv);
static void terminal_cmd_disable_plot(int argc, const char **argv);
static void terminal_cmd_help(int argc, const char **argv);
static void terminal_get_config(int argc, const char **argv);

static void update_pedal_torque(void);
static void update_pedal_speed_and_position(bool reset);
static void update_wheel_speed(void);
static void update_motor_speed(void);
static void update_clutch_state(void);
static void update_motor_control(void);

static void open_clutch(void);
static void sync_clutch(void);
static void close_clutch(void);
static void set_motor_speed(float mwrpm);
static void enable_interrupt(void);
static void init_plots(void);
static void plot_points(plot_index_t plot, float x, float y);
static void print_log(log_group_t log_group, const char* format, ...);
static void apply_ramping(float *value, systime_t *last_time, float target, float ramp_time_pos, float ramp_time_neg);

// Private variables
//// Config variables
static custom_config_type config;
static adc_config config_adc;

static volatile float max_pedal_period = 0.0;
static volatile float min_pedal_period = 0.0;
static volatile float max_wheel_period = 0.0;
static volatile float min_wheel_period = 0.0;
static volatile int32_t min_backward_counter = 0;
static volatile int32_t max_backward_counter = 0;

//// Control variables
static volatile float command_line_speed = -1;

//// State variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float pedal_torque = 0;
static volatile float pedal_torque_rel = 0;
static volatile float pedal_speed  = 0;    //CRPM
static volatile float pedal_speed_rel = 0; 
static volatile float pedal_brake_position = 0;
static volatile float pedal_brake_position_rel = 0;
static volatile float wheel_speed  = 0;    //WRPM
static volatile float wheel_speed_rel = 0;
static volatile float motor_speed  = 0;    //MWRPM
static volatile clutch_state_type clutch_state = CLUTCH_STATE_OPEN;
static volatile uint8_t HALL1_level = 0;
static volatile uint8_t HALL2_level = 0;
static volatile uint8_t HALL3_level = 0;

//// Other variables
static volatile uint32_t log_groups_enabled = 0;
static volatile uint32_t plots_enabled = 0;
static volatile int plot_numbers[PLOT_COUNT] = {0};
static volatile int plot_number = 0;
static volatile float ms_without_power = 0.0;
static volatile float wheel_sensor_timestamp = 0;
static volatile float clutch_timestamp = 0;
static volatile uint8_t clutch_open_error_counter = 0;
static volatile uint8_t clutch_close_error_counter = 0;
static volatile uint32_t HALL3_int_cntr_xp = 0;
static volatile uint32_t HALL3_int_cntr_rt = 0;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
#ifdef APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1
	palSetPadMode(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN1, PAL_MODE_INPUT_PULLUP);
#endif
#ifdef APP_CUSTOM_CONF_PEDAL_SENSOR_PORT2
	palSetPadMode(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT2, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN2, PAL_MODE_INPUT_PULLUP);
#endif

#ifdef APP_CUSTOM_CONF_WHEEL_SENSOR_PORT1
	palSetPadMode(APP_CUSTOM_CONF_WHEEL_SENSOR_PORT1, APP_CUSTOM_CONF_WHEEL_SENSOR_PIN1, PAL_MODE_INPUT_PULLUP);
#endif

#ifdef APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1
	palSetPadMode(APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1, APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1, PAL_MODE_OUTPUT_OPENDRAIN);
#endif

#ifdef APP_CUSTOM_CONF_TORQUE_SENSOR_PORT1
    if (APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE == TORQUE_SENSOR_TYPE_ADC) {
	    palSetPadMode(APP_CUSTOM_CONF_TORQUE_SENSOR_PORT1, APP_CUSTOM_CONF_TORQUE_SENSOR_PIN1, PAL_MODE_INPUT_ANALOG);
	}
#endif

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"set-speed",
			"Set the speed to RPM",
			"[RPM]",
			terminal_set_speed);

	terminal_register_command_callback(
			"config",
			"Configure custom app parameters",
			"[parameter] [value]",
			terminal_config);
	terminal_register_command_callback(
			"clutch",
			"Open or close the clutch",
			"[open/close]",
			terminal_clutch);
	terminal_register_command_callback(
			"log",
			"Enable/disable logging",
			"[log_group]",
			terminal_log);

	terminal_register_command_callback(
	        "enable_plot",
    	    "Enable a plot. Usage: enable_plot <plot_name>",
			"[plot_name]",
        	terminal_cmd_enable_plot);

    terminal_register_command_callback(
        	"disable_plot",
        	"Disable a plot. Usage: disable_plot <plot_name>",
			"[plot_name]",
        	terminal_cmd_disable_plot);

	terminal_register_command_callback(
        	"help2",
        	"List all commands, their usage, and possible arguments",
			"",
        	terminal_cmd_help);

	terminal_register_command_callback(
			"getconfig",
			"Get the current configuration settings",
			"",
			terminal_get_config);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(terminal_set_speed);
	terminal_unregister_callback(terminal_config);
	terminal_unregister_callback(terminal_clutch);
	terminal_unregister_callback(terminal_log);
	terminal_unregister_callback(terminal_cmd_enable_plot);
	terminal_unregister_callback(terminal_cmd_disable_plot);
	terminal_unregister_callback(terminal_cmd_help);
	terminal_unregister_callback(terminal_get_config);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

bool app_custom_is_running(void) {
	return is_running;
}

void app_custom_configure(app_configuration *conf) {
	eeprom_var v;
	
	load_default_config(&config);

	load_stored_config(&config);

	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_PLOTS_ENABLED_ADDR)) {
		plots_enabled = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_LOG_GROUPS_ENABLED_ADDR)) {
		log_groups_enabled = v.as_u32;
	}

	if (config.torque_sensor.sensor_type == TORQUE_SENSOR_TYPE_ADC) {
		config_adc = conf->app_adc_conf;
	}

	ms_without_power = 0.0;

	// a period longer than this should immediately reduce CRPM to zero
	max_pedal_period = 1.0 / ((config.pedal_sensor.rpm_min / 60.0) * config.pedal_sensor.magnets);

	// if pedal spins at max rpm, assume its beyond limits
	min_pedal_period = 1.0 / ((config.pedal_sensor.rpm_max / 60.0) * config.pedal_sensor.magnets);

	// a period longer than this should immediately reduce WRPM to zero
	max_wheel_period = 1.0 / ((config.wheel_sensor.rpm_min / 60.0) * config.wheel_sensor.magnets);

	// if wheel spins at max rpm, assume its beyond limits
	min_wheel_period = 1.0 / ((config.wheel_sensor.rpm_max / 60.0) * config.wheel_sensor.magnets);

	// convert pedal angles to quadrature counter
	min_backward_counter = floor((float)(config.back_pedal_brake.start_pos) / (360.0f / (float)(4.0 * config.pedal_sensor.magnets)));
	max_backward_counter = ceil((float)(config.back_pedal_brake.end_pos) / (360.0f / (float)(4.0 * config.pedal_sensor.magnets)));

	enable_interrupt();
}

void app_custom_pin_isr(void){
	wheel_sensor_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	HALL3_int_cntr_xp++;
	HALL3_int_cntr_rt++;
}

void app_custom_get_rtdata(float* data) {
	data[0] = pedal_speed;
	data[1] = wheel_speed;
	data[2] = motor_speed;
	data[3] = pedal_brake_position;
	data[4] = pedal_torque;
	data[5] = (float)clutch_state;
	//data[5] = HALL3_int_cntr_rt;
	//HALL3_int_cntr_rt = 0;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;
	float timestamp = 0;
	float wheel_inactivity_time = 0;

	chRegSetThreadName("App Custom");

	is_running = true;

	chThdSleepMilliseconds(1000);
	init_plots();

	// put clutch into defined state
	if (config.clutch.mode == CLUTCH_MODE_OPEN) {
		open_clutch();
	} else {
		close_clutch();
	}

	for(int cnt = 0; true; cnt++) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time); //TODO: enable exiting sleep when encoder interrupt happens??

		if (stop_now) {
			is_running = false;
			return;
		}

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			ms_without_power = 0;
		}

		// Reset timeout if everything is OK.
		timeout_reset(); 

		timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

		//measure torque
		update_pedal_torque();

		//measure pedal forward speed or backward position
		update_pedal_speed_and_position(FALSE);

		plot_points(PLOT_PEDAL_RPM, timestamp, pedal_speed);
        plot_points(PLOT_BRAKE_POS, timestamp, pedal_brake_position);

		//measure wheel speed
		update_wheel_speed();

		plot_points(PLOT_WHEEL_RPM, timestamp, wheel_speed);

		//get motor speed
		update_motor_speed();

		plot_points(PLOT_MOTOR_RPM, timestamp, motor_speed);

		//take care of clutch state transitions
		update_clutch_state();

		plot_points(PLOT_CLUTCH_STATE, timestamp, clutch_state == CLUTCH_STATE_OPEN ? 0 : (clutch_state == CLUTCH_STATE_CLOSED ? 30 : (clutch_state == CLUTCH_STATE_OPENING ? 10 : 20)));

		//control motor speed/current according to the current state variables
		update_motor_control();

		//if wheel speed is small then release brake after N seconds
		// note: motor speed is measured here because of the instability of wrpm in interrupt mode
		if (clutch_state == CLUTCH_STATE_CLOSED && motor_speed < config.back_pedal_brake.release_rpm && pedal_brake_position > 0){
			if (wheel_inactivity_time < config.back_pedal_brake.wait_before_release){
				wheel_inactivity_time += 1.0 / (float)config.update_rate_hz;
				if (wheel_inactivity_time >= config.back_pedal_brake.wait_before_release){
					update_pedal_speed_and_position(TRUE);
				}
			}
		} else {
			wheel_inactivity_time = 0;
		}
	}
}

static void load_default_config(custom_config_type* conf){
	conf->ctrl_type                  = APP_CUSTOM_CONF_CTRL_TYPE;

	conf->pedal_sensor.sensor_type   = APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE;
	conf->pedal_sensor.magnets       = APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS;
	conf->pedal_sensor.filter        = APP_CUSTOM_CONF_PEDAL_SENSOR_FILTER;
    conf->pedal_sensor.rpm_min       = APP_CUSTOM_CONF_PEDAL_RPM_MIN;
	conf->pedal_sensor.rpm_start     = APP_CUSTOM_CONF_PEDAL_RPM_START;
	conf->pedal_sensor.rpm_end       = APP_CUSTOM_CONF_PEDAL_RPM_END;
	conf->pedal_sensor.rpm_max       = APP_CUSTOM_CONF_PEDAL_RPM_MAX;
	conf->pedal_sensor.ramp_time_pos = APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS;
	conf->pedal_sensor.ramp_time_neg = APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG;
	conf->pedal_sensor.invert_direction = APP_CUSTOM_CONF_PEDAL_INVERT_DIR;

	conf->wheel_sensor.sensor_type   = APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE;
	conf->wheel_sensor.magnets       = APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS;
	conf->wheel_sensor.filter        = APP_CUSTOM_CONF_WHEEL_SENSOR_FILTER;
	conf->wheel_sensor.rpm_min       = APP_CUSTOM_CONF_WHEEL_RPM_MIN;
	conf->wheel_sensor.rpm_max       = APP_CUSTOM_CONF_WHEEL_RPM_MAX;
	conf->wheel_sensor.ramp_time_pos = APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS;
	conf->wheel_sensor.ramp_time_neg = APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG;
	conf->wheel_sensor.invert_direction = APP_CUSTOM_CONF_WHEEL_INVERT_DIR;

	conf->torque_sensor.sensor_type  = APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE;

	conf->back_pedal_brake.start_pos = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS;
	conf->back_pedal_brake.end_pos   = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS;
	conf->back_pedal_brake.wait_before_release = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_WAIT_BEFORE_RELEASE;
	conf->back_pedal_brake.release_rpm = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RELEASE_RPM;

	conf->clutch.wait_before_open    = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN;
	conf->clutch.wait_before_sync   = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC;
	conf->clutch.wait_before_check   = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK;
	conf->clutch.sync_rpm_diff       = APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF;
	conf->clutch.check_rpm_diff      = APP_CUSTOM_CONF_CLUTCH_CHECK_RPM_DIFF;
	conf->clutch.min_rpm 			 = APP_CUSTOM_CONF_CLUTCH_MIN_RPM;
	conf->clutch.max_rpm_open		 = APP_CUSTOM_CONF_CLUTCH_MAX_RPM_OPEN;
	conf->clutch.max_rpm_close		 = APP_CUSTOM_CONF_CLUTCH_MAX_RPM_CLOSE;
	conf->clutch.mode 				 = APP_CUSTOM_CONF_CLUTCH_MODE;

	conf->update_rate_hz = APP_CUSTOM_CONF_UPDATE_RATE_HZ;
}

static void load_stored_config(custom_config_type* conf){
	eeprom_var v;
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CTRL_TYPE_ADDR)) {
		conf->ctrl_type = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE_ADDR)) {
		conf->pedal_sensor.sensor_type = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS_ADDR)) {
		conf->pedal_sensor.magnets = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_SENSOR_FILTER_ADDR)) {
		conf->pedal_sensor.filter = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RPM_START_ADDR)) {
		conf->pedal_sensor.rpm_start = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RPM_END_ADDR)) {
		conf->pedal_sensor.rpm_end = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_INVERT_DIR_ADDR)) {
		conf->pedal_sensor.invert_direction = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE_ADDR)) {
		conf->wheel_sensor.sensor_type = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS_ADDR)) {
		conf->wheel_sensor.magnets = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_SENSOR_FILTER_ADDR)) {
		conf->wheel_sensor.filter = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_INVERT_DIR_ADDR)) {
		conf->wheel_sensor.invert_direction = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS_ADDR)) {
		conf->back_pedal_brake.start_pos = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS_ADDR)) {
		conf->back_pedal_brake.end_pos = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_WAIT_BEFORE_RELEASE_ADDR)) {
		conf->back_pedal_brake.wait_before_release = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RELEASE_RPM_ADDR)) {
		conf->back_pedal_brake.release_rpm = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN_ADDR)) {
		conf->clutch.wait_before_open = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC_ADDR)) {
		conf->clutch.wait_before_sync = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK_ADDR)) {
		conf->clutch.wait_before_check = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF_ADDR)) {
		conf->clutch.sync_rpm_diff = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_CHECK_RPM_DIFF_ADDR)) {
		conf->clutch.check_rpm_diff = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_UPDATE_RATE_HZ_ADDR)) {
		conf->update_rate_hz = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS_ADDR)) {
		conf->pedal_sensor.ramp_time_pos = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG_ADDR)) {
		conf->pedal_sensor.ramp_time_neg = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS_ADDR)) {
		conf->wheel_sensor.ramp_time_pos = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG_ADDR)) {
		conf->wheel_sensor.ramp_time_neg = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_MIN_RPM_ADDR)) {
		conf->clutch.min_rpm = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_MAX_RPM_OPEN_ADDR)) {
		conf->clutch.max_rpm_open = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_MAX_RPM_CLOSE_ADDR)) {
		conf->clutch.max_rpm_close = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_MODE_ADDR)) {
		conf->clutch.mode = v.as_u32;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RPM_MIN_ADDR)) {
		conf->pedal_sensor.rpm_min = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RPM_MAX_ADDR)) {
		conf->pedal_sensor.rpm_max = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_RPM_MIN_ADDR)) {
		conf->wheel_sensor.rpm_min = v.as_float;
	}
	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_RPM_MAX_ADDR)) {
		conf->wheel_sensor.rpm_max = v.as_float;
	}
}

// Callback function for the terminal command with arguments.
static void terminal_set_speed(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);
		command_line_speed = d;
		commands_printf("RPM set to %d", d);
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

// Callback function for the terminal command with arguments.
static void terminal_config(int argc, const char **argv) {
	eeprom_var v;
    if (argc == 3) {
        if (strcmp(argv[1], "ctrl-type") == 0) {
            if (strcmp(argv[2], "none") == 0) {
                config.ctrl_type = CUSTOM_CTRL_TYPE_NONE;
                commands_printf("Control type set to CUSTOM_CTRL_TYPE_NONE");
            } else if (strcmp(argv[2], "pid") == 0) {
                config.ctrl_type = CUSTOM_CTRL_TYPE_PID;
                commands_printf("Control type set to CUSTOM_CTRL_TYPE_PID");
            } else if (strcmp(argv[2], "speed") == 0) {
                config.ctrl_type = CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED;
                commands_printf("Control type set to CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED");
            } else if (strcmp(argv[2], "torque") == 0) {
                config.ctrl_type = CUSTOM_CTRL_TYPE_CURRENT_PEDAL_TORQUE;
                commands_printf("Control type set to CUSTOM_CTRL_TYPE_CURRENT_PEDAL_TORQUE");
            } else if (strcmp(argv[2], "torque_speed") == 0) {
                config.ctrl_type = CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED_AND_TORQUE;
                commands_printf("Control type set to CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED_AND_TORQUE");
            } else {
                commands_printf("Invalid value.\r\nValid values:\r\n  none\r\n  pid\r\n  speed\r\n  torque\r\n  torque_speed\r\n");
            }
			v.as_u32 = config.ctrl_type;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CTRL_TYPE_ADDR);
		} else if (strcmp(argv[1], "pedal_sensor_type") == 0) {
			if (strcmp(argv[2], "single_poll") == 0) {
			config.pedal_sensor.sensor_type = SPEED_SENSOR_TYPE_SINGLE_POLL;
			commands_printf("Pedal sensor type set to SINGLE_POLL");
			} else if (strcmp(argv[2], "single_int") == 0) {
			config.pedal_sensor.sensor_type = SPEED_SENSOR_TYPE_SINGLE_INTERRUPT;
			commands_printf("Pedal sensor type set to SINGLE_INTERRUPT");
			} else if (strcmp(argv[2], "quad_poll") == 0) {
			config.pedal_sensor.sensor_type = SPEED_SENSOR_TYPE_QUADRATURE_POLL;
			commands_printf("Pedal sensor type set to QUADRATURE_POLL");
			} else if (strcmp(argv[2], "quad_int") == 0) {
			config.pedal_sensor.sensor_type = SPEED_SENSOR_TYPE_QUADRATURE_INTERRUPT;
			commands_printf("Pedal sensor type set to QUADRATURE_INTERRUPT");
			} else {
			commands_printf("Invalid value.\r\nValid values:\r\n  single_poll\r\n  single_int\r\n  quad_poll\r\n  quad_int\r\n");
			}
			v.as_u32 = config.pedal_sensor.sensor_type;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE_ADDR);
        } else if (strcmp(argv[1], "pedal_magnets") == 0) {
            config.pedal_sensor.magnets = atoi(argv[2]);
            commands_printf("Pedal sensor magnets set to %d", config.pedal_sensor.magnets);
			v.as_u32 = config.pedal_sensor.magnets;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS_ADDR);
        } else if (strcmp(argv[1], "pedal_filter") == 0) {
            config.pedal_sensor.filter = atof(argv[2]);
            commands_printf("Pedal sensor filter set to %f", (double)config.pedal_sensor.filter);
			v.as_float = config.pedal_sensor.filter;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_SENSOR_FILTER_ADDR);
        } else if (strcmp(argv[1], "pedal_rpm_start") == 0) {
            config.pedal_sensor.rpm_start = atof(argv[2]);
            commands_printf("Pedal RPM start set to %f", (double)config.pedal_sensor.rpm_start);
			v.as_float = config.pedal_sensor.rpm_start;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RPM_START_ADDR);
        } else if (strcmp(argv[1], "pedal_rpm_end") == 0) {
            config.pedal_sensor.rpm_end = atof(argv[2]);
            commands_printf("Pedal RPM end set to %f", (double)config.pedal_sensor.rpm_end);
			v.as_float = config.pedal_sensor.rpm_end;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RPM_END_ADDR);
        } else if (strcmp(argv[1], "pedal_rpm_min") == 0) {
            config.pedal_sensor.rpm_min = atof(argv[2]);
            commands_printf("Pedal RPM min set to %f", (double)config.pedal_sensor.rpm_min);
			v.as_float = config.pedal_sensor.rpm_min;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RPM_MIN_ADDR);
        } else if (strcmp(argv[1], "pedal_rpm_max") == 0) {
            config.pedal_sensor.rpm_max = atof(argv[2]);
            commands_printf("Pedal RPM max set to %f", (double)config.pedal_sensor.rpm_max);
			v.as_float = config.pedal_sensor.rpm_max;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RPM_MAX_ADDR);
        } else if (strcmp(argv[1], "pedal_invert") == 0) {
            config.pedal_sensor.invert_direction = atoi(argv[2]);
            commands_printf("Pedal sensor invert direction set to %d", config.pedal_sensor.invert_direction);
			v.as_u32 = config.pedal_sensor.invert_direction;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_INVERT_DIR_ADDR);
		} else if (strcmp(argv[1], "wheel_sensor_type") == 0) {
			if (strcmp(argv[2], "single_poll") == 0) {
			config.wheel_sensor.sensor_type = SPEED_SENSOR_TYPE_SINGLE_POLL;
			commands_printf("Wheel sensor type set to SINGLE_POLL");
			} else if (strcmp(argv[2], "single_int") == 0) {
			config.wheel_sensor.sensor_type = SPEED_SENSOR_TYPE_SINGLE_INTERRUPT;
			commands_printf("Wheel sensor type set to SINGLE_INTERRUPT");
			} else if (strcmp(argv[2], "quad_poll") == 0) {
			config.wheel_sensor.sensor_type = SPEED_SENSOR_TYPE_QUADRATURE_POLL;
			commands_printf("Wheel sensor type set to QUADRATURE_POLL");
			} else if (strcmp(argv[2], "quad_int") == 0) {
			config.wheel_sensor.sensor_type = SPEED_SENSOR_TYPE_QUADRATURE_INTERRUPT;
			commands_printf("Wheel sensor type set to QUADRATURE_INTERRUPT");
			} else {
			commands_printf("Invalid value.\r\nValid values:\r\n  single_poll\r\n  single_int\r\n  quad_poll\r\n  quad_int\r\n");
			}
			v.as_u32 = config.wheel_sensor.sensor_type;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE_ADDR);
        } else if (strcmp(argv[1], "wheel_magnets") == 0) {
            config.wheel_sensor.magnets = atoi(argv[2]);
            commands_printf("Wheel sensor magnets set to %d", config.wheel_sensor.magnets);
			v.as_u32 = config.wheel_sensor.magnets;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS_ADDR);
        } else if (strcmp(argv[1], "wheel_filter") == 0) {
            config.wheel_sensor.filter = atof(argv[2]);
            commands_printf("Wheel sensor filter set to %f", (double)config.wheel_sensor.filter);
			v.as_float = config.wheel_sensor.filter;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_SENSOR_FILTER_ADDR);
        } else if (strcmp(argv[1], "wheel_rpm_min") == 0) {
            config.wheel_sensor.rpm_min = atof(argv[2]);
            commands_printf("Wheel RPM min set to %f", (double)config.wheel_sensor.rpm_min);
			v.as_float = config.wheel_sensor.rpm_min;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_RPM_MIN_ADDR);
        } else if (strcmp(argv[1], "wheel_rpm_max") == 0) {
            config.wheel_sensor.rpm_max = atof(argv[2]);
            commands_printf("Wheel RPM max set to %f", (double)config.wheel_sensor.rpm_max);
			v.as_float = config.wheel_sensor.rpm_max;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_RPM_MAX_ADDR);
        } else if (strcmp(argv[1], "wheel_invert") == 0) {
            config.wheel_sensor.invert_direction = atoi(argv[2]);
            commands_printf("Wheel sensor invert direction set to %d", config.wheel_sensor.invert_direction);
			v.as_u32 = config.wheel_sensor.invert_direction;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_INVERT_DIR_ADDR);
        } else if (strcmp(argv[1], "brake_start") == 0) {
            config.back_pedal_brake.start_pos = atof(argv[2]);
            commands_printf("Back pedal brake start position set to %f", (double)config.back_pedal_brake.start_pos);
			v.as_float = config.back_pedal_brake.start_pos;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS_ADDR);
        } else if (strcmp(argv[1], "brake_end") == 0) {
            config.back_pedal_brake.end_pos = atof(argv[2]);
            commands_printf("Back pedal brake end position set to %f", (double)config.back_pedal_brake.end_pos);
			v.as_float = config.back_pedal_brake.end_pos;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS_ADDR);
        } else if (strcmp(argv[1], "brake_wait_release") == 0) {
            config.back_pedal_brake.wait_before_release = atof(argv[2]);
            commands_printf("Back pedal brake wait before release set to %f", (double)config.back_pedal_brake.wait_before_release);
			v.as_float = config.back_pedal_brake.wait_before_release;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_WAIT_BEFORE_RELEASE_ADDR);
        } else if (strcmp(argv[1], "brake_release_rpm") == 0) {
            config.back_pedal_brake.release_rpm = atof(argv[2]);
            commands_printf("Back pedal brake release RPM set to %f", (double)config.back_pedal_brake.release_rpm);
			v.as_float = config.back_pedal_brake.release_rpm;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RELEASE_RPM_ADDR);
        } else if (strcmp(argv[1], "clutch_open") == 0) {
            config.clutch.wait_before_open = atof(argv[2]);
            commands_printf("Clutch wait before open set to %f", (double)config.clutch.wait_before_open);
			v.as_float = config.clutch.wait_before_open;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN_ADDR);
        } else if (strcmp(argv[1], "clutch_sync") == 0) {
            config.clutch.wait_before_sync = atof(argv[2]);
            commands_printf("Clutch wait before sync set to %f", (double)config.clutch.wait_before_sync);
			v.as_float = config.clutch.wait_before_sync;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC_ADDR);
        } else if (strcmp(argv[1], "clutch_check") == 0) {
            config.clutch.wait_before_check = atof(argv[2]);
            commands_printf("Clutch wait before check set to %f", (double)config.clutch.wait_before_check);
			v.as_float = config.clutch.wait_before_check;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK_ADDR);
        } else if (strcmp(argv[1], "clutch_sync_diff") == 0) {
            config.clutch.sync_rpm_diff = atof(argv[2]);
            commands_printf("Clutch sync RPM diff set to %f", (double)config.clutch.sync_rpm_diff);
			v.as_float = config.clutch.sync_rpm_diff;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF_ADDR);
        } else if (strcmp(argv[1], "clutch_check_diff") == 0) {
            config.clutch.check_rpm_diff = atof(argv[2]);
            commands_printf("Clutch check RPM diff set to %f", (double)config.clutch.check_rpm_diff);
			v.as_float = config.clutch.check_rpm_diff;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_CHECK_RPM_DIFF_ADDR);
        } else if (strcmp(argv[1], "update_rate") == 0) {
            config.update_rate_hz = atoi(argv[2]);
            commands_printf("Update rate set to %d Hz", config.update_rate_hz);
			v.as_u32 = config.update_rate_hz;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_UPDATE_RATE_HZ_ADDR);
        } else if (strcmp(argv[1], "pedal_ramp_time_pos") == 0) {
            config.pedal_sensor.ramp_time_pos = atof(argv[2]);
            commands_printf("Pedal ramp time positive set to %f", (double)config.pedal_sensor.ramp_time_pos);
			v.as_float = config.pedal_sensor.ramp_time_pos;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS_ADDR);
        } else if (strcmp(argv[1], "pedal_ramp_time_neg") == 0) {
            config.pedal_sensor.ramp_time_neg = atof(argv[2]);
            commands_printf("Pedal ramp time negative set to %f", (double)config.pedal_sensor.ramp_time_neg);
			v.as_float = config.pedal_sensor.ramp_time_neg;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG_ADDR);
        } else if (strcmp(argv[1], "wheel_ramp_time_pos") == 0) {
            config.wheel_sensor.ramp_time_pos = atof(argv[2]);
            commands_printf("Wheel ramp time positive set to %f", (double)config.wheel_sensor.ramp_time_pos);
			v.as_float = config.wheel_sensor.ramp_time_pos;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS_ADDR);
        } else if (strcmp(argv[1], "wheel_ramp_time_neg") == 0) {
            config.wheel_sensor.ramp_time_neg = atof(argv[2]);
            commands_printf("Wheel ramp time negative set to %f", (double)config.wheel_sensor.ramp_time_neg);
			v.as_float = config.wheel_sensor.ramp_time_neg;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG_ADDR);
        } else if (strcmp(argv[1], "clutch_min_rpm") == 0) {
            config.clutch.min_rpm = atof(argv[2]);
            commands_printf("Clutch min RPM set to %f", (double)config.clutch.min_rpm);
			v.as_float = config.clutch.min_rpm;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_MIN_RPM_ADDR);
        } else if (strcmp(argv[1], "clutch_max_rpm_open") == 0) {
            config.clutch.max_rpm_open = atof(argv[2]);
            commands_printf("Clutch max RPM open set to %f", (double)config.clutch.max_rpm_open);
			v.as_float = config.clutch.max_rpm_open;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_MAX_RPM_OPEN_ADDR);
        } else if (strcmp(argv[1], "clutch_max_rpm_close") == 0) {
            config.clutch.max_rpm_close = atof(argv[2]);
            commands_printf("Clutch max RPM close set to %f", (double)config.clutch.max_rpm_close);
			v.as_float = config.clutch.max_rpm_close;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_MAX_RPM_CLOSE_ADDR);
        } else if (strcmp(argv[1], "clutch_mode") == 0) {
            if (strcmp(argv[2], "closed") == 0) {
                config.clutch.mode = CLUTCH_MODE_CLOSED;
                commands_printf("Clutch mode set to CLOSED");
            } else if (strcmp(argv[2], "open") == 0) {
                config.clutch.mode = CLUTCH_MODE_OPEN;
                commands_printf("Clutch mode set to OPEN");
            } else if (strcmp(argv[2], "auto") == 0) {
                config.clutch.mode = CLUTCH_MODE_AUTO;
                commands_printf("Clutch mode set to AUTO");
            } else if (strcmp(argv[2], "manual") == 0) {
                config.clutch.mode = CLUTCH_MODE_MANUAL;
                commands_printf("Clutch mode set to MANUAL");
        	} else {
                commands_printf("Invalid value.\r\nValid values: closed, open, auto, manual");
            }
			v.as_u32 = config.clutch.mode;
			conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_CONF_CLUTCH_MODE_ADDR);
        } else {
            commands_printf("Unknown parameter.\r\nValid parameters:\r\n  ctrl-type\r\n  pedal_magnets\r\n  pedal_filter\r\n  pedal_rpm_start\r\n  pedal_rpm_end\r\n  pedal_invert\r\n  wheel_magnets\r\n  wheel_filter\r\n  wheel_invert\r\n  brake_start\r\n  brake_end\r\n  brake_wait_release\r\n  brake_release_rpm\r\n  clutch_open\r\n  clutch_close\r\n  clutch_check\r\n  clutch_sync_diff\r\n  clutch_check_diff\r\n  update_rate\r\n  pedal_ramp_time_pos\r\n  pedal_ramp_time_neg\r\n  wheel_ramp_time_pos\r\n  wheel_ramp_time_neg\r\n  clutch_min_rpm\r\n  clutch_max_rpm_open\r\n  clutch_max_rpm_close\r\n  clutch_mode\r\n  pedal_rpm_min\r\n  pedal_rpm_max\r\n  wheel_rpm_min\r\n  wheel_rpm_max\r\n");
        }
    } else {
        commands_printf("This command requires two arguments.\n");
    }
}

static void terminal_log(int argc, const char **argv) {
	eeprom_var v;
	if (argc == 3) {
		int en = 0;
		sscanf(argv[2], "%d", &en);
		if (en != 0 && en != 1){
			commands_printf("unknown value. Valid values: 0 / 1");
			return;
		}
		if (strcmp(argv[1],"sensor") == 0){
			if (en) {
                log_groups_enabled |= (1 << LOG_GROUP_SENSOR);
            } else {
                log_groups_enabled &= ~(1 << LOG_GROUP_SENSOR);
            }
		} else
		if (strcmp(argv[1],"motor") == 0){
			if (en) {
                log_groups_enabled |= (1 << LOG_GROUP_MOTOR);
            } else {
                log_groups_enabled &= ~(1 << LOG_GROUP_MOTOR);
            }
		} else
		if (strcmp(argv[1],"clutch") == 0){
			if (en) {
                log_groups_enabled |= (1 << LOG_GROUP_CLUTCH);
            } else {
                log_groups_enabled &= ~(1 << LOG_GROUP_CLUTCH);
            }
		} else
		if (strcmp(argv[1],"error") == 0){
			if (en) {
                log_groups_enabled |= (1 << LOG_GROUP_ERROR);
            } else {
                log_groups_enabled &= ~(1 << LOG_GROUP_ERROR);
            }
		} else {
			commands_printf("Unknown group.\r\nValid groups:\r\n  sensor\r\n  motor\r\n  clutch\r\n  error\r\n");
		}
		v.as_u32 = log_groups_enabled;
		conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_LOG_GROUPS_ENABLED_ADDR);
	} else {
		commands_printf("This command requires two arguments. Usage:\r\n  log [log_group] [0/1]");
		commands_printf("Valid groups:\r\n  sensor\r\n  motor\r\n  clutch\r\n  error\r\n");
	}
}

// Callback function for the terminal command with arguments.
static void terminal_clutch(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1],"open") == 0){
			open_clutch();
			commands_printf("Clutch opening...");
		} else
		if (strcmp(argv[1],"close") == 0){
			sync_clutch();
			commands_printf("Clutch closing...");
		} else {
			commands_printf("Invalid value.\r\nValid values:\r\n  open\r\n  close\r\n");
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

// Function to handle terminal commands
static void terminal_cmd_enable_plot(int argc, const char **argv) {
	eeprom_var v;
    if (argc == 2) {
        if (strcmp(argv[1], "crpm") == 0) {
            plots_enabled |= (1 << PLOT_PEDAL_RPM);
            commands_printf("Pedal RPM plot enabled");
        } else if (strcmp(argv[1], "brake") == 0) {
            plots_enabled |= (1 << PLOT_BRAKE_POS);
            commands_printf("Brake position plot enabled");
        } else if (strcmp(argv[1], "wrpm") == 0) {
            plots_enabled |= (1 << PLOT_WHEEL_RPM);
            commands_printf("Wheel RPM plot enabled");
        } else if (strcmp(argv[1], "hall1") == 0) {
            plots_enabled |= (1 << PLOT_HALL1);
            commands_printf("HALL1 plot enabled");
        } else if (strcmp(argv[1], "hall2") == 0) {
            plots_enabled |= (1 << PLOT_HALL2);
            commands_printf("HALL2 plot enabled");
        } else if (strcmp(argv[1], "hall3") == 0) {
            plots_enabled |= (1 << PLOT_HALL3);
            commands_printf("HALL3 plot enabled");
        } else if (strcmp(argv[1], "mwrpm") == 0) {
            plots_enabled |= (1 << PLOT_MOTOR_RPM);
            commands_printf("Motor RPM plot enabled");
        } else if (strcmp(argv[1], "clutch_state") == 0) {
            plots_enabled |= (1 << PLOT_CLUTCH_STATE);
            commands_printf("Clutch State plot enabled");
		} else if (strcmp(argv[1], "all") == 0) {
			plots_enabled = 0xFFFFFFFF;
			commands_printf("All plots enabled");
        } else {
            commands_printf("Invalid value.\r\nValid values:\r\n  crmp\r\n  brake\r\n  wrpm\r\n  hall1\r\n  hall2\r\n  hall3\r\n  mwrpm\r\n  clutch_state\r\n  all\r\n");
        }
		v.as_u32 = plots_enabled;
		conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_PLOTS_ENABLED_ADDR);
        init_plots();
    } else {
        commands_printf("This command requires one argument. Usage: enable_plot <plot_name>");
    }
}

static void terminal_cmd_disable_plot(int argc, const char **argv) {
	eeprom_var v;
    if (argc == 2) {
        if (strcmp(argv[1], "crpm") == 0) {
            plots_enabled &= ~(1 << PLOT_PEDAL_RPM);
            commands_printf("Pedal RPM plot disabled");
        } else if (strcmp(argv[1], "brake") == 0) {
            plots_enabled &= ~(1 << PLOT_BRAKE_POS);
            commands_printf("Brake position plot disabled");
        } else if (strcmp(argv[1], "wrpm") == 0) {
            plots_enabled &= ~(1 << PLOT_WHEEL_RPM);
            commands_printf("Wheel RPM plot disabled");
        } else if (strcmp(argv[1], "hall1") == 0) {
            plots_enabled &= ~(1 << PLOT_HALL1);
            commands_printf("HALL1 plot disabled");
        } else if (strcmp(argv[1], "hall2") == 0) {
            plots_enabled &= ~(1 << PLOT_HALL2);
            commands_printf("HALL2 plot disabled");
        } else if (strcmp(argv[1], "hall3") == 0) {
            plots_enabled &= ~(1 << PLOT_HALL3);
            commands_printf("HALL3 plot disabled");
        } else if (strcmp(argv[1], "mwrpm") == 0) {
            plots_enabled &= ~(1 << PLOT_MOTOR_RPM);
            commands_printf("Motor RPM plot disabled");
        } else if (strcmp(argv[1], "clutch_state") == 0) {
            plots_enabled &= ~(1 << PLOT_CLUTCH_STATE);
            commands_printf("Clutch State plot disabled");
		} else if (strcmp(argv[1], "all") == 0) {
			plots_enabled = 0;
			commands_printf("All plots disabled");
        } else {
			commands_printf("Invalid value.\r\nValid values:\r\n  crmp\r\n  brake\r\n  wrpm\r\n  hall1\r\n  hall2\r\n  hall3\r\n  mwrpm\r\n  clutch_state\r\n  all\r\n");
        }
		v.as_u32 = plots_enabled;
		conf_general_store_eeprom_var_custom(&v, APP_CUSTOM_PLOTS_ENABLED_ADDR);
        init_plots();
    } else {
		commands_printf("This command requires one argument. Usage: disable_plot <plot_name>");
    }
}

static void terminal_cmd_help(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	commands_printf("Available commands:");
	commands_printf("  set-speed [RPM] - Set the speed to RPM");
	commands_printf("  config [parameter] [value] - Configure custom app parameters");
	commands_printf("    Parameters:");
	commands_printf("      ctrl-type - Control type");
	commands_printf("        Values: none, pid, speed, torque, torque_speed");
	commands_printf("      pedal_sensor_type - Pedal sensor type");
	commands_printf("        Values: single_poll, single_int, quad_poll, quad_int");
	commands_printf("      pedal_magnets - Number of pedal sensor magnets");
	commands_printf("      pedal_filter - Pedal sensor filter (0 to 1 - 1 gives unfiltered value)");
	commands_printf("      pedal_rpm_start - Pedal RPM start value");
	commands_printf("      pedal_rpm_end - Pedal RPM end value");
	commands_printf("      pedal_rpm_min - Pedal RPM min value");
	commands_printf("      pedal_rpm_max - Pedal RPM max value");
	commands_printf("      pedal_ramp_time_pos - Pedal ramp time positive value");
	commands_printf("      pedal_ramp_time_neg - Pedal ramp time negative value");
	commands_printf("      pedal_invert - Invert pedal sensor direction (0 or 1)");
	commands_printf("      wheel_sensor_type - Wheel sensor type");
	commands_printf("        Values: single_poll, single_int, quad_poll, quad_int");
	commands_printf("      wheel_magnets - Number of wheel sensor magnets");
	commands_printf("      wheel_filter - Wheel sensor filter (0 to 1 - 1 gives unfiltered value)");
	commands_printf("      wheel_rpm_min - Wheel RPM min value");
	commands_printf("      wheel_rpm_max - Wheel RPM max value");
	commands_printf("      wheel_ramp_time_pos - Wheel ramp time positive value");
	commands_printf("      wheel_ramp_time_neg - Wheel ramp time negative value");
	commands_printf("      wheel_invert - Invert wheel sensor direction (0 or 1)");
	commands_printf("      brake_start - Back pedal brake start position");
	commands_printf("      brake_end - Back pedal brake end position");
	commands_printf("      brake_wait_release - Back pedal brake wait before release time");
	commands_printf("      brake_release_rpm - Back pedal brake release RPM");
	commands_printf("      clutch_open - Clutch wait before open time");
	commands_printf("      clutch_close - Clutch wait before close time");
	commands_printf("      clutch_check - Clutch wait before check time");
	commands_printf("      clutch_sync_diff - Clutch sync RPM difference");
	commands_printf("      clutch_check_diff - Clutch check RPM difference");
	commands_printf("      clutch_min_rpm - Clutch minimum RPM");
	commands_printf("      clutch_max_rpm_open - Clutch maximum RPM for opening");
	commands_printf("      clutch_max_rpm_close - Clutch maximum RPM for closing");
	commands_printf("      clutch_mode - Clutch mode (closed, open, auto, manual)");
	commands_printf("      update_rate - Update rate in Hz");
	commands_printf("  clutch [open/close] - Open or close the clutch");
	commands_printf("  log [log_group] [0/1] - Enable/disable logging");
	commands_printf("    Log groups: sensor, motor, clutch, error");
	commands_printf("  enable_plot [plot_name] - Enable a plot");
	commands_printf("    Plot names: crpm, brake, wrpm, hall1, hall2, hall3, mwrpm, clutch_state, all");
	commands_printf("  disable_plot [plot_name] - Disable a plot");
	commands_printf("    Plot names: crpm, brake, wrpm, hall1, hall2, hall3, mwrpm, clutch_state, all");
	commands_printf("  getconfig - Get the current configuration settings");
}

static void terminal_get_config(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	commands_printf("Current configuration settings:");
	commands_printf("  Control type: %s", config.ctrl_type == CUSTOM_CTRL_TYPE_NONE ? "none" :
		config.ctrl_type == CUSTOM_CTRL_TYPE_PID ? "pid" :
		config.ctrl_type == CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED ? "speed" :
		config.ctrl_type == CUSTOM_CTRL_TYPE_CURRENT_PEDAL_TORQUE ? "torque" :
		config.ctrl_type == CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED_AND_TORQUE ? "torque_speed" : "unknown");
	commands_printf("  Pedal sensor type: %s", config.pedal_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_POLL ? "single_poll" :
		config.pedal_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_INTERRUPT ? "single_int" :
		config.pedal_sensor.sensor_type == SPEED_SENSOR_TYPE_QUADRATURE_POLL ? "quad_poll" :
		config.pedal_sensor.sensor_type == SPEED_SENSOR_TYPE_QUADRATURE_INTERRUPT ? "quad_int" : "unknown");
	commands_printf("  Pedal sensor magnets: %d", config.pedal_sensor.magnets);
	commands_printf("  Pedal sensor filter: %.2f", (double)config.pedal_sensor.filter);
	commands_printf("  Pedal RPM start: %.2f", (double)config.pedal_sensor.rpm_start);
	commands_printf("  Pedal RPM end: %.2f", (double)config.pedal_sensor.rpm_end);
	commands_printf("  Pedal RPM min: %.2f", (double)config.pedal_sensor.rpm_min);
	commands_printf("  Pedal RPM max: %.2f", (double)config.pedal_sensor.rpm_max);
	commands_printf("  Pedal ramp time positive: %.2f", (double)config.pedal_sensor.ramp_time_pos);
	commands_printf("  Pedal ramp time negative: %.2f", (double)config.pedal_sensor.ramp_time_neg);
	commands_printf("  Pedal sensor invert direction: %d", config.pedal_sensor.invert_direction);
	commands_printf("  Wheel sensor type: %s", config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_POLL ? "single_poll" :
		config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_INTERRUPT ? "single_int" :
		config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_QUADRATURE_POLL ? "quad_poll" :
		config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_QUADRATURE_INTERRUPT ? "quad_int" : "unknown");
	commands_printf("  Wheel sensor magnets: %d", config.wheel_sensor.magnets);
	commands_printf("  Wheel sensor filter: %.2f", (double)config.wheel_sensor.filter);
	commands_printf("  Wheel RPM min: %.2f", (double)config.wheel_sensor.rpm_min);
	commands_printf("  Wheel RPM max: %.2f", (double)config.wheel_sensor.rpm_max);
	commands_printf("  Wheel ramp time positive: %.2f", (double)config.wheel_sensor.ramp_time_pos);
	commands_printf("  Wheel ramp time negative: %.2f", (double)config.wheel_sensor.ramp_time_neg);
	commands_printf("  Wheel sensor invert direction: %d", config.wheel_sensor.invert_direction);
	commands_printf("  Back pedal brake start position: %.2f", (double)config.back_pedal_brake.start_pos);
	commands_printf("  Back pedal brake end position: %.2f", (double)config.back_pedal_brake.end_pos);
	commands_printf("  Back pedal brake wait before release: %.2f", (double)config.back_pedal_brake.wait_before_release);
	commands_printf("  Back pedal brake release RPM: %.2f", (double)config.back_pedal_brake.release_rpm);
	commands_printf("  Clutch wait before open: %.2f", (double)config.clutch.wait_before_open);
	commands_printf("  Clutch wait before sync: %.2f", (double)config.clutch.wait_before_sync);
	commands_printf("  Clutch wait before check: %.2f", (double)config.clutch.wait_before_check);
	commands_printf("  Clutch sync RPM diff: %.2f", (double)config.clutch.sync_rpm_diff);
	commands_printf("  Clutch check RPM diff: %.2f", (double)config.clutch.check_rpm_diff);
	commands_printf("  Clutch min RPM: %.2f", (double)config.clutch.min_rpm);
	commands_printf("  Clutch max RPM open: %.2f", (double)config.clutch.max_rpm_open);
	commands_printf("  Clutch max RPM close: %.2f", (double)config.clutch.max_rpm_close);
	commands_printf("  Clutch mode: %s", config.clutch.mode == CLUTCH_MODE_CLOSED ? "closed" :
		config.clutch.mode == CLUTCH_MODE_OPEN ? "open" :
		config.clutch.mode == CLUTCH_MODE_AUTO ? "auto" :
		config.clutch.mode == CLUTCH_MODE_MANUAL ? "manual" : "unknown");
	commands_printf("  Update rate: %d Hz", config.update_rate_hz);
}

static void update_pedal_torque(void)
{
    if (config.torque_sensor.sensor_type == TORQUE_SENSOR_TYPE_ADC) {
		// Read the external ADC pin voltage
		float torque = ADC_VOLTS(ADC_IND_EXT);

		float torque_rel = utils_map(torque, config_adc.voltage_start, config_adc.voltage_end, 0.0, 1.0);
		
		// Optionally apply a filter
		static float torque_rel_filter = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(torque_rel_filter, torque_rel, FILTER_SAMPLES);

		if (config_adc.use_filter) {
			torque_rel = torque_rel_filter;
		}

		// Truncate the read voltage
		utils_truncate_number(&torque_rel, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config_adc.voltage_inverted) {
			torque_rel = 1.0 - torque_rel;
		}

		// Apply deadband
		utils_deadband(&torque_rel, config_adc.hyst, 1.0);

		// Apply throttle curve
		torque_rel = utils_throttle_curve(torque_rel, config_adc.throttle_exp, config_adc.throttle_exp_brake, config_adc.throttle_exp_mode);

		// Apply ramping
		static systime_t last_time = 0;
		static float torque_rel_ramp = 0.0;
		apply_ramping(&torque_rel_ramp, &last_time, torque_rel, config_adc.ramp_time_pos, config_adc.ramp_time_neg);
		torque_rel = torque_rel_ramp;

		pedal_torque = torque_rel;
		pedal_torque_rel = torque_rel;
    }
}

/* Check pedal speed using quadrature encoder.
*  When pedal is driven backward, calculate relative 
*  position instead of speed for back pedal braking (coaster brake).
*/
static void update_pedal_speed_and_position(bool reset)
{
#ifdef APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1
	// Quadrature Encoder Matrix
	const int8_t QEM[] = {  0, -1,  1,  2,
	                        1,  0,  2, -1,
						   -1,  2,  0,  1,
						    2,  1, -1,  0};
	int8_t direction;
	uint8_t new_state;
	float avg_period;
	static uint8_t old_state = 0;
	static float old_timestamp = 0;
	static float old_period = 0;
	static float inactivity_time = 0;
	static float period_filtered = 0;
	static int32_t forward_direction_counter = 0;
	static int32_t backward_direction_counter = 0;

	if (reset) {
		old_state = 0;
		old_timestamp = 0;
		old_period = 0;
		inactivity_time = 0;
		period_filtered = 0;
		forward_direction_counter = 0;
		backward_direction_counter = 0;
		pedal_speed  = 0;
		pedal_speed_rel = 0; 
		pedal_brake_position = 0;
		pedal_brake_position_rel = 0;
		return;
	}

	// read quadrature encoder state
	HALL1_level = palReadPad(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN1);
	HALL2_level = palReadPad(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT2, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN2);

	// determine direction from old and new state
	new_state = HALL2_level * 2 + HALL1_level;
	direction = (float) QEM[old_state * 4 + new_state];
	old_state = new_state;

	if (config.pedal_sensor.invert_direction) {
        direction *= -1;
	}

	// count the number of consecutive forward/backward phase changes
	// - backward counter is limited based on the back padal brake config
	// - to filter glitches, there should be always a 0 direction between 
	//      two state changes, meaning that we stay at least for 2 samples 
	//      in the same state
	if (direction == 1) {
		if (backward_direction_counter > 0){
			backward_direction_counter--;
		} else {
			forward_direction_counter++;
		}
	} 
	else if (direction == -1) {
		if (backward_direction_counter < max_backward_counter){
			backward_direction_counter++;
		}
		forward_direction_counter = 0;
	}
	
	const float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

	plot_points(PLOT_HALL1, timestamp, HALL1_level * 20);
    plot_points(PLOT_HALL2, timestamp, HALL2_level * 20);

	// calculate forward speed (for assistance)
	// sensors are poorly placed, so use only one rising edge as reference.
	if( (new_state == 3) &&  (direction == 1)) {
		// calculate the time of one full rotation from the time difference
		float period = (timestamp - old_timestamp) * (float)config.pedal_sensor.magnets;

		// quadrature encoder has 4 states, so we should observe 4 phase changes 
		// in the same direction before we reach a specific state again. 
		if (forward_direction_counter == 4) {
			// average last 2 periods due to differences between the upward and downward magnet orientation
			avg_period = 0.5 * (period + old_period);

			// apply simple low pass filtering.
			// 1.0 means no filtering, 0.0 means infinitely strong filtering
			UTILS_LP_FAST(period_filtered, avg_period, config.pedal_sensor.filter);

#ifdef DEBUG_PRINT
			print_log(LOG_GROUP_SENSOR,"%d - %d \r\n", forward_direction_counter, backward_direction_counter);
#endif

			if(period_filtered < min_pedal_period) { //can't be that short, abort
				return;
			}

			// calculate speed from rotation time
			pedal_speed = 60.0 / period_filtered;

			old_period = period;
			backward_direction_counter = 0;
			pedal_brake_position = 0.0;
		}

		old_timestamp = timestamp;
		inactivity_time = 0.0;
		forward_direction_counter = 0;
	}
	else {
		// if there was no measurement, check if the silent period is
		// longer than the latest period and decrease estimated speed accordingly
		float period = (timestamp - old_timestamp) * (float)config.pedal_sensor.magnets;
		avg_period = 0.5 * (period + old_period);		
		if ((60.0 / avg_period) < pedal_speed) {
			pedal_speed = 60.0 / avg_period;
		}	
		
		// increase inactivity time whenever we are between two measurements
		// does not necessarily mean that the pedal is not rotating, we just
		// don't know when the next measurement will happen
		inactivity_time += 1.0 / (float)config.update_rate_hz;

		//if no pedal activity for a given, long enough period, set RPM as zero
		if(inactivity_time > max_pedal_period) {
			pedal_speed = 0.0;
		}
	}

	// calculate backward position (for braking)
	if (backward_direction_counter >= min_backward_counter){
		// position is directly proportional to the encoder phase counter
		pedal_brake_position = backward_direction_counter * (360.0f / (float)(4.0 * config.pedal_sensor.magnets));
		pedal_speed = 0.0;
	} else {
		pedal_brake_position = 0.0;
	}

	// calculate relative speed and position
	pedal_speed_rel = utils_map(pedal_speed, config.pedal_sensor.rpm_start, config.pedal_sensor.rpm_end, 0.0, 1.0);
	utils_truncate_number((float*)&pedal_speed_rel, 0.0, 1.0);
	pedal_brake_position_rel = utils_map(pedal_brake_position, config.back_pedal_brake.start_pos, config.back_pedal_brake.end_pos, 0.0, 1.0);
	utils_truncate_number((float*)&pedal_brake_position_rel, 0.0, 1.0);

	// Apply ramping on pedal speed
	static systime_t last_time = 0;
	static float pedal_speed_ramp = 0.0;
	static float pedal_speed_rel_ramp = 0.0;
	if (pedal_speed > 0.0) {
		apply_ramping(&pedal_speed_ramp, &last_time, pedal_speed, 
						config.pedal_sensor.ramp_time_pos / (config.pedal_sensor.rpm_max - config.pedal_sensor.rpm_min), 
						config.pedal_sensor.ramp_time_neg / (config.pedal_sensor.rpm_max - config.pedal_sensor.rpm_min));
		apply_ramping(&pedal_speed_rel_ramp, &last_time, pedal_speed_rel, 
						config.pedal_sensor.ramp_time_pos / ((config.pedal_sensor.rpm_max - config.pedal_sensor.rpm_min) / (config.pedal_sensor.rpm_end - config.pedal_sensor.rpm_start)), 
						config.pedal_sensor.ramp_time_neg / ((config.pedal_sensor.rpm_max - config.pedal_sensor.rpm_min) / (config.pedal_sensor.rpm_end - config.pedal_sensor.rpm_start)));
		pedal_speed = pedal_speed_ramp;
		pedal_speed_rel = pedal_speed_rel_ramp;
	}

#endif
}

static void update_wheel_speed(void)
{
	static float old_period = 0;
	static float period_filtered = 0;
	static float wheel_sensor_timestamp_old = 0;
	static float inactivity_time = 0;
	static uint8_t HALL3_level_old =  1;
	static float old_timestamp = 0;
	float avg_period;
	float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

	if (config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_INTERRUPT) {
		plot_points(PLOT_HALL3, timestamp, HALL3_int_cntr_xp);
		HALL3_int_cntr_xp = 0;

		if (wheel_sensor_timestamp != 0){
			float period = (wheel_sensor_timestamp - wheel_sensor_timestamp_old) * (float)config.wheel_sensor.magnets;

			if (period < min_wheel_period) { //can't be that short, abort
				return;
			}

			avg_period = 0.5 * (period + old_period);

			UTILS_LP_FAST(period_filtered, avg_period, config.wheel_sensor.filter);

			if(period_filtered < min_wheel_period) { //can't be that short, abort
				return;
			}

			wheel_speed = 60.0 / period_filtered;

			old_period = period;
			wheel_sensor_timestamp_old = wheel_sensor_timestamp;
			wheel_sensor_timestamp = 0;
			inactivity_time = 0.0;
		} else {
			// if there was no measurement, check if the silent period is
			// longer than the latest period and decrease estimated speed accordingly
			float period = (timestamp - wheel_sensor_timestamp_old) * (float)config.wheel_sensor.magnets;
			
			if (period < min_wheel_period) { //can't be that short, abort
				return;
			}

			avg_period = 0.5 * (period + old_period);		
			if ((60.0 / avg_period) < wheel_speed) {
				wheel_speed = 60.0 / avg_period;
			}		

			// increase inactivity time whenever we are between two measurements
			// does not necessarily mean that the wheel is not rotating, we just
			// don't know when the next measurement will happen
			inactivity_time += 1.0 / (float)config.update_rate_hz;

			//if no wheel measurement for a given, long enough period, set RPM as zero
			if(inactivity_time > max_wheel_period) {
				wheel_speed = 0.0;
			}
		}
	} else 
	if (config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_POLL){
		// read the wheel sensor state
		HALL3_level = palReadPad(APP_CUSTOM_CONF_WHEEL_SENSOR_PORT1, APP_CUSTOM_CONF_WHEEL_SENSOR_PIN1);
		plot_points(PLOT_HALL3, timestamp, HALL3_level * 20);

		if (HALL3_level == 1 && HALL3_level_old == 0){
			// calculate the time of one full rotation from the time difference
			float period = (timestamp - old_timestamp) * (float)config.wheel_sensor.magnets;

			if (period < min_wheel_period) { //can't be that short, abort
				return;
			}

			avg_period = 0.5 * (period + old_period);

			// apply simple low pass filtering.
			// 1.0 means no filtering, 0.0 means infinitely strong filtering
			UTILS_LP_FAST(period_filtered, avg_period, 0.8);

			if(period_filtered < min_wheel_period) { //can't be that short, abort
				return;
			}

			// calculate speed from rotation time
			wheel_speed = 60.0 / period_filtered;

			old_period = period;
			old_timestamp = timestamp;
			inactivity_time = 0.0;
		} else {
			// if there was no measurement, check if the silent period is
			// longer than the latest period and decrease estimated speed accordingly
			float period = (timestamp - old_timestamp) * (float)config.wheel_sensor.magnets;
			avg_period = 0.5 * (period + old_period);		
			if ((60.0 / avg_period) < wheel_speed) {
				wheel_speed = 60.0 / avg_period;
			}		

			// increase inactivity time whenever we are between two measurements
			// does not necessarily mean that the wheel is not rotating, we just
			// don't know when the next measurement will happen
			inactivity_time += 1.0 / (float)config.update_rate_hz;

			//if no wheel measurement for a given, long enough period, set RPM as zero
			if(inactivity_time > max_wheel_period) {
				wheel_speed = 0.0;
			}
		}

		HALL3_level_old = HALL3_level;
	}

	// calculate relative wheel speed
	wheel_speed_rel = utils_map(wheel_speed, config.wheel_sensor.rpm_min, config.wheel_sensor.rpm_max, 0.0, 1.0);
	utils_truncate_number((float*)&wheel_speed_rel, 0.0, 1.0);

	// Apply ramping on wheel speed
	static systime_t last_time = 0;
	static float wheel_speed_ramp = 0.0;
	static float wheel_speed_rel_ramp = 0.0;
	if (wheel_speed > 0.0) {
		apply_ramping(&wheel_speed_ramp, &last_time, wheel_speed, config.wheel_sensor.ramp_time_pos / (config.wheel_sensor.rpm_max - config.wheel_sensor.rpm_min), config.wheel_sensor.ramp_time_neg / (config.wheel_sensor.rpm_max - config.wheel_sensor.rpm_min));
		apply_ramping(&wheel_speed_rel_ramp, &last_time, wheel_speed_rel, config.wheel_sensor.ramp_time_pos / 1.0, config.wheel_sensor.ramp_time_neg / 1.0);
		wheel_speed = wheel_speed_ramp;
		wheel_speed_rel = wheel_speed_rel_ramp;
	}
}

static void update_motor_speed(void)
{
	// calculate motor speed from erpm
	// the motor wheel speed (mwrpm) is the mechanical rpm (mrpm) divided by the gear ratio
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	const float mrpm = mc_interface_get_rpm() / (conf->si_motor_poles / 2.0);
	motor_speed = mrpm / conf->si_gear_ratio;
}

static void update_clutch_state(void)
{
	float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	float elapsed_time = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY - clutch_timestamp;
	uint8_t clutch_must_close = 0;
	uint8_t clutch_must_open = 0;
	static float pedal_inactivity_time = 0;
	static float pedal_activity_time = 0;

	if (clutch_state == CLUTCH_STATE_OPENING){
		if (elapsed_time > config.clutch.wait_before_check){
			clutch_state = CLUTCH_STATE_OPEN;
			print_log(LOG_GROUP_CLUTCH,"[%4.2f] OPEN", (double)timestamp);
		}
	} else
	if (clutch_state == CLUTCH_STATE_OPEN && config.clutch.mode != CLUTCH_MODE_CLOSED){
		// check if clutch was opened (motor should slow down)
		if (abs(wheel_speed - motor_speed) < config.clutch.check_rpm_diff){
			clutch_open_error_counter++;
			print_log(LOG_GROUP_CLUTCH,"[%4.2f] OPEN FAILED (%d)", (double)timestamp, clutch_open_error_counter);
			open_clutch();
		} else {
			clutch_open_error_counter = 0;
		}
	} else
	if (clutch_state == CLUTCH_STATE_SYNCING){
		if (abs(wheel_speed - motor_speed) < config.clutch.check_rpm_diff || config.clutch.mode == CLUTCH_MODE_OPEN){
			clutch_state = CLUTCH_STATE_SYNCED;
			print_log(LOG_GROUP_CLUTCH,"[%4.2f] SYNCED", (double)timestamp);
			close_clutch();
		}
	} else 
	if (clutch_state == CLUTCH_STATE_CLOSING){
		if (elapsed_time > config.clutch.wait_before_check){
			clutch_state = CLUTCH_STATE_CLOSED;
			print_log(LOG_GROUP_CLUTCH,"[%4.2f] CLOSED", (double)timestamp);
		}		
	} else
	if (clutch_state == CLUTCH_STATE_CLOSED && config.clutch.mode != CLUTCH_MODE_OPEN){
		// check if clutch was closed (motor should stay in sync with wheel)
		if (abs(wheel_speed - motor_speed) > config.clutch.check_rpm_diff){
			clutch_close_error_counter++;
			print_log(LOG_GROUP_CLUTCH,"[%4.2f] CLOSE FAILED / SYNC LOST (%d)", (double)timestamp, clutch_close_error_counter);
			close_clutch();
		} else {
			clutch_close_error_counter = 0;
		}
	}

	if (config.clutch.mode != CLUTCH_MODE_MANUAL) {
			
		//if wheel speed is too low then clutch must be kept closed for instant start
		if (wheel_speed < config.clutch.min_rpm){
			clutch_must_close = 1;
		} else {
			clutch_must_close = 0;
		}
			
		//if wheel speed is too high then clutch must be kept open to save the motor
		if (wheel_speed > config.clutch.max_rpm_open){
			clutch_must_open = 1;
		}
		if (wheel_speed < config.clutch.max_rpm_close){
			clutch_must_open = 0;
		}

		//if wheel speed is too low then clutch must be kept closed for instant start
		if (clutch_must_close){
			pedal_activity_time = 0;
			pedal_inactivity_time = 0;
			sync_clutch();
		} else if (clutch_must_open){
			pedal_activity_time = 0;
			pedal_inactivity_time = 0;
			open_clutch();
		} else {
			//if pedal speed = 0 and not braking then disconnect clutch after N seconds
			if (pedal_speed == 0 && pedal_brake_position == 0){
				pedal_activity_time = 0;
				if (pedal_inactivity_time < config.clutch.wait_before_open){
					pedal_inactivity_time += 1.0 / (float)config.update_rate_hz;
					if (pedal_inactivity_time >= config.clutch.wait_before_open){
						open_clutch();
					}
				}
			}
			//if pedal speed > 0 then start syncing motor to wheel after N seconds
			// and set power based on torque and pedal speed
			if (pedal_speed > 0){
				pedal_inactivity_time = 0;
				if (pedal_activity_time < config.clutch.wait_before_sync){
					pedal_activity_time += 1.0 / (float)config.update_rate_hz;
					if (pedal_activity_time >= config.clutch.wait_before_sync){
						sync_clutch();
					}
				}
			}
			//if pedal brake is active then start syncing motor to wheel immediately
			if (pedal_brake_position > 0){
				sync_clutch();
			}
		}
	}
}

static void update_motor_control()
{
	float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	static uint32_t cnt = 0;
	
	cnt++;

	if (command_line_speed >= 0){
		set_motor_speed(command_line_speed);
	} else if (clutch_state == CLUTCH_STATE_SYNCING || clutch_state == CLUTCH_STATE_SYNCED || clutch_state == CLUTCH_STATE_CLOSING){
		float target_speed = wheel_speed - config.clutch.sync_rpm_diff;
		if (target_speed < 0){
			target_speed = 0;
		}
		set_motor_speed(target_speed);
		if (cnt % (config.update_rate_hz / 10) == 0){
			print_log(LOG_GROUP_MOTOR,"[%4.2f] RPM set to %4.0f", (double)timestamp, (double)(target_speed));
		}
	} else if (clutch_state == CLUTCH_STATE_CLOSED){
		if (pedal_brake_position > 0){
			float brake_force = (pedal_brake_position - config.back_pedal_brake.start_pos) / (config.back_pedal_brake.end_pos - config.back_pedal_brake.start_pos);
			mc_interface_set_brake_current_rel(brake_force);
			if (cnt % (config.update_rate_hz / 10) == 0){
				print_log(LOG_GROUP_MOTOR,"[%4.2f] BREAK set to %d%%", (double)timestamp, (int)floor(brake_force*100));
			}
		} else if (pedal_speed > 0){
			switch (config.ctrl_type){
				case CUSTOM_CTRL_TYPE_NONE:
					break;
				case CUSTOM_CTRL_TYPE_PID:
					set_motor_speed(pedal_speed);
					if (cnt % (config.update_rate_hz / 10) == 0){
						print_log(LOG_GROUP_MOTOR,"[%4.2f] RPM set to %4.0f (tmp solution)", (double)timestamp, (double)(pedal_speed));
					}
					break;
				case CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED: 
					mc_interface_set_current_rel(pedal_speed_rel);
					break;
				case CUSTOM_CTRL_TYPE_CURRENT_PEDAL_TORQUE: 
					mc_interface_set_current_rel(pedal_torque_rel);
					break;
				case CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED_AND_TORQUE:
					mc_interface_set_current_rel(pedal_speed_rel * pedal_torque_rel);
					break;
				default: 
					break;
			}
		} else {
			if (cnt % (config.update_rate_hz / 10) == 0){
				print_log(LOG_GROUP_MOTOR,"[%4.2f] RPM set to %4.0f", (double)timestamp, 0);
			}
			mc_interface_set_current_rel(0.0);
		}
	} else { //clutch open or opening
		if (cnt % (config.update_rate_hz / 10) == 0){
			print_log(LOG_GROUP_MOTOR,"[%4.2f] CURRENT set to %d", (double)timestamp, 0);
		}
		mc_interface_set_current_rel(0.0);
	}
}

static void open_clutch(void)
{
	//if (clutch_open_error_counter > APP_CUSTOM_CONF_CLUTCH_MAX_ATTEMPTS){
	//	print_log(LOG_GROUP_CLUTCH,"[%4.2f] CLUTCH OPEN DISABLED DUE TO TOO MANY FAILURES", (double)clutch_timestamp);
	//	return;
	//}
	if (clutch_state != CLUTCH_STATE_OPEN && clutch_state != CLUTCH_STATE_OPENING){ 
		palWritePad(APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1, APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1, 1);
		clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		clutch_state = CLUTCH_STATE_OPENING;
		print_log(LOG_GROUP_CLUTCH,"[%4.2f] OPENING...", (double)clutch_timestamp);
	}
}

static void sync_clutch(void)
{
	//if (clutch_close_error_counter > APP_CUSTOM_CONF_CLUTCH_MAX_ATTEMPTS){
	//	print_log(LOG_GROUP_CLUTCH,"[%4.2f] CLUTCH SYNC DISABLED DUE TO TOO MANY FAILURES", (double)clutch_timestamp);
	//	return;
	//}
	if (clutch_state == CLUTCH_STATE_OPEN || clutch_state == CLUTCH_STATE_OPENING){ 
		clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		clutch_state = CLUTCH_STATE_SYNCING;
		print_log(LOG_GROUP_CLUTCH,"[%4.2f] SYNCING...", (double)clutch_timestamp);
	}
}

static void close_clutch(void)
{
	//if (clutch_close_error_counter > APP_CUSTOM_CONF_CLUTCH_MAX_ATTEMPTS){
	//	print_log(LOG_GROUP_CLUTCH,"[%4.2f] CLUTCH CLOSE DISABLED DUE TO TOO MANY FAILURES", (double)clutch_timestamp);
	//	return;
	//}
	if (clutch_state != CLUTCH_STATE_CLOSED && clutch_state != CLUTCH_STATE_CLOSING){ 
		palWritePad(APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1, APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1, 0);
		clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		clutch_state = CLUTCH_STATE_CLOSING;
		print_log(LOG_GROUP_CLUTCH,"[%4.2f] CLOSING...", (double)clutch_timestamp);
	}
}

static void set_motor_speed(float mwrpm) {
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	const float erpm = mwrpm * conf->si_gear_ratio * (conf->si_motor_poles / 2.0);
	mc_interface_set_pid_speed(erpm);
}

// Setting up pin interrupt
void enable_interrupt()
{
#ifdef HW_ENC_EXTI_PORTSRC
	EXTI_InitTypeDef EXTI_InitStructure;
	
	// Connect EXTI Line to pin
	SYSCFG_EXTILineConfig(HW_ENC_EXTI_PORTSRC, HW_ENC_EXTI_PINSRC);

	// Configure EXTI Line
	EXTI_InitStructure.EXTI_Line = HW_ENC_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set EXTI Line Interrupt to the highest priority
	nvicEnableVector(HW_ENC_EXTI_CH, 0);
#endif
}

// Function to initialize plots and add only the enabled graphs
static void init_plots(void) {
    plot_number = 0;
    commands_init_plot("Time", "RPM");

    if (plots_enabled & (1 << PLOT_PEDAL_RPM)) {
        plot_numbers[PLOT_PEDAL_RPM] = plot_number++;
        commands_plot_add_graph("Pedal RPM (CRPM)");
    }
    if (plots_enabled & (1 << PLOT_BRAKE_POS)) {
        plot_numbers[PLOT_BRAKE_POS] = plot_number++;
        commands_plot_add_graph("Brake position");
    }
    if (plots_enabled & (1 << PLOT_WHEEL_RPM)) {
        plot_numbers[PLOT_WHEEL_RPM] = plot_number++;
        commands_plot_add_graph("Wheel RPM (WRPM)");
    }
    if (plots_enabled & (1 << PLOT_MOTOR_RPM)) {
		const volatile mc_configuration *conf = mc_interface_get_configuration();
		char legend[32];
		// Motor Wheel RPM
	    sprintf(legend,"MWRPM = MRPM / %.1f", (double)(conf->si_gear_ratio));
        plot_numbers[PLOT_MOTOR_RPM] = plot_number++;
        commands_plot_add_graph(legend);
    }
    if (plots_enabled & (1 << PLOT_CLUTCH_STATE)) {
        plot_numbers[PLOT_CLUTCH_STATE] = plot_number++;
        commands_plot_add_graph("Clutch State");
    }
    if (plots_enabled & (1 << PLOT_HALL3)) {
        plot_numbers[PLOT_HALL3] = plot_number++;
        commands_plot_add_graph("HALL3");
    }
    if (plots_enabled & (1 << PLOT_HALL1)) {
        plot_numbers[PLOT_HALL1] = plot_number++;
        commands_plot_add_graph("HALL1");
    }
    if (plots_enabled & (1 << PLOT_HALL2)) {
        plot_numbers[PLOT_HALL2] = plot_number++;
        commands_plot_add_graph("HALL2");
    }
}

// Function to plot points if the plot is enabled
static void plot_points(plot_index_t plot, float x, float y) {
    if (plots_enabled & (1 << plot)) {
        commands_plot_set_graph(plot_numbers[plot]);
        commands_send_plot_points(x, y);
    }
}

static void print_log(log_group_t log_group, const char* format, ...) {
	va_list arg;
	va_start (arg, format);

	if (log_groups_enabled & (1 << log_group)) {
        commands_printf(format, arg);
    }
	va_end (arg);
}

static void apply_ramping(float *value, systime_t *last_time, float target, float ramp_time_pos, float ramp_time_neg) {
	systime_t now = chVTGetSystemTimeX();
	float dt = (float)(now - *last_time) / (float)CH_CFG_ST_FREQUENCY;
	*last_time = now;

	if (target > *value) {
		*value += dt / ramp_time_pos;
		if (*value > target) {
			*value = target;
		}
	} else {
		*value -= dt / ramp_time_neg;
		if (*value < target) {
			*value = target;
		}
	}
}
