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

// App settings
#define FILTER_SAMPLES					5

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void terminal_test(int argc, const char **argv);

static void update_pedal_torque(void);
static void update_pedal_speed_and_position(void);
static void update_wheel_speed(void);
static void update_motor_speed(void);
static void update_clutch_state(void);

static void open_clutch(void);
static void sync_clutch(void);
static void close_clutch(void);
static void enable_interrupt(void);

// Private variables
//// Config variables
static volatile custom_config_type config;
static volatile adc_config config_adc;

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
static volatile float pedal_speed  = 0;
static volatile float pedal_speed_rel = 0;
static volatile float pedal_brake_position = 0;
static volatile float pedal_brake_position_rel = 0;
static volatile float wheel_speed  = 0;
static volatile float motor_speed  = 0;
static volatile clutch_state_type clutch_state = CLUTCH_STATE_OPEN;

//// Other variables
static volatile float ms_without_power = 0.0;
static volatile float wheel_sensor_timestamp = 0;
static volatile float clutch_timestamp = 0;

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
			"set the speed to RPM",
			"[RPM]",
			terminal_test);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(terminal_test);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

bool app_custom_is_running(void) {
	return is_running;
}

void app_custom_configure(app_configuration *conf) {
	config.ctrl_type                  = APP_CUSTOM_CONF_CTRL_TYPE;

	config.pedal_sensor.sensor_type   = APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE;
	config.pedal_sensor.magnets       = APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS;
	config.pedal_sensor.use_filter    = APP_CUSTOM_CONF_PEDAL_SENSOR_USE_FILTER;
	config.pedal_sensor.rpm_start     = APP_CUSTOM_CONF_PEDAL_RPM_START;
	config.pedal_sensor.rpm_end       = APP_CUSTOM_CONF_PEDAL_RPM_END;
	config.pedal_sensor.ramp_time_pos = APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS;
	config.pedal_sensor.ramp_time_neg = APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG;
	config.pedal_sensor.invert_direction = APP_CUSTOM_CONF_PEDAL_INVERT_DIR;

	config.wheel_sensor.sensor_type   = APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE;
	config.wheel_sensor.magnets       = APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS;
	config.wheel_sensor.use_filter    = APP_CUSTOM_CONF_WHEEL_SENSOR_USE_FILTER;
	config.wheel_sensor.rpm_start     = APP_CUSTOM_CONF_WHEEL_RPM_START;
	config.wheel_sensor.rpm_end       = APP_CUSTOM_CONF_WHEEL_RPM_END;
	config.wheel_sensor.ramp_time_pos = APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS;
	config.wheel_sensor.ramp_time_neg = APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG;
	config.wheel_sensor.invert_direction = APP_CUSTOM_CONF_WHEEL_INVERT_DIR;

	config.torque_sensor.sensor_type  = APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE;

	config.back_pedal_brake.start_pos = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS;
	config.back_pedal_brake.end_pos   = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS;

	config.clutch.wait_before_open    = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN;
	config.clutch.wait_before_close   = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CLOSE;
	config.clutch.wait_before_check   = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK;
	config.clutch.sync_rpm_diff       = APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF;
	config.clutch.check_rpm_diff      = APP_CUSTOM_CONF_CLUTCH_CHECK_RPM_DIFF;

	config.update_rate_hz = APP_CUSTOM_CONF_UPDATE_RATE_HZ;

	if (config.torque_sensor.sensor_type == TORQUE_SENSOR_TYPE_ADC) {
		config_adc = conf->app_adc_conf;
	}

	ms_without_power = 0.0;

	// a period longer than this should immediately reduce power to zero
	max_pedal_period = 1.0 / ((config.pedal_sensor.rpm_start / 60.0) * config.pedal_sensor.magnets) * 1.2;

	// if pedal spins at x3 the end rpm, assume its beyond limits
	min_pedal_period = 1.0 / ((config.pedal_sensor.rpm_end * 3.0 / 60.0));

	// a period longer than this should immediately reduce measurements to zero
	max_wheel_period = 1.0 / ((config.wheel_sensor.rpm_start / 60.0) * config.wheel_sensor.magnets) * 1.2;

	// if wheel spins at x3 the end rpm, assume its beyond limits
	min_wheel_period = 1.0 / ((config.wheel_sensor.rpm_end * 3.0 / 60.0));

	// convert pedal angles to quadrature counter
	min_backward_counter = floor((float)(config.back_pedal_brake.start_pos) / (360.0f / (float)(4.0 * config.pedal_sensor.magnets)));
	max_backward_counter = ceil((float)(config.back_pedal_brake.end_pos) / (360.0f / (float)(4.0 * config.pedal_sensor.magnets)));

	enable_interrupt();
}

void app_custom_pin_isr(void){
	wheel_sensor_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;
	float timestamp = 0;
	float pedal_inactivity_time = 0;
	float pedal_activity_time = 0;

	chRegSetThreadName("App Custom");

	is_running = true;

	// Example of using the experiment plot
#ifdef DEBUG_PLOT
	chThdSleepMilliseconds(1000);
	commands_init_plot("Time", "RPM");
	commands_plot_add_graph("Pedal RPM");
	commands_plot_add_graph("Brake position");
	commands_plot_add_graph("Wheel RPM");
	commands_plot_add_graph("HALL1");
	commands_plot_add_graph("HALL2");
	commands_plot_add_graph("Motor RPM");
#endif

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
		update_pedal_speed_and_position();
#ifdef DEBUG_PLOT
		commands_plot_set_graph(0);
		commands_send_plot_points(timestamp, pedal_speed);
		commands_plot_set_graph(1);
		commands_send_plot_points(timestamp, pedal_brake_position);
#endif

		//measure wheel speed
		update_wheel_speed();
#ifdef DEBUG_PLOT
		commands_plot_set_graph(2);
		commands_send_plot_points(timestamp, wheel_speed);
#endif

		//get motor speed
		update_motor_speed();
#ifdef DEBUG_PLOT
		commands_plot_set_graph(5);
		commands_send_plot_points(timestamp, motor_speed);
#endif

		//take care of clutch state transitions
		update_clutch_state();

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
		//   and set power based on torque and pedal speed
		if (pedal_speed > 0){
			pedal_inactivity_time = 0;
			if (pedal_activity_time < config.clutch.wait_before_close){
				pedal_activity_time += 1.0 / (float)config.update_rate_hz;
				if (pedal_activity_time >= config.clutch.wait_before_close){
					sync_clutch();
				}
			}
		}
		//if pedal brake is active then start syncing motor to wheel immediately
		if (pedal_brake_position > 0){
			if (clutch_state == CLUTCH_STATE_OPEN) {
			sync_clutch();
			}
		}

		if (command_line_speed >= 0){
			mc_interface_set_pid_speed(command_line_speed);
		} else if (clutch_state == CLUTCH_STATE_SYNCING || clutch_state == CLUTCH_STATE_OPENING || clutch_state == CLUTCH_STATE_CLOSING || clutch_state == CLUTCH_STATE_SYNCED){
			mc_interface_set_pid_speed(20*wheel_speed + config.clutch.sync_rpm_diff);
			if (cnt % (config.update_rate_hz / 10) == 0){
				commands_printf("[%4.2f] RPM set to %4.0f", (double)timestamp, (double)(20*wheel_speed + config.clutch.sync_rpm_diff));
			}
		} else if (clutch_state == CLUTCH_STATE_CLOSED){
			if (pedal_brake_position > 0){
				float brake_force = (pedal_brake_position - config.back_pedal_brake.start_pos) / (config.back_pedal_brake.end_pos - config.back_pedal_brake.start_pos);
				mc_interface_set_brake_current_rel(brake_force);
				if (cnt % (config.update_rate_hz / 10) == 0){
					commands_printf("[%4.2f] BREAK set to %d%%", (double)timestamp, (int)floor(brake_force*100));
				}
			} else if (pedal_speed > 0){
				switch (config.ctrl_type){
					case CUSTOM_CTRL_TYPE_NONE:
					    break;
					case CUSTOM_CTRL_TYPE_PID:
						mc_interface_set_pid_speed(20*pedal_speed);
						if (cnt % (config.update_rate_hz / 10) == 0){
							commands_printf("[%4.2f] RPM set to %4.0f (tmp solution)", (double)timestamp, (double)(20*pedal_speed));
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
			}
		} else { //clutch open
			if (cnt % (config.update_rate_hz / 10) == 0){
				commands_printf("[%4.2f] CURRENT set to %d", (double)timestamp, 0);
			}
			mc_interface_set_current_rel(0.0);
		}
	}
}

// Callback function for the terminal command with arguments.
static void terminal_test(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);
		command_line_speed = d;
		commands_printf("RPM set to %d", d);
	} else {
		commands_printf("This command requires one argument.\n");
	}
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
		float ramp_time = fabsf(torque_rel) > fabsf(torque_rel_ramp) ? config_adc.ramp_time_pos : config_adc.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&torque_rel_ramp, torque_rel, ramp_step);
			last_time = chVTGetSystemTimeX();
			torque_rel = torque_rel_ramp;
		}

		pedal_torque = torque_rel;
		pedal_torque_rel = torque_rel;
    }
}

/* Check pedal speed using quadrature encoder.
*  When pedal is driven backward, calculate relative 
*  position instead of speed for back pedal braking (coaster brake).
*/
static void update_pedal_speed_and_position(void)
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

	// read quadrature encoder state
	uint8_t HALL1_level = palReadPad(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN1);
	uint8_t HALL2_level = palReadPad(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT2, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN2);

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

#ifdef DEBUG_PLOT
	commands_plot_set_graph(3);
	commands_send_plot_points(timestamp, HALL1_level*20);
	commands_plot_set_graph(4);
	commands_send_plot_points(timestamp, HALL2_level*20);
#endif

	// calculate forward speed (for assistance)
	// sensors are poorly placed, so use only one rising edge as reference.
	if( (new_state == 3) &&  (direction == 1)) {
		// calculate the time of one full rotation from the time difference
		float period = (timestamp - old_timestamp) * (float)config.pedal_sensor.magnets;

		old_timestamp = timestamp;

		// quadrature encoder has 4 states, so we should observe 4 phase changes 
		// in the same direction before we reach a specific state again. 
		if (forward_direction_counter == 4) {
			// average last 2 periods due to differences between the upward and downward magnet orientation
			avg_period = 0.5 * (period + old_period);
			old_period = period;

			// apply simple low pass filtering.
			// 1.0 means no filtering, 0.0 means infinitely strong filtering
			UTILS_LP_FAST(period_filtered, avg_period, 0.8);

#ifdef DEBUG_PRINT
			commands_printf("%d - %d \r\n", forward_direction_counter, backward_direction_counter);
#endif

			if(period_filtered < min_pedal_period) { //can't be that short, abort
				return;
			}

			// calculate speed from rotation time
			pedal_speed = 60.0 / period_filtered;
			backward_direction_counter = 0;
			pedal_brake_position = 0.0;
		}

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

	pedal_speed_rel = utils_map(pedal_speed, config.pedal_sensor.rpm_start, config.pedal_sensor.rpm_end, 0.0, 1.0);
	pedal_brake_position_rel = utils_map(pedal_brake_position, config.back_pedal_brake.start_pos, config.back_pedal_brake.end_pos, 0.0, 1.0);

#endif
}

static void update_wheel_speed(void)
{
	static float old_period = 0;
	static float period_filtered = 0;
	static float wheel_sensor_timestamp_old = 0;
	static float inactivity_time = 0;

	if (wheel_sensor_timestamp != 0){
		float period = (wheel_sensor_timestamp - wheel_sensor_timestamp_old) * (float)config.wheel_sensor.magnets;
		float avg_period = 0.5 * (period + old_period);
		old_period = period;		
		UTILS_LP_FAST(period_filtered, avg_period, 0.8);
		wheel_speed = 60.0 / period_filtered;
		wheel_sensor_timestamp_old = wheel_sensor_timestamp;
		wheel_sensor_timestamp = 0;
		inactivity_time = 0.0;
	} else {
		// if there was no measurement, check if the silent period is
		// longer than the latest period and decrease estimated speed accordingly
		float period = ((float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY - wheel_sensor_timestamp_old) * (float)config.wheel_sensor.magnets;
		float avg_period = 0.5 * (period + old_period);		
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
}

static void update_motor_speed(void)
{
	motor_speed = mc_interface_get_rpm()/20;
}

static void update_clutch_state(void)
{
	float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	float elapsed_time = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY - clutch_timestamp;

	if (clutch_state == CLUTCH_STATE_OPENING){
		if (elapsed_time > config.clutch.wait_before_check){
			//TODO: check if opening succeeded
			clutch_state = CLUTCH_STATE_OPEN;
			commands_printf("[%4.2f] OPEN", (double)timestamp);
		}
	} else
	if (clutch_state == CLUTCH_STATE_SYNCING){
		if (abs(wheel_speed - motor_speed) < config.clutch.check_rpm_diff){
			clutch_state = CLUTCH_STATE_SYNCED;
			commands_printf("[%4.2f] SYNCED", (double)timestamp);
			close_clutch();
		}
	} else 
	if (clutch_state == CLUTCH_STATE_CLOSING){
		if (elapsed_time > config.clutch.wait_before_check){
			//TODO: check if closing succeeded
			clutch_state = CLUTCH_STATE_CLOSED;
			commands_printf("[%4.2f] CLOSED", (double)timestamp);
		}		
	}
}

static void open_clutch(void)
{
	if (clutch_state != CLUTCH_STATE_OPEN && clutch_state != CLUTCH_STATE_OPENING){ 
		palWritePad(APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1, APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1, 1);
		clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		clutch_state = CLUTCH_STATE_OPENING;
		commands_printf("[%4.2f] OPENING...", (double)clutch_timestamp);
	}
}

static void sync_clutch(void)
{
	if (clutch_state == CLUTCH_STATE_OPEN || clutch_state == CLUTCH_STATE_OPENING){ 
		clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		clutch_state = CLUTCH_STATE_SYNCING;
		commands_printf("[%4.2f] SYNCING...", (double)clutch_timestamp);
	}
}

static void close_clutch(void)
{
	if (clutch_state != CLUTCH_STATE_CLOSED && clutch_state != CLUTCH_STATE_CLOSING){ 
		palWritePad(APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1, APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1, 0);
		clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		clutch_state = CLUTCH_STATE_CLOSING;
		commands_printf("[%4.2f] CLOSING...", (double)clutch_timestamp);
	}
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