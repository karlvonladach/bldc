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

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void terminal_test(int argc, const char **argv);
static void update_pedal_torque(void);
static void update_pedal_speed(void);
static void update_wheel_speed(void);
static void update_motor_speed(void);
//static void enable_interrupt(void);

// Private variables
static volatile custom_config_type config;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float pedal_torque = 0;
static volatile float pedal_speed  = 0;
static volatile float wheel_speed  = 0;
static volatile float motor_speed  = 0;
static volatile float output_speed = 0;
static volatile float command_line_speed = -1;
static volatile float ms_without_power = 0.0;
static volatile float max_pedal_period = 0.0;
static volatile float min_pedal_period = 0.0;

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

void app_custom_configure(app_configuration *conf) {
	(void)conf;
	config.pedal_sensor.sensor_type   = APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE;
	config.pedal_sensor.magnets       = APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS;
	config.pedal_sensor.use_filter    = APP_CUSTOM_CONF_PEDAL_SENSOR_USE_FILTER;
	config.pedal_sensor.rpm_start     = APP_CUSTOM_CONF_PEDAL_RPM_START;
	config.pedal_sensor.rpm_end       = APP_CUSTOM_CONF_PEDAL_RPM_END;
	config.pedal_sensor.ramp_time_pos = APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS;
	config.pedal_sensor.ramp_time_neg = APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG;

	config.wheel_sensor.sensor_type   = APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE;
	config.wheel_sensor.magnets       = APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS;
	config.wheel_sensor.use_filter    = APP_CUSTOM_CONF_WHEEL_SENSOR_USE_FILTER;
	config.wheel_sensor.rpm_start     = APP_CUSTOM_CONF_WHEEL_RPM_START;
	config.wheel_sensor.rpm_end       = APP_CUSTOM_CONF_WHEEL_RPM_END;
	config.wheel_sensor.ramp_time_pos = APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS;
	config.wheel_sensor.ramp_time_neg = APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG;

	config.update_rate_hz = APP_CUSTOM_CONF_UPDATE_RATE_HZ;

	ms_without_power = 0.0;

	// a period longer than this should immediately reduce power to zero
	max_pedal_period = 1.0 / ((config.pedal_sensor.rpm_start / 60.0) * config.pedal_sensor.magnets) * 1.2;

	// if pedal spins at x3 the end rpm, assume its beyond limits
	min_pedal_period = 1.0 / ((config.pedal_sensor.rpm_end * 3.0 / 60.0));
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;
	float timestamp = 0;

	chRegSetThreadName("App Custom");

	is_running = true;

	// Example of using the experiment plot
#ifdef DEBUG_PLOT
	chThdSleepMilliseconds(1000);
	commands_init_plot("Time", "RPM");
	commands_plot_add_graph("Pedal RPM");
	commands_plot_add_graph("Wheel RPM");
	commands_plot_add_graph("Motor RPM");
	commands_plot_add_graph("HALL1");
	commands_plot_add_graph("HALL2");
#endif

	for(;;) {
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

		//measure pedal speed
		update_pedal_speed();
#ifdef DEBUG_PLOT
		commands_plot_set_graph(0);
		commands_send_plot_points(timestamp, pedal_speed);
#endif

		//measure wheel speed
		update_wheel_speed();
#ifdef DEBUG_PLOT
		commands_plot_set_graph(1);
		commands_send_plot_points(timestamp, wheel_speed);
#endif

		//get motor speed
		update_motor_speed();
#ifdef DEBUG_PLOT
		commands_plot_set_graph(2);
		commands_send_plot_points(timestamp, motor_speed);
#endif

		//TODO: if pedal speed = 0 then disconnect clutch after N seconds
		//TODO: if pedal speed > 0 or < 0 then make sure clutch is connected
		//         if disconnected then sync motor speed with wheel speed and connect clutch
		//         start pedal position tracking
		//TODO: if pedal speed > 0 then set power based on torque (and pedal speed)
		//TODO: if pedal speed < 0 then set breaking power based on pedal position

		if (command_line_speed >= 0){
			mc_interface_set_pid_speed(command_line_speed);
		} else {
			mc_interface_set_pid_speed(output_speed);
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
	//TODO
}

static void update_pedal_speed(void)
{
#ifdef APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1
	// Quadrature Encoder Matrix
	const int8_t QEM[] = {  0, -1,  1,  2,
	                        1,  0,  2, -1,
						   -1,  2,  0,  1,
						    2,  1, -1,  0};
	int8_t direction;
	uint8_t new_state;
	static uint8_t old_state = 0;
	static float old_timestamp = 0;
	static float inactivity_time = 0;
	static float period_filtered = 0;
	static int32_t forward_direction_counter = 0;
	static int32_t backward_direction_counter = 0;

	uint8_t HALL1_level = palReadPad(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN1);
	uint8_t HALL2_level = palReadPad(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT2, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN2);

	new_state = HALL2_level * 2 + HALL1_level;
	direction = (float) QEM[old_state * 4 + new_state];
	old_state = new_state;

	if (direction == 1){
		forward_direction_counter++;
		backward_direction_counter = 0;
	} 
	else if (direction == -1) {
		backward_direction_counter++;
		forward_direction_counter = 0;
	}
	
	const float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

#ifdef DEBUG_PLOT
	commands_plot_set_graph(3);
	commands_send_plot_points(timestamp, HALL1_level*20);
	commands_plot_set_graph(4);
	commands_send_plot_points(timestamp, HALL2_level*20);
#endif

	// sensors are poorly placed, so use only one rising edge as reference
	if( (new_state == 3) && ((forward_direction_counter >= 4) || backward_direction_counter >= 4) ) {
		float period = (timestamp - old_timestamp) * (float)config.pedal_sensor.magnets;
		old_timestamp = timestamp;

		UTILS_LP_FAST(period_filtered, period, 0.8);

#ifdef DEBUG_PRINT
		commands_printf("%d - %d \r\n", forward_direction_counter, backward_direction_counter);
#endif

		if(period_filtered < min_pedal_period) { //can't be that short, abort
			return;
		}
		pedal_speed = 60.0 / period_filtered;
		pedal_speed *= (float)direction;
		inactivity_time = 0.0;
		forward_direction_counter = 0;
		backward_direction_counter = 0;
	}
	else {
		inactivity_time += 1.0 / (float)config.update_rate_hz;

		//if no pedal activity, set RPM as zero
		if(inactivity_time > max_pedal_period) {
			pedal_speed = 0.0;
		}
	}
#endif
}

static void update_wheel_speed(void)
{
	//TODO
}

static void update_motor_speed(void)
{
	//TODO
}

	// Example of setting up pin interrupt
// void enable_interrupt()
// {
// 	// Connect EXTI Line to pin
// 	SYSCFG_EXTILineConfig(cfg->exti_portsrc, cfg->exti_pinsrc);

// 	// Configure EXTI Line
// 	EXTI_InitStructure.EXTI_Line = cfg->exti_line;
// 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
// 	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
// 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// 	EXTI_Init(&EXTI_InitStructure);

// 	// Enable and set EXTI Line Interrupt to the highest priority
// 	nvicEnableVector(cfg->exti_ch, 0);
// }