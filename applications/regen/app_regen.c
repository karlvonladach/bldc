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
#include "mempools.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "serial.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923
#endif

// App settings
#define FILTER_SAMPLES				            5u
#define CALIBRATION_ROUNDS			           10u
#define DIFF_THRESHOLD_TO_APPLY_COMPENSATION    0.1f
#define MAX_PERIODS_TO_AVG					    8u
#define BIQUAD_FILTER_MEMORY_SIZE               4u
#define NOTCH_FILTER_MEMORY_SIZE				7u
#define MAX_UART_DATA_LEN                       2u

// Macros
#define APP_NOW_SEC ((float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY)

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static const config_param_t* find_config_param(const char* name);
static void display_config_value(const config_param_t *param);
static bool set_config_value(const config_param_t* param, const char* value_str);

static void load_config_defaults(void);
static void load_config_from_eeprom(void);

static void terminal_set_speed(int argc, const char **argv);
static void terminal_calibrate(int argc, const char **argv);
static void terminal_reset_calib(int argc, const char **argv);
static void terminal_config(int argc, const char **argv);
static void terminal_clutch(int argc, const char **argv);
static void terminal_clutch_state(int argc, const char **argv);
static void terminal_log(int argc, const char **argv);
static void terminal_cmd_enable_plot(int argc, const char **argv);
static void terminal_cmd_disable_plot(int argc, const char **argv);
static void terminal_cmd_help(int argc, const char **argv);
static void terminal_get_config(int argc, const char **argv);
static void terminal_set_pin(int argc, const char **argv);
static void terminal_profile(int argc, const char **argv);

static profile_t* get_profile(void);

static void update_pedal_torque(void);
static void update_pedal_speed_and_position(float set_brake_position);
static void update_wheel_speed(void);
static void update_motor_speed(void);
static void update_clutch_state(void);
static void update_assistance_level(void);
static void update_extra_resistance_ekf(float F_motor);
static void update_motor_control(void);

static float notch_filter(float new_value, float *memory, float timeout, bool dual_mode);
static float biquad_filter(float new_value, float *memory, float cutoff_freq, bool derivator);
//static void  calibrate_wheel_sensor(float last_wheel_speed, float last_motor_speed);
//static float compensate_wheel_sensor(float last_wheel_speed, float last_motor_speed);

static void record_clutch_operation(void);
static void open_clutch(void);
static void sync_clutch(void);
static void close_clutch(void);
static void limit_current(void);
static void remove_current_limit(void);
static void new_clutch_state(clutch_state_type cs);

static void set_motor_speed(float mwrpm);
static void enable_interrupt(void);
static void init_plots(void);
static void plot_points(plot_index_t plot, float x, float y);
static void print_log(log_group_t log_group, const char* format, ...);
static void apply_ramping(float *value, systime_t *last_time, float target, float ramp_time_pos, float ramp_time_neg);

static void uart_parser_feed(uint8_t byte);

// Private variables
//// Config variables
static custom_config_type config;
static adc_config config_adc;
static const profile_t profile_table[7] = {
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f},
	{0.5f, 0.0f, 0.1f, 0.0f, 0.7f, 0.5f, 5.0f},
	{0.6f, 0.1f, 0.35f, 0.5f, 2.0f, 0.5f, 7.0f},
	{0.8f, 0.2f, 0.7f, 1.0f, 4.0f, 0.5f, 7.5f},
	{1.2f, 0.2f, 0.7f, 1.0f, 4.0f, 1.2f, 7.5f},
	{0.8f, 0.2f, 0.7f, 1.0f, 4.0f, 0.8f, 13.0f},
	{1.2f, 0.2f, 0.7f, 1.0f, 4.0f, 1.2f, 13.0f}
};
static uint8_t profile_to_use = 3u;
static profile_t profile_active;

static volatile float max_pedal_period = 0.0;
static volatile float min_pedal_period = 0.0;
static volatile float max_wheel_period = 0.0;
static volatile float min_wheel_period = 0.0;

static profile_t* get_profile(void) {
	if (profile_to_use == 3u) {
		profile_active.torque_base_gain = config.ctrl.torque_base_gain;
		profile_active.torque_extra_rel_gain = config.ctrl.torque_extra_rel_gain;
		profile_active.torque_extra_abs_gain = config.ctrl.torque_extra_abs_gain;
		profile_active.torque_acc_gain = config.ctrl.torque_acc_gain;
		profile_active.torque_max_gain = config.ctrl.torque_max_gain;
		profile_active.torque_min_gain = config.ctrl.torque_min_gain;
		profile_active.cutoff_speed = config.ctrl.cutoff_speed;
	} else {
		uint8_t index = profile_to_use;
		if (index >= 7u) {
			index = 3u;
		}
		profile_active = profile_table[index];
	}

	return &profile_active;
}

//// Control variables
static volatile float command_line_speed = -1;

//// State variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float pedal_torque = 0;
static volatile float pedal_torque_rel = 0;
static volatile float pedal_torque_filtered = 0;
static volatile float pedal_torque_filtered_rel = 0;
static volatile float pedal_speed  = 0;     //CRPM
static volatile float pedal_speed_rel = 0;
static volatile float pedal_brake_position = 0;
static volatile float pedal_brake_position_rel = 0;
static volatile float pedal_current_direction = 0;
static volatile float wheel_speed  = 0;     //WRPM
static volatile float wheel_speed_rel = 0;
static volatile float wheel_speed_filtered = 0;
static volatile float wheel_speed_filtered_rel = 0;
static volatile float wheel_accel  = 0;     //WRPM/s
static volatile float wheel_accel_filtered = 0;
static volatile float wheel_speed_pred = 0;
static volatile float motor_speed  = 0;     //MWRPM
static volatile float motor_current_rel = 0;
static volatile float bike_speed = 0;       // m/s
static volatile float bike_speed_filtered = 0;   // m/s
static volatile float bike_accel = 0;	    // m/s²
static volatile float bike_accel_filtered = 0;   // m/s²
static volatile float human_power_w = 0;    // Watts
static volatile float human_energy_Wh = 0;   // Joules
static volatile float normal_resistance = 0;
static volatile float extra_resistance = 0; // Newton
static volatile float extra_resistance_rel = 0;
static volatile float torque_gain = 0;
static volatile clutch_state_type clutch_state = CLUTCH_STATE_OPEN;

// EKF state for extra resistance estimation (row-major 5x5 covariance)
// State vector: x = [bike_speed (m/s), extra_resistance (N), pedal_torque (Nm), pedal_omega (rad/s), bike_accel (m/s^2)]
static float ekf_x[5];
static float ekf_P[25];
static volatile float pedal_torque_estimated = 0.0f;  // [Nm]    EKF filtered pedal torque
static volatile float pedal_speed_estimated  = 0.0f;  // [rad/s] EKF filtered pedal angular speed
static volatile float bike_speed_estimated   = 0.0f;  // [m/s]   EKF filtered bike speed
static volatile float extra_resistance_ekf   = 0.0f;  // [N]     EKF estimated extra resistance
static volatile float wheel_speed_estimated  = 0.0f;  // [m/s]   derived from EKF filtered bike speed
static volatile float bike_accel_estimated   = 0.0f;  // [m/s²]  derived from EKF filtered bike speed
static volatile uint8_t HALL1_level = 0;
static volatile uint8_t HALL2_level = 0;
static volatile uint8_t HALL3_level = 0;

//// Other variables
static volatile uint32_t log_groups_enabled = 0;
static volatile uint32_t plots_enabled = 0;
static volatile int      plot_numbers[PLOT_COUNT] = {0};
static volatile int      plot_number = 0;
static volatile float    ms_without_power = 0.0;
static volatile float    wheel_sensor_timestamp = 0;
static volatile float    clutch_timestamp = 0;
static volatile uint8_t  clutch_open_error_counter = 0;
static volatile uint8_t  clutch_close_error_counter = 0;
static volatile float    clutch_operation_timestamps[CLUTCH_OPERATION_BUFFER_SIZE];
static volatile uint32_t clutch_operation_buffer_index = 0;
static volatile uint32_t clutch_operation_count = 0;
static volatile uint32_t HALL3_int_cntr_xp = 0;
static volatile uint32_t HALL3_int_cntr_rt = 0;
static volatile float    last_close_time = 0;
static volatile bool     calibration_active = false;
static volatile uint32_t calibration_step = 0;
//static volatile float    wheel_sensor_calibration_values[WHEEL_SENSOR_CALIBRATION_VALUES_COUNT] = {0};
//static volatile float    last_motor_speeds[WHEEL_SENSOR_CALIBRATION_VALUES_COUNT] = {0};
//static volatile float    last_wheel_speeds[WHEEL_SENSOR_CALIBRATION_VALUES_COUNT] = {0};
static volatile uint8_t  wheel_sensor_magnet_cntr = 0;
static volatile bool     compensation_active = false;
static volatile float    sin_lut[PEDAL_SENSOR_MAX_MAGNETS * 4] = {0};

// Config table - add new parameters here
static const config_param_t config_table[] = {
    // Control type
    {"astype", "Motor control strategy", CONFIG_TYPE_ENUM, &config.ctrl.ctrl_type, APP_CUSTOM_CONF_CTRL_TYPE_ADDR, 
     {.enum_default = APP_CUSTOM_CONF_CTRL_TYPE}, "none,pid,cadence,torque,cadence_torque,auto"},
    {"astbasegain", "[float] Base torque gain", CONFIG_TYPE_FLOAT, &config.ctrl.torque_base_gain, APP_CUSTOM_CONF_CTRL_TORQUE_BASE_GAIN_ADDR,
     {.float_default = APP_CUSTOM_CONF_CTRL_TORQUE_BASE_GAIN}, NULL},
    {"astexrelgain", "[float] Coefficient of additional torque gain based on (extra_resistance / normal_resistance)", CONFIG_TYPE_FLOAT, &config.ctrl.torque_extra_rel_gain, APP_CUSTOM_CONF_CTRL_TORQUE_EXTRA_REL_GAIN_ADDR,
     {.float_default = APP_CUSTOM_CONF_CTRL_TORQUE_EXTRA_REL_GAIN}, NULL},
    {"astexabsgain", "[float] Coefficient of additional torque gain based on extra resistance", CONFIG_TYPE_FLOAT, &config.ctrl.torque_extra_abs_gain, APP_CUSTOM_CONF_CTRL_TORQUE_EXTRA_ABS_GAIN_ADDR,
     {.float_default = APP_CUSTOM_CONF_CTRL_TORQUE_EXTRA_ABS_GAIN}, NULL},
    {"astaccgain", "[float] Coefficient of additional torque gain based on acceleration", CONFIG_TYPE_FLOAT, &config.ctrl.torque_acc_gain, APP_CUSTOM_CONF_CTRL_TORQUE_ACC_GAIN_ADDR,
     {.float_default = APP_CUSTOM_CONF_CTRL_TORQUE_ACC_GAIN}, NULL},
    {"astmaxgain", "[float] Maximum allowed torque gain", CONFIG_TYPE_FLOAT, &config.ctrl.torque_max_gain, APP_CUSTOM_CONF_CTRL_TORQUE_MAX_GAIN_ADDR,
     {.float_default = APP_CUSTOM_CONF_CTRL_TORQUE_MAX_GAIN}, NULL},
    {"astmingain", "[float] Minimum allowed torque gain", CONFIG_TYPE_FLOAT, &config.ctrl.torque_min_gain, APP_CUSTOM_CONF_CTRL_TORQUE_MIN_GAIN_ADDR,
     {.float_default = APP_CUSTOM_CONF_CTRL_TORQUE_MIN_GAIN}, NULL},
    {"astexp", "[float] Torque control exponent (1.0 is linear, < 1.0 gives more torque at low pedal inputs)", CONFIG_TYPE_FLOAT, &config.ctrl.torque_exponent, APP_CUSTOM_CONF_CTRL_TORQUE_EXPONENT_ADDR,
     {.float_default = APP_CUSTOM_CONF_CTRL_TORQUE_EXPONENT}, NULL},
    {"ascgain", "[float] Cadence control gain", CONFIG_TYPE_FLOAT, &config.ctrl.cadence_gain, APP_CUSTOM_CONF_CTRL_CADENCE_GAIN_ADDR,
     {.float_default = APP_CUSTOM_CONF_CTRL_CADENCE_GAIN}, NULL},
	{"asmotconst", "[float] Motor torque constant in Nm/A, used for calculating motor power", CONFIG_TYPE_FLOAT, &config.ctrl.motor_torque_constant, APP_CUSTOM_CONF_MOTOR_TORQUE_CONSTANT_ADDR,
	 {.float_default = APP_CUSTOM_CONF_MOTOR_TORQUE_CONSTANT}, NULL},
	{"asmotgeff", "[float] Motor-to-wheel gear efficiency", CONFIG_TYPE_FLOAT, &config.ctrl.motor_gear_efficiency, APP_CUSTOM_CONF_MOTOR_GEAR_EFFICIENCY_ADDR,
	 {.float_default = APP_CUSTOM_CONF_MOTOR_GEAR_EFFICIENCY}, NULL},
	{"aspedgeff", "[float] Pedal-to-wheel gear efficiency", CONFIG_TYPE_FLOAT, &config.ctrl.pedal_gear_efficiency, APP_CUSTOM_CONF_PEDAL_GEAR_EFFICIENCY_ADDR,
	 {.float_default = APP_CUSTOM_CONF_PEDAL_GEAR_EFFICIENCY}, NULL},
	{"asmeff", "[kg] Effective rider+bike mass", CONFIG_TYPE_FLOAT, &config.ctrl.effective_mass, APP_CUSTOM_CONF_EFFECTIVE_MASS_ADDR,
	 {.float_default = APP_CUSTOM_CONF_EFFECTIVE_MASS}, NULL},
	{"asresc0", "[N] 0th-order resistance coefficient", CONFIG_TYPE_FLOAT, &config.ctrl.resistance_coeff_0, APP_CUSTOM_CONF_RESISTANCE_COEFF_0_ADDR,
	 {.float_default = APP_CUSTOM_CONF_RESISTANCE_COEFF_0}, NULL},
	{"asresc1", "[N*s/m] 1st-order resistance coefficient", CONFIG_TYPE_FLOAT, &config.ctrl.resistance_coeff_1, APP_CUSTOM_CONF_RESISTANCE_COEFF_1_ADDR,
	 {.float_default = APP_CUSTOM_CONF_RESISTANCE_COEFF_1}, NULL},
	{"asresc2", "[N*s^2/m^2] 2nd-order resistance coefficient", CONFIG_TYPE_FLOAT, &config.ctrl.resistance_coeff_2, APP_CUSTOM_CONF_RESISTANCE_COEFF_2_ADDR,
	 {.float_default = APP_CUSTOM_CONF_RESISTANCE_COEFF_2}, NULL},
	{"asresratmax", "[float] Maximum ratio of extra resistance to normal resistance", CONFIG_TYPE_FLOAT, &config.ctrl.resistance_ratio_max, APP_CUSTOM_CONF_RESISTANCE_RATIO_MAX_ADDR,
	 {.float_default = APP_CUSTOM_CONF_RESISTANCE_RATIO_MAX}, NULL},
	{"assoftsta", "[m/s] Soft start speed interval for gradually increasing assist", CONFIG_TYPE_FLOAT, &config.ctrl.ramp_up_speed_interval, APP_CUSTOM_CONF_CTRL_RAMP_UP_ADDR,
	 {.float_default = APP_CUSTOM_CONF_CTRL_RAMP_UP}, NULL},
	{"ascutint", "[m/s] Soft limit speed interval for gradually decreasing assist", CONFIG_TYPE_FLOAT, &config.ctrl.ramp_down_speed_interval, APP_CUSTOM_CONF_CTRL_RAMP_DOWN_ADDR,
	 {.float_default = APP_CUSTOM_CONF_CTRL_RAMP_DOWN}, NULL},
	{"ascutend", "[m/s] Speed above which assist is disabled", CONFIG_TYPE_FLOAT, &config.ctrl.cutoff_speed, APP_CUSTOM_CONF_CTRL_CUTOFF_SPEED_ADDR,
	 {.float_default = APP_CUSTOM_CONF_CTRL_CUTOFF_SPEED}, NULL},

    {"velsrate", "[Hz] Velocity sampling rate", CONFIG_TYPE_UINT32, &config.velocity_sampling_rate, APP_CUSTOM_CONF_VELOCITY_SAMPLING_RATE_ADDR,
	 {.uint32_default = APP_CUSTOM_CONF_VELOCITY_SAMPLING_RATE}, NULL},
	{"xresfilt", "[0.0-1.0] Extra resistance filter: 0.0 to 1.0 where 1.0 gives unfiltered value", CONFIG_TYPE_FLOAT, &config.extra_resistance_filter, APP_CUSTOM_CONF_EXTRA_RESISTANCE_FILTER_ADDR,
	 {.float_default = APP_CUSTOM_CONF_EXTRA_RESISTANCE_FILTER}, NULL},
	{"accfilt", "[0.0-1.0] Acceleration filter: 0.0 to 1.0 where 1.0 gives unfiltered value", CONFIG_TYPE_FLOAT, &config.acceleration_filter, APP_CUSTOM_CONF_ACCELERATION_FILTER_ADDR,
	 {.float_default = APP_CUSTOM_CONF_ACCELERATION_FILTER}, NULL},
	{"acctout", "[sec] Time of pedal inactivity before zeroing acceleration", CONFIG_TYPE_FLOAT, &config.acceleration_timeout, APP_CUSTOM_CONF_ACCELERATION_TIMEOUT_ADDR,
	 {.float_default = APP_CUSTOM_CONF_ACCELERATION_TIMEOUT}, NULL},

    // Pedal sensor config
    {"pedstype", "Pedal sensor encoding type", CONFIG_TYPE_ENUM, &config.pedal_sensor.sensor_type, APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE_ADDR, 
     {.enum_default = APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE}, "single_poll,single_int,quad_poll,quad_int"},
    {"pedmagn", "[count] Number of pedal sensor magnets including 'virtual' magnets", CONFIG_TYPE_UINT32, &config.pedal_sensor.magnets, APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS_ADDR, 
     {.uint32_default = APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS}, NULL},
    {"pedfilt", "[0.0-1.0] Pedal sensor filter: 0.0 to 1.0 where 1.0 gives unfiltered value", CONFIG_TYPE_FLOAT, &config.pedal_sensor.filter, APP_CUSTOM_CONF_PEDAL_SENSOR_FILTER_ADDR, 
     {.float_default = APP_CUSTOM_CONF_PEDAL_SENSOR_FILTER}, NULL},
    {"pedavgrpm", "[rpm] CRPM threshold above which to average last two samples", CONFIG_TYPE_FLOAT, &config.pedal_sensor.avg_above_rpm, APP_CUSTOM_CONF_PEDAL_AVG_ABOVE_RPM_ADDR, 
     {.float_default = APP_CUSTOM_CONF_PEDAL_AVG_ABOVE_RPM}, NULL},
    {"pedstrpm", "[rpm] CRPM start of boost range", CONFIG_TYPE_FLOAT, &config.pedal_sensor.rpm_start, APP_CUSTOM_CONF_PEDAL_RPM_START_ADDR, 
     {.float_default = APP_CUSTOM_CONF_PEDAL_RPM_START}, NULL},
    {"pedendrpm", "[rpm] CRPM end of boost range", CONFIG_TYPE_FLOAT, &config.pedal_sensor.rpm_end, APP_CUSTOM_CONF_PEDAL_RPM_END_ADDR, 
     {.float_default = APP_CUSTOM_CONF_PEDAL_RPM_END}, NULL},
    {"pedminrpm", "[rpm] CRPM minimum threshold - set 0 CRPM below this value", CONFIG_TYPE_FLOAT, &config.pedal_sensor.rpm_min, APP_CUSTOM_CONF_PEDAL_RPM_MIN_ADDR, 
     {.float_default = APP_CUSTOM_CONF_PEDAL_RPM_MIN}, NULL},
    {"pedmaxrpm", "[rpm] CRPM maximum threshold - raise error above this value", CONFIG_TYPE_FLOAT, &config.pedal_sensor.rpm_max, APP_CUSTOM_CONF_PEDAL_RPM_MAX_ADDR, 
     {.float_default = APP_CUSTOM_CONF_PEDAL_RPM_MAX}, NULL},
    {"pedramppos", "[sec] Pedal positive ramp time in sec/fullscale from min to max", CONFIG_TYPE_FLOAT, &config.pedal_sensor.ramp_time_pos, APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS_ADDR, 
     {.float_default = APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS}, NULL},
    {"pedrampneg", "[sec] Pedal negative ramp time in sec/fullscale from max to min", CONFIG_TYPE_FLOAT, &config.pedal_sensor.ramp_time_neg, APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG_ADDR, 
     {.float_default = APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG}, NULL},
    {"pedinv", "[0/1] Invert pedal sensor direction: 1=invert, 0=no invert", CONFIG_TYPE_BOOL, &config.pedal_sensor.invert_direction, APP_CUSTOM_CONF_PEDAL_INVERT_DIR_ADDR, 
     {.bool_default = APP_CUSTOM_CONF_PEDAL_INVERT_DIR}, NULL},
    
    // Wheel sensor config
    {"whstype", "Wheel sensor encoding type", CONFIG_TYPE_ENUM, &config.wheel_sensor.sensor_type, APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE_ADDR, 
     {.enum_default = APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE}, "single_poll,single_int,quad_poll,quad_int,single_poll_single_int,none"},
    {"whpollintrpm", "[rpm] WRPM threshold at which to switch from polling to interrupt mode", CONFIG_TYPE_FLOAT, &config.wheel_sensor.poll_to_int_rpm, APP_CUSTOM_CONF_WHEEL_POLL_TO_INT_RPM_ADDR, 
     {.float_default = APP_CUSTOM_CONF_WHEEL_POLL_TO_INT_RPM}, NULL},
    {"whmagn", "[count] Number of wheel sensor magnets including 'virtual' magnets", CONFIG_TYPE_UINT32, &config.wheel_sensor.magnets, APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS_ADDR, 
     {.uint32_default = APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS}, NULL},
    {"whfilter", "biquad filter cutoff frequency in Hz (0.5, 1.0, 2.0, 4.0)", CONFIG_TYPE_FLOAT, &config.wheel_sensor.filter, APP_CUSTOM_CONF_WHEEL_SENSOR_FILTER_ADDR, 
     {.float_default = APP_CUSTOM_CONF_WHEEL_SENSOR_FILTER}, NULL},
    {"whavgrpm", "[rpm] WRPM threshold above which to average last two samples", CONFIG_TYPE_FLOAT, &config.wheel_sensor.avg_above_rpm, APP_CUSTOM_CONF_WHEEL_AVG_ABOVE_RPM_ADDR, 
     {.float_default = APP_CUSTOM_CONF_WHEEL_AVG_ABOVE_RPM}, NULL},
	{"whprogavgrpm", "[rpm] if > 0, use progressive averaging, adding one more sample to the average for every multiple of this RPM", CONFIG_TYPE_FLOAT, &config.wheel_sensor.progressive_avg_rpm, APP_CUSTOM_CONF_WHEEL_PROGRESSIVE_AVG_RPM_ADDR,
	 {.float_default = APP_CUSTOM_CONF_WHEEL_PROGRESSIVE_AVG_RPM}, NULL},
    {"whminrpm", "[rpm] WRPM minimum threshold - set 0 WRPM below this value", CONFIG_TYPE_FLOAT, &config.wheel_sensor.rpm_min, APP_CUSTOM_CONF_WHEEL_RPM_MIN_ADDR, 
     {.float_default = APP_CUSTOM_CONF_WHEEL_RPM_MIN}, NULL},
    {"whmaxrpm", "[rpm] WRPM maximum threshold - raise error above this value", CONFIG_TYPE_FLOAT, &config.wheel_sensor.rpm_max, APP_CUSTOM_CONF_WHEEL_RPM_MAX_ADDR, 
     {.float_default = APP_CUSTOM_CONF_WHEEL_RPM_MAX}, NULL},
    {"whramppos", "[sec] Wheel positive ramp time in sec/fullscale from min to max", CONFIG_TYPE_FLOAT, &config.wheel_sensor.ramp_time_pos, APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS_ADDR, 
     {.float_default = APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS}, NULL},
    {"whrampneg", "[sec] Wheel negative ramp time in sec/fullscale from max to min", CONFIG_TYPE_FLOAT, &config.wheel_sensor.ramp_time_neg, APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG_ADDR, 
     {.float_default = APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG}, NULL},
    {"whinv", "[0/1] Invert wheel sensor direction: 1=invert, 0=no invert", CONFIG_TYPE_BOOL, &config.wheel_sensor.invert_direction, APP_CUSTOM_CONF_WHEEL_INVERT_DIR_ADDR, 
     {.bool_default = APP_CUSTOM_CONF_WHEEL_INVERT_DIR}, NULL},
    {"whskipthr", "[1.0-3.0] Wheel sensor skipped magnet threshold ratio: skipped period / normal period", CONFIG_TYPE_FLOAT, &config.wheel_sensor.skipped_magnet_threshold, APP_CUSTOM_CONF_WHEEL_SKIPPED_MAGNET_THR_ADDR, 
     {.float_default = APP_CUSTOM_CONF_WHEEL_SKIPPED_MAGNET_THR}, NULL},
	{"whcalrpm", "[rpm] Wheel calibration RPM", CONFIG_TYPE_FLOAT, &config.wheel_sensor.calibration_rpm, APP_CUSTOM_CONF_WHEEL_CALIBRATION_RPM_ADDR,
     {.float_default = APP_CUSTOM_CONF_WHEEL_CALIBRATION_RPM}, NULL},
    
	// Torque sensor config
	{"tqstype", "Torque sensor type", CONFIG_TYPE_ENUM, &config.torque_sensor.sensor_type, APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE_ADDR, 
	 {.enum_default = APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE}, "none,throttle,pedal"},
	{"tqfilter", "[0.0-1.0] Torque sensor filter: 0.0 to 1.0 where 1.0 gives unfiltered value", CONFIG_TYPE_FLOAT, &config.torque_sensor.filter, APP_CUSTOM_CONF_TORQUE_SENSOR_FILTER_ADDR,
	 {.float_default = APP_CUSTOM_CONF_TORQUE_SENSOR_FILTER}, NULL},
	{"tqmaxnm", "[Nm] Maximum torque in Nm corresponding to max sensor value", CONFIG_TYPE_FLOAT, &config.torque_sensor.nm_max, APP_CUSTOM_CONF_TORQUE_NM_MAX_ADDR,
	 {.float_default = APP_CUSTOM_CONF_TORQUE_NM_MAX}, NULL},
	{"tqthresh", "[Nm] Threshold for detecting if torque is being applied", CONFIG_TYPE_FLOAT, &config.torque_sensor.threshold, APP_CUSTOM_CONF_TORQUE_THRESHOLD_ADDR,
	 {.float_default = APP_CUSTOM_CONF_TORQUE_THRESHOLD}, NULL},
	{"tqto", "[sec] Timeout for torque sensor in seconds", CONFIG_TYPE_FLOAT, &config.torque_sensor.timeout, APP_CUSTOM_CONF_TORQUE_TIMEOUT_ADDR,
	 {.float_default = APP_CUSTOM_CONF_TORQUE_TIMEOUT}, NULL},

    // Back pedal brake config
    {"brstpos", "[deg] Back pedal brake start position in degrees mechanical", CONFIG_TYPE_FLOAT, &config.back_pedal_brake.start_pos, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS_ADDR, 
     {.float_default = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS}, NULL},
    {"brendpos", "[deg] Back pedal brake end position in degrees mechanical", CONFIG_TYPE_FLOAT, &config.back_pedal_brake.end_pos, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS_ADDR, 
     {.float_default = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS}, NULL},
    {"brwaitrls", "[sec] Back pedal brake wait time before release in seconds", CONFIG_TYPE_FLOAT, &config.back_pedal_brake.wait_before_release, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_WAIT_BEFORE_RELEASE_ADDR, 
     {.float_default = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_WAIT_BEFORE_RELEASE}, NULL},
    {"brrlsrpm", "[rpm] WRPM below which back pedal brake release is started", CONFIG_TYPE_FLOAT, &config.back_pedal_brake.release_rpm, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RELEASE_RPM_ADDR, 
     {.float_default = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RELEASE_RPM}, NULL},
    {"brsyncstpos", "[deg] Back pedal brake sync start position in degrees mechanical", CONFIG_TYPE_FLOAT, &config.back_pedal_brake.sync_start_pos, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_SYNC_START_POS_ADDR, 
     {.float_default = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_SYNC_START_POS}, NULL},
    {"brramp", "[sec] Back pedal brake current ramp time in sec/fullscale from min to max", CONFIG_TYPE_FLOAT, &config.back_pedal_brake.current_ramp_time, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_CURRENT_RAMP_TIME_ADDR, 
     {.float_default = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_CURRENT_RAMP_TIME}, NULL},
	{"brrstpos", "[0.0-1.0] Percentage of brake position set just after closing", CONFIG_TYPE_FLOAT, &config.back_pedal_brake.reset_pos_percent, APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RESET_POS_PERCENT_ADDR,
	 {.float_default = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RESET_POS_PERCENT}, NULL},
	 
    // Clutch config
    {"clopenwait", "[sec] Clutch wait time before opening in seconds", CONFIG_TYPE_FLOAT, &config.clutch.wait_before_open, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN}, NULL},
    {"clsyncwait", "[sec] Clutch wait time before sync in seconds", CONFIG_TYPE_FLOAT, &config.clutch.wait_before_sync, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC}, NULL},
    {"clcheckwait", "[sec] Clutch wait time before check in seconds", CONFIG_TYPE_FLOAT, &config.clutch.wait_before_check, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK}, NULL},
    {"clerrorwait", "[sec] Clutch wait time before sync loss in seconds", CONFIG_TYPE_FLOAT, &config.clutch.wait_before_sync_loss, APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC_LOSS_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC_LOSS}, NULL},
    {"clsynctime", "[sec] Time for ensuring stable sync in seconds", CONFIG_TYPE_FLOAT, &config.clutch.sync_time, APP_CUSTOM_CONF_CLUTCH_SYNC_TIME_ADDR, 
	 {.float_default = APP_CUSTOM_CONF_CLUTCH_SYNC_TIME}, NULL},
	{"cldesynctime", "[sec] Time for motor to slow down after clutch is opened in seconds", CONFIG_TYPE_FLOAT, &config.clutch.desync_time, APP_CUSTOM_CONF_CLUTCH_DESYNC_TIME_ADDR, 
	 {.float_default = APP_CUSTOM_CONF_CLUTCH_DESYNC_TIME}, NULL},
	{"clsyncbraketo", "[sec] Clutch sync timeout in seconds", CONFIG_TYPE_FLOAT, &config.clutch.sync_timeout, APP_CUSTOM_CONF_CLUTCH_SYNC_TIMEOUT_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_SYNC_TIMEOUT}, NULL},
	{"clclosedfcheck", "[sec] sync_check_rpm_diff is checked for this long in closed state", CONFIG_TYPE_FLOAT, &config.clutch.closed_first_check_time, APP_CUSTOM_CONF_CLUTCH_CLOSED_FIRST_CHECK_TIME_ADDR, 
	 {.float_default = APP_CUSTOM_CONF_CLUTCH_CLOSED_FIRST_CHECK_TIME}, NULL},
    {"clsyncdiff", "[rpm] Clutch sync target WRPM difference", CONFIG_TYPE_FLOAT, &config.clutch.sync_rpm_diff, APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF}, NULL},
    {"clclosedcheckdiff", "[rpm] Clutch closed check WRPM difference threshold", CONFIG_TYPE_FLOAT, &config.clutch.closed_check_rpm_diff, APP_CUSTOM_CONF_CLUTCH_CLOSED_CHECK_RPM_DIFF_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_CLOSED_CHECK_RPM_DIFF}, NULL},
	{"clopencheckdiff", "[rpm] Clutch open check WRPM difference threshold", CONFIG_TYPE_FLOAT, &config.clutch.open_check_rpm_diff, APP_CUSTOM_CONF_CLUTCH_OPEN_CHECK_RPM_DIFF_ADDR, 
	 {.float_default = APP_CUSTOM_CONF_CLUTCH_OPEN_CHECK_RPM_DIFF}, NULL},
    {"clsynccheckdiff", "[rpm] Clutch sync check WRPM difference threshold", CONFIG_TYPE_FLOAT, &config.clutch.first_check_rpm_diff, APP_CUSTOM_CONF_CLUTCH_FIRST_CHECK_RPM_DIFF_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_FIRST_CHECK_RPM_DIFF}, NULL},
	{"clminrpmopen", "[rpm] WRPM above which clutch can be opened", CONFIG_TYPE_FLOAT, &config.clutch.min_rpm_open, APP_CUSTOM_CONF_CLUTCH_MIN_RPM_OPEN_ADDR, 
	 {.float_default = APP_CUSTOM_CONF_CLUTCH_MIN_RPM_OPEN}, NULL},
	{"clminrpmclose", "[rpm] WRPM below which clutch must be closed", CONFIG_TYPE_FLOAT, &config.clutch.min_rpm_close, APP_CUSTOM_CONF_CLUTCH_MIN_RPM_CLOSE_ADDR, 
	 {.float_default = APP_CUSTOM_CONF_CLUTCH_MIN_RPM_CLOSE}, NULL},
    {"clmaxrpmopen", "[rpm] WRPM above which clutch must be opened", CONFIG_TYPE_FLOAT, &config.clutch.max_rpm_open, APP_CUSTOM_CONF_CLUTCH_MAX_RPM_OPEN_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_MAX_RPM_OPEN}, NULL},
    {"clmaxrpmclose", "[rpm] WRPM below which clutch can be closed", CONFIG_TYPE_FLOAT, &config.clutch.max_rpm_close, APP_CUSTOM_CONF_CLUTCH_MAX_RPM_CLOSE_ADDR, 
     {.float_default = APP_CUSTOM_CONF_CLUTCH_MAX_RPM_CLOSE}, NULL},
    {"clmode", "Clutch operation mode", CONFIG_TYPE_ENUM, &config.clutch.mode, APP_CUSTOM_CONF_CLUTCH_MODE_ADDR, 
     {.enum_default = APP_CUSTOM_CONF_CLUTCH_MODE}, "closed,open,auto,manual,fullmanual"},
    {"clinv", "Invert clutch direction: 1=invert, 0=no invert", CONFIG_TYPE_BOOL, &config.clutch.invert_direction, APP_CUSTOM_CONF_CLUTCH_INVERT_DIR_ADDR, 
     {.bool_default = APP_CUSTOM_CONF_CLUTCH_INVERT_DIR}, NULL},
	{"clerrorlimit", "[count] Maximum number of clutch errors in defined period before disabling clutch", CONFIG_TYPE_UINT32, &config.clutch.error_limit, APP_CUSTOM_CONF_CLUTCH_ERROR_LIMIT_ADDR, 
	 {.uint32_default = APP_CUSTOM_CONF_CLUTCH_ERROR_LIMIT}, NULL},
	{"clerrorperiod", "[sec] Clutch error counting period in seconds", CONFIG_TYPE_FLOAT, &config.clutch.error_period, APP_CUSTOM_CONF_CLUTCH_ERROR_PERIOD_ADDR, 
	 {.float_default = APP_CUSTOM_CONF_CLUTCH_ERROR_PERIOD}, NULL},
	{"clsyncwhileclosing", "[0/1] Enable/disable sync while clutch is closing: 1=enable, 0=disable", CONFIG_TYPE_BOOL, &config.clutch.sync_while_closing, APP_CUSTOM_CONF_CLUTCH_SYNC_WHILE_CLOSING_ADDR, 
	 {.bool_default = APP_CUSTOM_CONF_CLUTCH_SYNC_WHILE_CLOSING}, NULL},
	{"clclimitclosing", "[0.0-1.0] Relative current limit when clutch is closing (0.0 to 1.0)", CONFIG_TYPE_FLOAT, &config.clutch.current_limit_closing, APP_CUSTOM_CONF_CLUTCH_CURRENT_LIMIT_CLOSING_ADDR,
	 {.float_default = APP_CUSTOM_CONF_CLUTCH_CURRENT_LIMIT_CLOSING}, NULL},

    // Other config
    {"updrate", "[Hz] Sensor signal processing and clutch control rate in Hz", CONFIG_TYPE_UINT32, &config.update_rate_hz, APP_CUSTOM_CONF_UPDATE_RATE_HZ_ADDR, 
     {.uint32_default = APP_CUSTOM_CONF_UPDATE_RATE_HZ}, NULL}
};

#define CONFIG_TABLE_SIZE (sizeof(config_table) / sizeof(config_param_t))

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
	palSetPadMode(APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1, APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1, PAL_MODE_OUTPUT_PUSHPULL);
#endif

#ifdef APP_CUSTOM_CONF_TORQUE_SENSOR_PORT1
    if (APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE == TORQUE_SENSOR_TYPE_ADC_THROTTLE || APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE == TORQUE_SENSOR_TYPE_ADC_PEDAL) {
	    palSetPadMode(APP_CUSTOM_CONF_TORQUE_SENSOR_PORT1, APP_CUSTOM_CONF_TORQUE_SENSOR_PIN1, PAL_MODE_INPUT_ANALOG);
	}
#endif

#ifdef APP_CUSTOM_CONF_TORQUE_SENSOR_PORT2
    if (APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE == TORQUE_SENSOR_TYPE_ADC_THROTTLE || APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE == TORQUE_SENSOR_TYPE_ADC_PEDAL) {
	    palSetPadMode(APP_CUSTOM_CONF_TORQUE_SENSOR_PORT2, APP_CUSTOM_CONF_TORQUE_SENSOR_PIN2, PAL_MODE_INPUT_ANALOG);
	}
#endif

	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_OUTPUT_PUSHPULL);

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
			"calibrate",
			"Calibrate the wheel sensors",
			"",
			terminal_calibrate);

	terminal_register_command_callback(
			"reset-calib",
			"Reset wheel sensor calibration values",
			"",
			terminal_reset_calib);

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
			"clutch_state",
			"Set the clutch state",
			"[state number]",
			terminal_clutch_state);

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

	terminal_register_command_callback(
			"setpin",
			"Set the given pin to logical 0 or 1",
			"",
			terminal_set_pin);

	terminal_register_command_callback(
			"profile",
			"Set active profile index",
			"[0-6]",
			terminal_profile);
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
	terminal_unregister_callback(terminal_set_pin);
	terminal_unregister_callback(terminal_profile);

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

//    for (uint8_t i=0; i < WHEEL_SENSOR_CALIBRATION_VALUES_COUNT; i++) {
//		wheel_sensor_calibration_values[i] = 0;
//	}

	load_config_defaults();

	load_config_from_eeprom();

	if (conf_general_read_eeprom_var_custom(&v, APP_CUSTOM_PLOTS_ENABLED_ADDR)) {
		plots_enabled = v.as_u32;
	}

	if (config.torque_sensor.sensor_type == TORQUE_SENSOR_TYPE_ADC_THROTTLE || config.torque_sensor.sensor_type == TORQUE_SENSOR_TYPE_ADC_PEDAL) {
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

	for (uint8_t i=0; i < config.pedal_sensor.magnets*4; i++) {
		sin_lut[i] = sinf((float)i * 2.0f * M_PI / (config.pedal_sensor.magnets*4));
	}

	// Initialize EKF state and covariance
	ekf_x[0] = 0.5f;   // bike speed [m/s]
	ekf_x[1] = 0.0f;   // extra resistance [N]
	ekf_x[2] = 0.0f;   // pedal torque [Nm]
	ekf_x[3] = 0.0f;   // pedal angular speed [rad/s]
	ekf_x[4] = 0.0f;   // bike acceleration [m/s^2]
	for (int i = 0; i < 25; i++) ekf_P[i] = 0.0f;
	ekf_P[0*5+0] =   1.0f;
	ekf_P[1*5+1] = 100.0f;
	ekf_P[2*5+2] =  50.0f;
	ekf_P[3*5+3] =   5.0f;
	ekf_P[4*5+4] =   2.0f;

	enable_interrupt();
}

void app_custom_pin_isr(void){
	wheel_sensor_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	HALL3_int_cntr_xp++;
	HALL3_int_cntr_rt++;
}

void app_custom_get_rtdata(float* data) {
	data[0] = pedal_speed_estimated;
	data[1] = wheel_speed_estimated;
	data[2] = motor_speed;
#if defined(HW_UBOX_SINGLE_80)
	data[3] = pedal_brake_position;
	data[4] = pedal_torque * 100;
	data[5] = clutch_state;
	data[6] = wheel_speed;
#elif defined(HW60_IS_MK1)
	data[3] = pedal_speed;
	data[4] = pedal_torque * 100;
	data[5] = wheel_speed;
	data[6] = pedal_torque_estimated * 100;
#endif
	data[7] = bike_accel_filtered * 100;
	data[8] = torque_gain;
	data[9] = extra_resistance_ekf;
	data[10] = bike_accel_estimated * 100;
	data[11] = human_power_w;
	data[12] = pedal_torque_filtered * 100;
	data[13] = human_energy_Wh;
}

void app_custom_process_byte(unsigned char byte) {
	//sdWrite(&HW_UART_P_DEV, &byte, 1);
	//commands_printf("SD:%02X\r\n", byte);
	uart_parser_feed(byte);
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;
	static float last_timestamp = 0;
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

		//plot_points(PLOT_TORQUE, timestamp, pedal_torque*100);
		plot_points(PLOT_TORQUE, timestamp, pedal_torque_filtered*100);
		plot_points(PLOT_TORQUE2, timestamp, pedal_torque_estimated*100);

		//measure pedal forward speed or backward position
		update_pedal_speed_and_position(-1);

		plot_points(PLOT_PEDAL_RPM, timestamp, pedal_speed_estimated);
        plot_points(PLOT_BRAKE_POS, timestamp, pedal_brake_position);

		//get motor speed
		update_motor_speed();

		plot_points(PLOT_MOTOR_RPM, timestamp, motor_speed);

		//measure wheel speed
		update_wheel_speed();

		plot_points(PLOT_WHEEL_RPM, timestamp, wheel_speed_estimated);
		plot_points(PLOT_WHEEL_PRED_RPM, timestamp, wheel_speed);
		plot_points(PLOT_ACCEL, timestamp, bike_accel_estimated);

		//take care of clutch state transitions
		update_clutch_state();

		plot_points(PLOT_CLUTCH_STATE, timestamp, clutch_state);

		// take care of auto-assist level changes
		update_assistance_level();
		plot_points(PLOT_ASSIST_LEVEL, timestamp, torque_gain);

		//control motor speed/current according to the current state variables
		update_motor_control();

		//if wheel speed is small then release brake after N seconds
		// note: motor speed is measured here because of the instability of wrpm in interrupt mode
		if (clutch_state == CLUTCH_STATE_CLOSED_BRAKE && motor_speed < config.back_pedal_brake.release_rpm && pedal_brake_position > 0){
			if (wheel_inactivity_time < config.back_pedal_brake.wait_before_release){
				wheel_inactivity_time += 1.0 / (float)config.update_rate_hz;
				if (wheel_inactivity_time >= config.back_pedal_brake.wait_before_release){
					update_pedal_speed_and_position(0);
				}
			}
		} else {
			wheel_inactivity_time = 0;
		}

		if (last_timestamp > 0) {
			// accumulate human energy
			human_energy_Wh += human_power_w * (timestamp - last_timestamp) / 3600.0f;
		}
		last_timestamp = timestamp;
	}
}

// Helper functions for config table
static const config_param_t* find_config_param(const char* name) {
    for (size_t i = 0; i < CONFIG_TABLE_SIZE; i++) {
        if (strcmp(config_table[i].name, name) == 0) {
            return &config_table[i];
        }
    }
    return NULL;
}

// Helper function to display a config value
static void display_config_value(const config_param_t *param) {
    switch (param->type) {
        case CONFIG_TYPE_FLOAT: {
            float value = *(float*)param->config_ptr;
            commands_printf("  %s: %.2f   (%s)", param->name, (double)value, param->description);
            break;
        }
        case CONFIG_TYPE_UINT32: {
            uint32_t value = *(uint32_t*)param->config_ptr;
            commands_printf("  %s: %u   (%s)", param->name, value, param->description);
            break;
        }
        case CONFIG_TYPE_BOOL: {
            uint32_t value = *(uint32_t*)param->config_ptr;
            commands_printf("  %s: %s   (%s)", param->name, value ? "enabled" : "disabled", param->description);
            break;
        }
        case CONFIG_TYPE_ENUM: {
            uint32_t value = *(uint32_t*)param->config_ptr;
            const char *str_value = "unknown";
            
			// Pick the right string from the enum_values field using strtok
			if (param->enum_values) {
				char enum_buf[128];
				strncpy(enum_buf, param->enum_values, sizeof(enum_buf) - 1);
				enum_buf[sizeof(enum_buf) - 1] = '\0';
				char *token = strtok(enum_buf, ",");
				int idx = 0;
				while (token) {
					if (idx == (int)value) {
						str_value = token;
						break;
					}
					token = strtok(NULL, ",");
					idx++;
				}
			}
            commands_printf("  %s: %s   (%s)", param->name, str_value, param->description);
            break;
        }
    }
}

static bool set_config_value(const config_param_t* param, const char* value_str) {
    eeprom_var v;
    
    switch (param->type) {
        case CONFIG_TYPE_FLOAT: {
            float val = atof(value_str);
            *(float*)param->config_ptr = val;
            v.as_float = val;
            break;
        }
        case CONFIG_TYPE_UINT32: {
            uint32_t val = atoi(value_str);
            *(uint32_t*)param->config_ptr = val;
            v.as_u32 = val;
            break;
        }
        case CONFIG_TYPE_BOOL: {
            bool val = atoi(value_str) != 0;
            *(bool*)param->config_ptr = val;
            v.as_u32 = val ? 1 : 0;
            break;
        }
        case CONFIG_TYPE_ENUM: {
            // Parse enum value from string
            uint32_t enum_val = 0;
            const char* enum_values = param->enum_values;
            char enum_copy[128];
            strncpy(enum_copy, enum_values, sizeof(enum_copy) - 1);
            enum_copy[sizeof(enum_copy) - 1] = '\0';

            char* token = strtok(enum_copy, ",");
            while (token != NULL) {
                if (strcmp(token, value_str) == 0) {
                    *(uint32_t*)param->config_ptr = enum_val;
                    v.as_u32 = enum_val;
                    conf_general_store_eeprom_var_custom(&v, param->eeprom_addr);
                    return true;
                }
                enum_val++;
                token = strtok(NULL, ",");
            }
            commands_printf("Invalid enum value '%s'\r\n", value_str);
			commands_printf("  Valid values: %s\r\n", param->enum_values);
            return false;
        }
        default:
            return false;
    }
    
    conf_general_store_eeprom_var_custom(&v, param->eeprom_addr);
    return true;
}

static void load_config_defaults(void) {
    for (size_t i = 0; i < CONFIG_TABLE_SIZE; i++) {
        const config_param_t* param = &config_table[i];
        
        switch (param->type) {
            case CONFIG_TYPE_FLOAT:
                *(float*)param->config_ptr = param->default_value.float_default;
                break;
            case CONFIG_TYPE_UINT32:
                *(uint32_t*)param->config_ptr = param->default_value.uint32_default;
                break;
            case CONFIG_TYPE_BOOL:
                *(bool*)param->config_ptr = param->default_value.bool_default;
                break;
            case CONFIG_TYPE_ENUM:
                *(uint32_t*)param->config_ptr = param->default_value.enum_default;
                break;
        }
    }
}

static void load_config_from_eeprom(void) {
    eeprom_var v;
    
    for (size_t i = 0; i < CONFIG_TABLE_SIZE; i++) {
        const config_param_t* param = &config_table[i];
        
        if (conf_general_read_eeprom_var_custom(&v, param->eeprom_addr)) {
            switch (param->type) {
                case CONFIG_TYPE_FLOAT:
                    *(float*)param->config_ptr = v.as_float;
                    break;
                case CONFIG_TYPE_UINT32:
                case CONFIG_TYPE_ENUM:
                    *(uint32_t*)param->config_ptr = v.as_u32;
                    break;
                case CONFIG_TYPE_BOOL:
                    *(bool*)param->config_ptr = v.as_u32 != 0;
                    break;
            }
        }
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
static void terminal_calibrate(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	command_line_speed = config.wheel_sensor.calibration_rpm;
	calibration_active = true;
	calibration_step = 0;
	commands_printf("Calibration started...");
}

static void terminal_reset_calib(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	// for (uint8_t i=0; i < WHEEL_SENSOR_CALIBRATION_VALUES_COUNT; i++) {
	// 	wheel_sensor_calibration_values[i] = 0;
	// }
	calibration_active = false;
	compensation_active = false;
	wheel_sensor_magnet_cntr = 0;
	commands_printf("Wheel sensor calibration values reset.");
}

// Callback function for the terminal command with arguments.
static void terminal_config(int argc, const char **argv) {
    if (argc == 3) {
        const config_param_t *param = find_config_param(argv[1]);
        if (param) {
            if (set_config_value(param, argv[2])) {
                commands_printf("%s set successfully", param->description);
            } else {
                commands_printf("Failed to set %s", param->name);
            }
        } else {
            // Print available parameters
            commands_printf("Unknown parameter.\r\nValid parameters:\r\n");
            for (unsigned int i = 0; i < sizeof(config_table) / sizeof(config_table[0]); i++) {
                commands_printf("  %s\r\n", config_table[i].name);
            }
        }
    } else {
        commands_printf("This command requires two arguments.\n");
    }
}

static void terminal_log(int argc, const char **argv) {
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
		} else 
		if (strcmp(argv[1],"uart") == 0){
			if (en) {
                log_groups_enabled |= (1 << LOG_GROUP_UART);
            } else {
                log_groups_enabled &= ~(1 << LOG_GROUP_UART);
            }
		} else {
			commands_printf("Unknown group.\r\nValid groups:\r\n  sensor\r\n  motor\r\n  clutch\r\n  error\r\n uart\r\n");
		}
	} else {
		commands_printf("This command requires two arguments. Usage:\r\n  log [log_group] [0/1]");
		commands_printf("Valid groups:\r\n  sensor\r\n  motor\r\n  clutch\r\n  error\r\n  uart\r\n");
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

// Callback function for the terminal command with arguments.
static void terminal_clutch_state(int argc, const char **argv) {
	if (argc == 2) {
		int cs;
		sscanf(argv[1], "%d", &cs);
		clutch_state = cs;
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
		} else if (strcmp(argv[1], "wrpm_pred") == 0) {
			plots_enabled |= (1 << PLOT_WHEEL_PRED_RPM);
			commands_printf("Predicted Wheel RPM plot enabled");
		} else if (strcmp(argv[1], "torque") == 0) {
			plots_enabled |= (1 << PLOT_TORQUE);
			commands_printf("Torque plot enabled");
		} else if (strcmp(argv[1], "torque2") == 0) {
			plots_enabled |= (1 << PLOT_TORQUE2);
			commands_printf("Torque2 plot enabled");
		} else if (strcmp(argv[1], "motor_current") == 0) {
			plots_enabled |= (1 << PLOT_MOTOR_CURRENT);
			commands_printf("Motor current plot enabled");
		} else if (strcmp(argv[1], "assist_level") == 0) {
			plots_enabled |= (1 << PLOT_ASSIST_LEVEL);
			commands_printf("Assist level plot enabled");
		} else if (strcmp(argv[1], "accel") == 0) {
			plots_enabled |= (1 << PLOT_ACCEL);
			commands_printf("Acceleration plot enabled");
		} else if (strcmp(argv[1], "main") == 0) {
			plots_enabled |= (1 << PLOT_PEDAL_RPM);
			plots_enabled |= (1 << PLOT_BRAKE_POS);
			plots_enabled |= (1 << PLOT_WHEEL_RPM);
			plots_enabled |= (1 << PLOT_MOTOR_RPM);
			plots_enabled |= (1 << PLOT_TORQUE);
			plots_enabled |= (1 << PLOT_TORQUE2);
			plots_enabled |= (1 << PLOT_MOTOR_CURRENT);
			plots_enabled |= (1 << PLOT_ASSIST_LEVEL);
			plots_enabled |= (1 << PLOT_ACCEL);
			commands_printf("Main plots (crpm, brake, wrpm, mwrpm, torque, torque2, motor_current, assist_level, accel) enabled");
		} else if (strcmp(argv[1], "all") == 0) {
			plots_enabled = 0xFFFFFFFF;
			commands_printf("All plots enabled");
        } else {
			commands_printf("Invalid value.\r\nValid values:\r\n  crmp\r\n  brake\r\n  wrpm\r\n  hall1\r\n  hall2\r\n  hall3\r\n  mwrpm\r\n  clutch_state\r\n  wrpm_pred\r\n  torque\r\n  torque2\r\n  motor_current\r\n  assist_level\r\n  accel\r\n  main\r\n  all\r\n");
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
		} else if (strcmp(argv[1], "wrpm_pred") == 0) {
			plots_enabled &= ~(1 << PLOT_WHEEL_PRED_RPM);
			commands_printf("Predicted Wheel RPM plot disabled");
		} else if (strcmp(argv[1], "torque") == 0) {
			plots_enabled &= ~(1 << PLOT_TORQUE);
			commands_printf("Torque plot disabled");
		} else if (strcmp(argv[1], "torque2") == 0) {
			plots_enabled &= ~(1 << PLOT_TORQUE2);
			commands_printf("Torque2 plot disabled");
		} else if (strcmp(argv[1], "motor_current") == 0) {
			plots_enabled &= ~(1 << PLOT_MOTOR_CURRENT);
			commands_printf("Motor current plot disabled");
		} else if (strcmp(argv[1], "assist_level") == 0) {
			plots_enabled &= ~(1 << PLOT_ASSIST_LEVEL);
			commands_printf("Assist level plot disabled");
		} else if (strcmp(argv[1], "accel") == 0) {
			plots_enabled &= ~(1 << PLOT_ACCEL);
			commands_printf("Acceleration plot disabled");
		} else if (strcmp(argv[1], "main") == 0) {
			plots_enabled &= ~(1 << PLOT_PEDAL_RPM);
			plots_enabled &= ~(1 << PLOT_BRAKE_POS);
			plots_enabled &= ~(1 << PLOT_WHEEL_RPM);
			plots_enabled &= ~(1 << PLOT_MOTOR_RPM);
			plots_enabled &= ~(1 << PLOT_TORQUE);
			plots_enabled &= ~(1 << PLOT_TORQUE2);
			plots_enabled &= ~(1 << PLOT_MOTOR_CURRENT);
			plots_enabled &= ~(1 << PLOT_ASSIST_LEVEL);
			plots_enabled &= ~(1 << PLOT_ACCEL);
			commands_printf("Main plots (crpm, brake, wrpm, mwrpm, torque, torque2, motor_current, assist_level, accel) disabled");
		} else if (strcmp(argv[1], "all") == 0) {
			plots_enabled = 0;
			commands_printf("All plots disabled");
        } else {
			commands_printf("Invalid value.\r\nValid values:\r\n  crmp\r\n  brake\r\n  wrpm\r\n  hall1\r\n  hall2\r\n  hall3\r\n  mwrpm\r\n  clutch_state\r\n  wrpm_pred\r\n  torque\r\n  torque2\r\n  motor_current\r\n  assist_level\r\n  accel\r\n  main\r\n  all\r\n");
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
	commands_printf("Build time: %s %s\r\n", __DATE__, __TIME__);
	commands_printf("Available commands:");
	commands_printf("  set-speed [RPM] - Set the speed to RPM");
	commands_printf("  config [parameter] [value] - Configure custom app parameters");
	commands_printf("    Parameters:");
	
	// Generate parameter help from config table
	for (unsigned int i = 0; i < sizeof(config_table) / sizeof(config_table[0]); i++) {
		const config_param_t *param = &config_table[i];
		commands_printf("      %s - %s", param->name, param->description);
		
		// Add type-specific information
		if (param->type == CONFIG_TYPE_ENUM) {
			commands_printf("        Values: %s", param->enum_values);
		} else if (param->type == CONFIG_TYPE_BOOL) {
			commands_printf("        Values: 0, 1");
		}
	}
	
	commands_printf("  clutch [open/close] - Open or close the clutch");
	commands_printf("  clutch_state [state number] - Set the clutch state (for debugging)");
	commands_printf("  log [log_group] [0/1] - Enable/disable logging. Logs are grouped by functionality. Groups can be enabled/disabled separately.");
	commands_printf("    Log groups: sensor, motor, clutch, error");
	commands_printf("  enable_plot [plot_name] - Enable a plot");
	commands_printf("    Plot names: crpm, brake, wrpm, hall1, hall2, hall3, mwrpm, clutch_state, wrpm_pred, torque, torque2, motor_current, assist_level, accel, main, all");
	commands_printf("  disable_plot [plot_name] - Disable a plot");
	commands_printf("    Plot names: crpm, brake, wrpm, hall1, hall2, hall3, mwrpm, clutch_state, wrpm_pred, torque, torque2, motor_current, assist_level, accel, main, all");
	commands_printf("  getconfig - Get the current configuration settings");
	commands_printf("  setpin [pin] [value] - Set a pin value");
	commands_printf("    Pins: tx, rx");
	commands_printf("    Values: 0, 1");
	commands_printf("  profile [1-7|name] - Select active assist profile (4/base uses live config values)");
	commands_printf("    Names: charge, ultraeco, eco, base, boost, fast, fast boost");
	commands_printf("  calibrate - Calibrate wheel sensor to compensate magnet misalignments");
	commands_printf("  reset-calib - Reset wheel sensor calibration values");
}

static void terminal_get_config(int argc, const char **argv) {
    (void)argc;
    (void)argv;
    commands_printf("Current configuration settings:");
    
    // Iterate through config table and display all values
    for (unsigned int i = 0; i < sizeof(config_table) / sizeof(config_table[0]); i++) {
        display_config_value(&config_table[i]);
    }
}

static void terminal_set_pin(int argc, const char **argv) {
	if (argc == 3) {
		int en = 0;
		sscanf(argv[2], "%d", &en);
		if (en != 0 && en != 1){
			commands_printf("unknown value. Valid values: 0 / 1");
			return;
		}
		if (strcmp(argv[1],"tx") == 0){
			if (en) {
				palWritePad(HW_UART_TX_PORT, HW_UART_TX_PIN, 1);
			} else {
				palWritePad(HW_UART_TX_PORT, HW_UART_TX_PIN, 0);
			}
		} else 
		if (strcmp(argv[1],"rx") == 0){
			if (en) {
				palWritePad(HW_UART_RX_PORT, HW_UART_RX_PIN, 1);
			} else {
				palWritePad(HW_UART_RX_PORT, HW_UART_RX_PIN, 0);
			}
		} else {
			commands_printf("Unknown pin.\r\nValid pins:\r\n  tx\r\n  rx\r\n  adc2\r\n");
		}
	} else {
		commands_printf("This command requires two arguments. Usage:\r\n  set_pin [pin] [0/1]");
		commands_printf("Valid pins:\r\n  tx\r\n  rx\r\n");
	}
}

static void terminal_profile(int argc, const char **argv) {
	if (argc == 2) {
		uint8_t profile = 255;
		int profile_num = 0;

		if (sscanf(argv[1], "%d", &profile_num) == 1) {
			if (profile_num >= 1 && profile_num <= 7) {
				profile = (uint8_t)(profile_num - 1);
			}
		} else if (strcmp(argv[1], "charge") == 0) {
			profile = 0;
		} else if (strcmp(argv[1], "ultraeco") == 0) {
			profile = 1;
		} else if (strcmp(argv[1], "eco") == 0) {
			profile = 2;
		} else if (strcmp(argv[1], "base") == 0) {
			profile = 3;
		} else if (strcmp(argv[1], "boost") == 0) {
			profile = 4;
		} else if (strcmp(argv[1], "fast") == 0) {
			profile = 5;
		} else if (strcmp(argv[1], "fast boost") == 0) {
			profile = 6;
		}

		if (profile <= 6) {
			profile_to_use = profile;
			commands_printf("Profile set to %u", (unsigned int)(profile + 1));
		} else {
			commands_printf("Invalid profile. Valid numeric values: 1-7");
			commands_printf("Valid names: charge, ultraeco, eco, base, boost, fast, fast boost");
		}
	} else {
		commands_printf("Current profile: %u", (unsigned int)(profile_to_use + 1));
		commands_printf("Usage: profile [1-7|name]");
		commands_printf("Names: charge, ultraeco, eco, base, boost, fast, fast boost");
	}
}

static void update_pedal_torque(void)
{
	if (config.torque_sensor.sensor_type == TORQUE_SENSOR_TYPE_ADC_THROTTLE) {
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
		pedal_torque_filtered = torque_rel;
		pedal_torque_filtered_rel = torque_rel;

	} else
	if (config.torque_sensor.sensor_type == TORQUE_SENSOR_TYPE_ADC_PEDAL) {
		static float torque_inactivity_time = 0;
		static float torque_notch_filter_memory[NOTCH_FILTER_MEMORY_SIZE] = {0};
		static float torque_biquad2_filter_memory[BIQUAD_FILTER_MEMORY_SIZE] = {0};
		static float torque2_lp_filtered = 0;
		float torque2 = ADC_VOLTS(ADC_IND_EXT2);
		float torque2_filtered = torque2;

		// Map the read voltage to 0-1 range based on config values
		torque2 = utils_map(torque2, config_adc.voltage2_start, config_adc.voltage2_end, 0.0, 1.0);

		// Optionally apply a low pass filter to reduce noise. 
		// 1.0 means no filtering, 0.0 means infinitely strong filtering.
		UTILS_LP_FAST(torque2_lp_filtered, torque2, config.torque_sensor.filter);
		torque2 = torque2_lp_filtered;

		// Apply ramping
		static systime_t last_time2 = 0;
		static float torque2_ramp = 0.0;
		apply_ramping(&torque2_ramp, &last_time2, torque2, config_adc.ramp_time_pos, config_adc.ramp_time_neg);
		
		pedal_torque = torque2_ramp;
		pedal_torque_rel = torque2_ramp;

		torque2_filtered = notch_filter(pedal_torque, torque_notch_filter_memory, config.torque_sensor.timeout, true);
		torque2_filtered = biquad_filter(torque2_filtered, torque_biquad2_filter_memory, config.wheel_sensor.filter, false);

		if (torque2 * config.torque_sensor.nm_max < config.torque_sensor.threshold) {
			torque_inactivity_time += 1.0 / (float)config.update_rate_hz;
			if (torque_inactivity_time >= config.torque_sensor.timeout) {
				torque2_filtered = 0;
			}
		} else {
			torque_inactivity_time = 0;
		}

		pedal_torque_filtered = torque2_filtered;
		pedal_torque_filtered_rel = torque2_filtered;
	}
}

/* Check pedal speed using quadrature encoder.
*  When pedal is driven backward, calculate relative 
*  position instead of speed for back pedal braking (coaster brake).
*/
static void update_pedal_speed_and_position(float set_brake_position)
{
#ifdef APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1
	// Quadrature Encoder Matrix
	const int8_t QEM[] = {  0, -1,  1,  2,
	                        1,  0,  2, -1,
						   -1,  2,  0,  1,
						    2,  1, -1,  0};
	int8_t direction;
	int32_t max_backward_counter;
	int32_t brake_start_backward_counter;
	uint8_t new_state;
	float avg_period;
	static uint8_t old_state = 0;
	static float old_timestamp = 0;
	static float old_periods[4] = {0};
	static uint8_t index = 0;
	static float inactivity_time = 0;
	static float period_filtered = 0;
	static int32_t forward_direction_counter = 0;
	static int32_t backward_direction_counter = 0;
	static float brake_inactivity_time = 0;

	// read quadrature encoder state
	HALL1_level = palReadPad(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN1);
	HALL2_level = palReadPad(APP_CUSTOM_CONF_PEDAL_SENSOR_PORT2, APP_CUSTOM_CONF_PEDAL_SENSOR_PIN2);

	// determine direction from old and new state
	new_state = HALL2_level * 2 + HALL1_level;
	direction = QEM[old_state * 4 + new_state];
	old_state = new_state;

	if (config.pedal_sensor.invert_direction) {
        direction *= -1;
	}

	pedal_current_direction = direction;

    max_backward_counter = ceil((float)(config.back_pedal_brake.end_pos) / (360.0f / (float)(4.0 * config.pedal_sensor.magnets)));
	brake_start_backward_counter = ceil((float)(config.back_pedal_brake.start_pos) / (360.0f / (float)(4.0 * config.pedal_sensor.magnets)));

	// count the number of consecutive forward/backward phase changes
	// - backward counter is limited based on the back pedal brake config
	// - to filter glitches, there should be always a 0 direction between 
	//      two state changes, meaning that we stay at least for 2 samples 
	//      in the same state
	if (direction == 1) {
		if (backward_direction_counter > 0){
			if (backward_direction_counter < brake_start_backward_counter) {
				backward_direction_counter = 0;
				forward_direction_counter++;
			} else {
			backward_direction_counter--;
			}
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

	if (set_brake_position >= 0) {
		direction = -1;
		forward_direction_counter = 0;
		backward_direction_counter = ceil(set_brake_position / config.back_pedal_brake.end_pos * max_backward_counter);
		if (backward_direction_counter > max_backward_counter){
			backward_direction_counter = max_backward_counter;
		}
	}
	
	const float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

	plot_points(PLOT_HALL1, timestamp, HALL1_level * 20);
    plot_points(PLOT_HALL2, timestamp, HALL2_level * 20);

	// calculate forward speed (for assistance)
	if(direction == 1) {
		// calculate the time of one full rotation from the time difference
		float period = (timestamp - old_timestamp) * (float)config.pedal_sensor.magnets * 4;

		if (forward_direction_counter > 0) {
			if (pedal_speed > config.pedal_sensor.avg_above_rpm) {
				// average last 4 due to poor alignment of sensors
				old_periods[index] = period;
				index = (index + 1) % 4;
				avg_period = 0.25 * (old_periods[0] + old_periods[1] + old_periods[2] + old_periods[3]);
			} else {
				avg_period = period;
			}

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
		float period = (timestamp - old_timestamp) * (float)config.pedal_sensor.magnets * 4;
		if (pedal_speed > config.pedal_sensor.avg_above_rpm) {
			// average last 4 due to poor alignment of sensors
			old_periods[index] = period;
			avg_period = 0.25 * (old_periods[0] + old_periods[1] + old_periods[2] + old_periods[3]);
		} else {
			avg_period = period;
		}
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
			old_periods[0] = 0.0;
			old_periods[1] = 0.0;
			old_periods[2] = 0.0;
			old_periods[3] = 0.0;
		}
	}

	// calculate backward position (for braking)
	if (backward_direction_counter > 0){
		// position is directly proportional to the encoder phase counter
		pedal_brake_position = backward_direction_counter * (360.0f / (float)(4.0 * config.pedal_sensor.magnets));

		pedal_speed = 0.0;
		old_periods[0] = 0.0;
		old_periods[1] = 0.0;
		old_periods[2] = 0.0;
		old_periods[3] = 0.0;

		if (pedal_brake_position < config.back_pedal_brake.start_pos) {
			brake_inactivity_time += 1.0 / (float)config.update_rate_hz;

			//if brake is not active for a given, long enough period, reset counters
			if(brake_inactivity_time > config.clutch.sync_timeout) {
				backward_direction_counter = 0.0;
				pedal_brake_position = 0.0;
			}
		} else {
			brake_inactivity_time = 0.0;
		}
	} else {
		pedal_brake_position = 0.0;
		brake_inactivity_time = 0.0;
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
	static float old_periods[MAX_PERIODS_TO_AVG-1] = {0.0f};
	static float wheel_speed_raw = 0;

	static float inactivity_time = 0;
	static uint8_t HALL3_level_old =  1;
	static float old_timestamp = 0;
	static bool interrupt_mode  = false;
	float new_timestamp = 0;
	float period, avg_period;
	float current_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	uint8_t num_events = 0;
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	const float wheel_circumference = M_PI * conf->si_wheel_diameter;

	if (config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_INTERRUPT ||
	    (config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_POLL_SINGLE_INTERRUPT && interrupt_mode == true)) {

		// new measurement is based on the interrupt timestamp
		if (wheel_sensor_timestamp != 0) {
			chSysLock();
			// disregard falling edge interrupts
			if (palReadPad(APP_CUSTOM_CONF_WHEEL_SENSOR_PORT1, APP_CUSTOM_CONF_WHEEL_SENSOR_PIN1) == 1) {
			new_timestamp = wheel_sensor_timestamp;
			num_events = HALL3_int_cntr_xp;
			}
			wheel_sensor_timestamp = 0;
			HALL3_int_cntr_xp = 0;
			chSysUnlock();
		}

		if (new_timestamp != 0) {
			plot_points(PLOT_HALL3, new_timestamp, num_events * 10);
		}

		// read the wheel sensor state
		HALL3_level = palReadPad(APP_CUSTOM_CONF_WHEEL_SENSOR_PORT1, APP_CUSTOM_CONF_WHEEL_SENSOR_PIN1);

		plot_points(PLOT_HALL3, current_timestamp, HALL3_level * (10) - 15);

	} else 
	if (config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_POLL ||
		(config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_POLL_SINGLE_INTERRUPT && interrupt_mode == false)) {

		// read the wheel sensor state
		HALL3_level = palReadPad(APP_CUSTOM_CONF_WHEEL_SENSOR_PORT1, APP_CUSTOM_CONF_WHEEL_SENSOR_PIN1);

		plot_points(PLOT_HALL3, current_timestamp, HALL3_level * 10 + 5);

		// new measurement is based on current timestamp if a falling edge was detected
		if (HALL3_level == 1 && HALL3_level_old == 0){
			new_timestamp = current_timestamp;
		}

		wheel_sensor_timestamp = 0;
		HALL3_int_cntr_xp = 0;
		num_events = 1;
	} else 
	if (config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_NONE) {
		wheel_speed_raw = motor_speed;
		return;
	}

	HALL3_level_old = HALL3_level;

	if (new_timestamp != 0) {
		// if there was new measurement, then calculate speed from elapsed time
		//period = (new_timestamp - old_timestamp) * (float)config.wheel_sensor.magnets / (float)num_events;
		period = (new_timestamp - old_timestamp) * (float)config.wheel_sensor.magnets;

		// skip if the measured period is too short, probably a glitch
		if (period < min_wheel_period) {
			return;
		}

		// try to detect missed magnet
		//if (config.wheel_sensor.skipped_magnet_threshold > 0.0f && period > old_period * config.wheel_sensor.skipped_magnet_threshold && 
		//	wheel_speed > config.wheel_sensor.avg_above_rpm && pedal_brake_position_rel == 0.0) {
		//	period /= 2.0;
		//}

		// try to detect glitches
		if (period < old_period / 2.0 && wheel_speed > config.wheel_sensor.avg_above_rpm) {
			return;
		}		

		// If calibration is active, use the new measurement to calibrate the sensor
		//if (calibration_active) {
		//	calibrate_wheel_sensor(60.0 / period, motor_speed);
		//} else 
		// if calibration is not active but we have calibration values, apply them to the new measurement
		// if (wheel_sensor_calibration_values[0] != 0) {
		// 	period = 60.0 / compensate_wheel_sensor(60.0 / period, motor_speed);
		// }

		// average last 2 periods due to differences between the upward and downward magnet orientation
		if (wheel_speed > config.wheel_sensor.avg_above_rpm) {
			if (config.wheel_sensor.progressive_avg_rpm < 0.1) {
				avg_period = 0.5 * (period + old_period);
			} else {
				uint8_t samples_to_avg;
				float samples_f = wheel_speed / config.wheel_sensor.progressive_avg_rpm;
				int samples_i = (int)samples_f + 1;
				if (samples_i < 1) {
					samples_i = 1;
				} else if (samples_i > (int)MAX_PERIODS_TO_AVG) {
					samples_i = (int)MAX_PERIODS_TO_AVG;
				}
				samples_to_avg = (uint8_t)samples_i;
				avg_period = 0.0;
				for (uint8_t i = 0; i < samples_to_avg; i++) {
					avg_period += (i == 0) ? period : old_periods[i-1];
				}
				avg_period /= samples_to_avg;
			}
		} else {
			avg_period = period;
		}

		// if the measured period is too short, probably a glitch
		if(avg_period >= min_wheel_period) {
			// calculate speed from rotation time
			wheel_speed_raw = 60.0 / avg_period;

			// apply simple low pass filtering.
			//UTILS_LP_FAST(wheel_speed_filtered, wheel_speed_raw, config.wheel_sensor.filter);
			//wheel_speed = wheel_speed_filtered;

			// predict wheel speed for the next sample - experimental
			wheel_speed_pred = (60.0 / old_period) + ((60.0 / avg_period) - (60.0 / old_period)) * 1.5;
			if (wheel_speed_pred < 0) {
				wheel_speed_pred = 0.0;
			}

			for (uint8_t i = MAX_PERIODS_TO_AVG - 2; i > 0; i--) {
				old_periods[i] = old_periods[i-1];
			}
			old_periods[0] = period;
			old_period = period;
			old_timestamp = new_timestamp;
			inactivity_time = 0.0;
		}

	} else {
		// if there was no measurement, check if the silent period is
		// longer than the latest period and decrease estimated speed accordingly
		period = (current_timestamp - old_timestamp) * (float)config.wheel_sensor.magnets;
		
		if (period >= min_wheel_period) {
			if (wheel_speed > config.wheel_sensor.avg_above_rpm) {
				avg_period = 0.5 * (period + old_period);
			} else {
				avg_period = period;
			}

			if ((60.0 / avg_period) < wheel_speed) {
				wheel_speed_raw = 60.0 / avg_period;
			}

			if ((60.0 / avg_period) < wheel_speed_pred) {
				wheel_speed_pred = 60.0 / avg_period;
			}

			// increase inactivity time whenever we are between two measurements
			// does not necessarily mean that the wheel is not rotating, we just
			// don't know when the next measurement will happen
			inactivity_time += 1.0 / (float)config.update_rate_hz;

			//if no wheel measurement for a given, long enough period, set RPM as zero
			if(inactivity_time > max_wheel_period) {
				wheel_speed = 0.0;
				wheel_speed_pred = 0.0;
				wheel_speed_raw = 0.0;
				for (uint8_t i = 0; i < MAX_PERIODS_TO_AVG - 1; i++) {
					old_periods[i] = 0;
				}
			}
		}
	}
	
	// apply simple low pass filtering.
	//UTILS_LP_FAST(wheel_speed, wheel_speed_raw, config.wheel_sensor.filter);
	wheel_speed = wheel_speed_raw;

	if (wheel_speed < config.wheel_sensor.rpm_min) {
		wheel_speed = 0.0;
	}

	// calculate bike speed and acceleration from wheel speed
	bike_speed = wheel_speed * wheel_circumference / 60.0;

	// calculate relative wheel speed
	wheel_speed_rel = utils_map(wheel_speed, config.wheel_sensor.rpm_min, config.wheel_sensor.rpm_max, 0.0, 1.0);
	utils_truncate_number((float*)&wheel_speed_rel, 0.0, 1.0);

	// Switch between polling and interrupt mode based on the current wheel speed
	if (new_timestamp != 0 && config.wheel_sensor.sensor_type == SPEED_SENSOR_TYPE_SINGLE_POLL_SINGLE_INTERRUPT) {
		if (wheel_speed >= config.wheel_sensor.poll_to_int_rpm) {
			interrupt_mode = true;
		} else {
			interrupt_mode = false;
		}
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

	if (config.clutch.mode == CLUTCH_MODE_FULL_MANUAL) {
		if (clutch_state == CLUTCH_STATE_OPENING) {
			clutch_state = CLUTCH_STATE_OPEN;
			print_log(LOG_GROUP_CLUTCH,"[%4.2f] OPEN", (double)timestamp);
		} else if (clutch_state == CLUTCH_STATE_SYNCING) {
			clutch_state = CLUTCH_STATE_SYNCED;
			print_log(LOG_GROUP_CLUTCH,"[%4.2f] SYNCED", (double)timestamp);
			close_clutch();
		} else if (clutch_state == CLUTCH_STATE_CLOSING) {
			clutch_state = CLUTCH_STATE_CLOSED_FLOAT;
			print_log(LOG_GROUP_CLUTCH,"[%4.2f] CLOSED", (double)timestamp);
		}
		return;
	}

	bool stopped           = (wheel_speed < config.wheel_sensor.rpm_min && motor_speed < config.wheel_sensor.rpm_min);
	bool going_forward     = (motor_speed > -0.5);
	bool too_slow          = (wheel_speed < config.clutch.min_rpm_close);
	bool not_too_slow      = (wheel_speed > config.clutch.min_rpm_open);
	bool not_too_fast      = (wheel_speed < config.clutch.max_rpm_close);
	bool too_fast          = (wheel_speed > config.clutch.max_rpm_open);
	bool diff_to_target_small_enough = (abs(MAX((wheel_speed - config.clutch.sync_rpm_diff), 0) - motor_speed) < config.clutch.first_check_rpm_diff);
	bool diff_small_enough = (abs(wheel_speed - motor_speed) < config.clutch.first_check_rpm_diff);
	bool diff_large_enough = (abs(wheel_speed - motor_speed) > config.clutch.open_check_rpm_diff);
	bool diff_too_small    = (abs(wheel_speed - motor_speed) < config.clutch.open_check_rpm_diff) && (wheel_speed > config.clutch.open_check_rpm_diff);
	bool diff_too_large    = (abs(wheel_speed - motor_speed) > config.clutch.closed_check_rpm_diff);
	bool pedaling          = (pedal_speed > 0 && pedal_torque > 0 && torque_gain > 0);
	bool braking           = (pedal_brake_position > config.back_pedal_brake.start_pos); // or (pedal_brake_position_rel > 0);
	bool brake_tentative   = (pedal_brake_position > config.back_pedal_brake.sync_start_pos);
	bool manual_mode       = (config.clutch.mode == CLUTCH_MODE_MANUAL);
	bool auto_mode         = (config.clutch.mode == CLUTCH_MODE_AUTO);

	switch (clutch_state) {
		case CLUTCH_STATE_OPEN:
			if (stopped) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (too_slow && auto_mode) {
				new_clutch_state(CLUTCH_STATE_SYNCING);
			}
			else if (elapsed_time > config.clutch.desync_time && diff_too_small) { // got stuck closed
				new_clutch_state(CLUTCH_STATE_OPEN_ERROR);
			}
			else if (pedaling && auto_mode) {
				new_clutch_state(CLUTCH_STATE_WAITING);
			}
			else if (brake_tentative && not_too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_SYNCING);
			}
			break;
		case CLUTCH_STATE_OPEN_ERROR:
			if (stopped) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (too_slow && auto_mode) {
				new_clutch_state(CLUTCH_STATE_SYNCING);
			}
			else if (!diff_too_small) { // got out of closed
				new_clutch_state(CLUTCH_STATE_OPEN);
			}
			else if (elapsed_time > config.clutch.wait_before_sync_loss) { // stuck for too long, try to open
				new_clutch_state(CLUTCH_STATE_CLOSING_TMP);
			}
			break;
		case CLUTCH_STATE_WAITING:
			if (stopped) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (too_slow && auto_mode) {
				new_clutch_state(CLUTCH_STATE_SYNCING);
			}
			else if (!pedaling && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPEN);
			}
			else if (elapsed_time > config.clutch.wait_before_sync && not_too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_SYNCING);
			}
			break;
		case CLUTCH_STATE_SYNCING:
			if (stopped) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPEN);
			} 
			else if (diff_to_target_small_enough) {
				new_clutch_state(CLUTCH_STATE_SYNCED);
			}
			break;
		case CLUTCH_STATE_SYNCED:
			if (stopped) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPEN);
			}
			else if (!diff_to_target_small_enough) {
				new_clutch_state(CLUTCH_STATE_SYNCING);
			}
			else if (too_slow && auto_mode) {
                new_clutch_state(CLUTCH_STATE_CLOSING);
            }
			else if (elapsed_time > config.clutch.sync_time && pedaling && not_too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (elapsed_time > config.clutch.sync_time && braking && not_too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (elapsed_time > config.clutch.sync_time && manual_mode) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (elapsed_time > config.clutch.wait_before_open && !brake_tentative && auto_mode){
				update_pedal_speed_and_position(0); // reset brake position to avoid immediate re-sync
				new_clutch_state(CLUTCH_STATE_OPEN);                
            }
			else if (elapsed_time > config.clutch.sync_timeout && auto_mode) {
				update_pedal_speed_and_position(0); // reset brake position to avoid immediate re-sync
				new_clutch_state(CLUTCH_STATE_OPEN);
			}
			break;
		case CLUTCH_STATE_CLOSING:
			if (elapsed_time > config.clutch.wait_before_check && too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPENING);
			} 
			else if (elapsed_time > config.clutch.wait_before_check && diff_small_enough && pedaling ) {
				new_clutch_state(CLUTCH_STATE_CLOSED_ASSIST);
			}
			else if (elapsed_time > config.clutch.wait_before_check && diff_small_enough && braking ) {
				update_pedal_speed_and_position((pedal_brake_position - config.back_pedal_brake.start_pos) * config.back_pedal_brake.reset_pos_percent + config.back_pedal_brake.start_pos);
				new_clutch_state(CLUTCH_STATE_CLOSED_BRAKE);
			}
			else if (elapsed_time > config.clutch.wait_before_check && diff_small_enough && !pedaling && !braking) {
				new_clutch_state(CLUTCH_STATE_CLOSED_FLOAT);
			}
			else if (elapsed_time > config.clutch.wait_before_check && !diff_small_enough ) { // closing unsuccessful, retry
				new_clutch_state(CLUTCH_STATE_OPENING_TMP);
			}
			break;
		case CLUTCH_STATE_CLOSING_TMP:
			if (stopped) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (elapsed_time > config.clutch.wait_before_check) {
				new_clutch_state(CLUTCH_STATE_OPENING);
			}
			break;
		case CLUTCH_STATE_CLOSED_FLOAT:
			if (too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPENING);
			} 
			else if (going_forward && diff_too_large) { // got out of closed
				new_clutch_state(CLUTCH_STATE_CLOSED_ERROR);
			}
			else if (going_forward && pedaling) {
				new_clutch_state(CLUTCH_STATE_CLOSED_ASSIST);
			}
			else if (going_forward && braking) {
				new_clutch_state(CLUTCH_STATE_CLOSED_BRAKE);
			}
			else if (going_forward && elapsed_time > config.clutch.wait_before_open && not_too_slow && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPENING);
			}
			break;
		case CLUTCH_STATE_CLOSED_BRAKE:
			if (too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPENING);
			} 
			else if (elapsed_time < config.clutch.closed_first_check_time && !diff_small_enough) {
				new_clutch_state(CLUTCH_STATE_CLOSED_ERROR);
			}
			else if (diff_too_large) { // got out of closed
				new_clutch_state(CLUTCH_STATE_CLOSED_ERROR);
			}
			else if (!braking) {
				new_clutch_state(CLUTCH_STATE_CLOSED_FLOAT);
			}
			break;
		case CLUTCH_STATE_CLOSED_ASSIST:
			if (too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPENING);
			}
			else if (elapsed_time < config.clutch.closed_first_check_time && !diff_small_enough) {
				new_clutch_state(CLUTCH_STATE_CLOSED_ERROR);
			}
			else if (diff_too_large) { // got out of closed
				new_clutch_state(CLUTCH_STATE_CLOSED_ERROR);
			}
			else if (!pedaling) {
				new_clutch_state(CLUTCH_STATE_CLOSED_FLOAT);
			}
			break;
		case CLUTCH_STATE_CLOSED_ERROR:
			if (stopped) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPENING);
			} 
			else if (!diff_too_large && pedaling) {
				new_clutch_state(CLUTCH_STATE_CLOSED_ASSIST);
			}
			else if (!diff_too_large && braking) {
				new_clutch_state(CLUTCH_STATE_CLOSED_BRAKE);
			}
			else if (!diff_too_large && !pedaling && !braking) {
				new_clutch_state(CLUTCH_STATE_CLOSED_FLOAT);
			}
			else if (elapsed_time > config.clutch.wait_before_sync_loss) { // out of sync for too long, try to sync again
				new_clutch_state(CLUTCH_STATE_OPENING_TMP);
			}
			break;
		case CLUTCH_STATE_OPENING:
			if (elapsed_time > config.clutch.wait_before_check && stopped) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (elapsed_time > config.clutch.wait_before_check && too_slow && auto_mode) {
				new_clutch_state(CLUTCH_STATE_SYNCING);
			}
			else if (elapsed_time > config.clutch.wait_before_check && diff_large_enough) { // opening successful
				new_clutch_state(CLUTCH_STATE_OPEN);
			}
			else if (elapsed_time > config.clutch.wait_before_check && !diff_large_enough) { // opening failed
				new_clutch_state(CLUTCH_STATE_CLOSING_TMP);
			}
			else if (elapsed_time > config.clutch.wait_before_check && pedaling && auto_mode) {
				new_clutch_state(CLUTCH_STATE_WAITING);
			}
			else if (elapsed_time > config.clutch.wait_before_check && brake_tentative && not_too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_SYNCING);
			}
			break;
		case CLUTCH_STATE_OPENING_TMP:
			if (stopped) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (elapsed_time > config.clutch.wait_before_check) {
				new_clutch_state(CLUTCH_STATE_SYNCING);
			}
			break;
		default:
			break;
	}
}

static void update_assistance_level()
{
	profile_t *profile = get_profile();
	float motor_current_measured;
	float motor_force;
	//float human_force;
	//float extra_resistance_raw;
	//static float wheel_accel_bq_filter_memory[BIQUAD_FILTER_MEMORY_SIZE] = {0};
	static float bike_accel_notch_filter_memory[NOTCH_FILTER_MEMORY_SIZE] = {0};
	static float bike_accel_bq_filter_memory[BIQUAD_FILTER_MEMORY_SIZE] = {0};
	const volatile mc_configuration *conf = mc_interface_get_configuration();

	if (config.ctrl.ctrl_type != CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED_AND_TORQUE_AUTO) {
		return;
	}

	motor_current_measured = mc_interface_get_tot_current_directional_filtered();

	motor_force = motor_current_measured * config.ctrl.motor_torque_constant * 
				conf->si_gear_ratio * config.ctrl.motor_gear_efficiency / 
				(conf->si_wheel_diameter * 0.5f);

	human_power_w = pedal_torque_estimated * config.torque_sensor.nm_max * 
					pedal_speed_estimated * (2.0f * M_PI / 60.0f) * 
					config.ctrl.pedal_gear_efficiency;

	//human_force = human_power_w / MAX(bike_speed_estimated, 0.1f);

	normal_resistance = config.ctrl.resistance_coeff_0 +
	 					config.ctrl.resistance_coeff_1 * bike_speed_estimated +
	 					config.ctrl.resistance_coeff_2 * bike_speed_estimated * bike_speed_estimated;

	// extra_resistance_raw = motor_force + human_force - bike_accel * config.ctrl.effective_mass - normal_resistance;

	// extra_resistance = biquad_filter(extra_resistance_raw, wheel_accel_bq_filter_memory, 0.5f, false);

	// EKF-based extra resistance estimation (replaces the biquad-filtered estimate above)
	update_extra_resistance_ekf(motor_force);

	extra_resistance = extra_resistance_ekf;

	extra_resistance_rel = extra_resistance / MAX(normal_resistance, 0.1f);

	bike_accel_filtered = notch_filter(bike_accel_estimated, bike_accel_notch_filter_memory, config.acceleration_timeout, false);
	bike_accel_filtered = biquad_filter(bike_accel_filtered, bike_accel_bq_filter_memory, config.wheel_sensor.filter, false);

	utils_truncate_number((float *)&extra_resistance_rel, -config.ctrl.resistance_ratio_max, config.ctrl.resistance_ratio_max);
	
	torque_gain = profile->torque_base_gain +
				(bike_speed_estimated < 1.0 ? 0 : profile->torque_extra_rel_gain) * extra_resistance_rel +
				(bike_speed_estimated < 1.0 ? 0 : profile->torque_extra_abs_gain) * extra_resistance / config.ctrl.effective_mass +
				profile->torque_acc_gain * ((bike_accel_filtered > 0) ? bike_accel_filtered : 0);
	
	utils_truncate_number((float *)&torque_gain, profile->torque_min_gain, profile->torque_max_gain);

	// Ramp up around 0 speed and ramp down at regulatory speed limit
	if (bike_speed_estimated >= 0 &&bike_speed_estimated < config.ctrl.ramp_up_speed_interval) {
		torque_gain *= bike_speed_estimated / config.ctrl.ramp_up_speed_interval;
	} else if (bike_speed_estimated >= (profile->cutoff_speed - config.ctrl.ramp_down_speed_interval) && bike_speed_estimated < profile->cutoff_speed) {
		torque_gain *= 1.0 - (bike_speed_estimated - (profile->cutoff_speed - config.ctrl.ramp_down_speed_interval)) / (config.ctrl.ramp_down_speed_interval);
	} else if (bike_speed_estimated >= profile->cutoff_speed) {
		torque_gain = 0.0;
	}
}

static void update_extra_resistance_ekf(float F_motor)
{
	// Extended Kalman Filter for extra resistance (terrain slope) estimation.
	// State:        x = [bike_speed (m/s), extra_resistance (N), pedal_torque (Nm), pedal_omega (rad/s), bike_accel (m/s^2)]
	// Dynamics:     dv/dt = bike_accel
	//               bike_accel = (F_human + F_motor - normal_resistance(v) - extra_resistance) / m
	//               where F_human = tau * omega / v
	// Measurements: z = [bike_speed, pedal_torque_nm, pedal_omega_rads]

	const volatile mc_configuration *conf = mc_interface_get_configuration();

	const float dt = 1.0f / (float)config.update_rate_hz;
	const float m  = MAX(config.ctrl.effective_mass, 1.0f);

	// Convert noisy measurements to SI units
	const float z_v     = bike_speed;
	const float z_tau   = pedal_torque * config.torque_sensor.nm_max;
	const float z_omega = pedal_speed * (2.0f * M_PI / 60.0f);

	// Process noise (Q) diagonal - how much each state can change per step
	const float Q_v     = (0.01f/2.0f)*(0.01f/2.0f); // max 5m/s/1sec 				 -> 0.01/0.002sec = 2sigma
	const float Q_res   = (0.2f/2.0f)*(0.2f/2.0f);   // max 100N/1sec 				 -> 0.2/0.002sec  = 2sigma
	const float Q_tau   = (1.6f/2.0f)*(1.6f/2.0f);   // max 160Nm/0.2sec  			 -> 1.6/0.002sec  = 2sigma
	const float Q_omega = (0.01f/2.0f)*(0.01f/2.0f); // max 50RPM/sec -> 5rad/s/1sec -> 0.01/0.002sec = 2sigma
	const float Q_acc   = (0.01f/2.0f)*(0.01f/2.0f); // max 5m/sec2/1sec			 -> 0.01/0.002sec = 2sigma

	// Measurement noise (R) diagonal - sensor standard deviations squared
	const float R_v     = 0.01f;  // sigma_v     = 3.0 RPM -> 0.1  m/s
	const float R_tau   = 4.0f;   // sigma_tau   = 2.0 Nm
	const float R_omega = 0.04f;  // sigma_omega = 2.0 RPM -> 0.2  rad/s

	// --- PREDICTION STEP ---

	float v_est     = ekf_x[0];
	float res_est   = ekf_x[1];
	float tau_est   = ekf_x[2];
	float omega_est = ekf_x[3];
	float acc_est   = ekf_x[4];

	const bool v_active = (v_est > 0.1f);

	const float normal_res_est = (v_active ? (config.ctrl.resistance_coeff_0
	                           + config.ctrl.resistance_coeff_1 * v_est
	                           + config.ctrl.resistance_coeff_2 * v_est * v_est) : 0.0f);

    if (!v_active){
		res_est = 0.0f;
		acc_est = 0.0f;
	}

	const bool omega_active = (omega_est > 0.1f);

	const float F_human_est = (omega_active && v_active) ? (tau_est * omega_est / v_est) : 0.0f;

	const float acc_next = (F_human_est + F_motor - normal_res_est - res_est) / m;
	float v_next = v_est + dt * acc_est;
	if (v_next < 0.0f) v_next = 0.0f;

	const float x_pred[5] = {v_next, res_est, tau_est, omega_est, acc_next};

	// Jacobian F_jac[row*5+col] = d(x_next[row]) / d(x[col])
	float F_jac[25] = {
		1.0f, 0.0f, 0.0f, 0.0f, dt,
		0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f
	};
	F_jac[4*5+0] = (1.0f / m) * (
		((omega_active && v_active) ? -(tau_est * omega_est) / (v_est * v_est) : 0.0f)
		- config.ctrl.resistance_coeff_1
		- 2.0f * config.ctrl.resistance_coeff_2 * v_est);
	F_jac[4*5+1] = -(1.0f / m);
	F_jac[4*5+2] = (omega_active && v_active) ? (1.0f / m) * (omega_est / v_est) : 0.0f;
	F_jac[4*5+3] = (omega_active && v_active) ? (1.0f / m) * (tau_est  / v_est) : 0.0f;

	// P_pred = F_jac * P * F_jac^T + Q
	float FP[25] = {0.0f};
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 5; j++) {
			for (int k = 0; k < 5; k++) {
				FP[i*5+j] += F_jac[i*5+k] * ekf_P[k*5+j];
			}
		}
	}
	float P_pred[25] = {0.0f};
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 5; j++) {
			for (int k = 0; k < 5; k++) {
				P_pred[i*5+j] += FP[i*5+k] * F_jac[j*5+k];  // F_jac^T[k,j] = F_jac[j,k]
			}
		}
	}
	P_pred[0*5+0] += Q_v;
	P_pred[1*5+1] += Q_res;
	P_pred[2*5+2] += Q_tau;
	P_pred[3*5+3] += Q_omega;
	P_pred[4*5+4] += Q_acc;

	// --- CORRECTION STEP ---
	// H = [[1,0,0,0],[0,0,1,0],[0,0,0,1]]
	// mi[m] = state index for measurement m
	const int mi[3] = {0, 2, 3};

	// S = H*P_pred*H^T + R   (3x3)
	float S[9];
	for (int mr = 0; mr < 3; mr++) {
		for (int nc = 0; nc < 3; nc++) {
			S[mr*3+nc] = P_pred[mi[mr]*5 + mi[nc]];
		}
	}
	S[0*3+0] += R_v;
	S[1*3+1] += R_tau;
	S[2*3+2] += R_omega;

	// Invert S via Cramer's rule
	const float det = S[0]*(S[4]*S[8] - S[5]*S[7])
	                - S[1]*(S[3]*S[8] - S[5]*S[6])
	                + S[2]*(S[3]*S[7] - S[4]*S[6]);
	if (fabsf(det) < 1e-10f) return;  // singular matrix, skip update
	const float inv_det = 1.0f / det;
	float S_inv[9];
	S_inv[0] =  (S[4]*S[8] - S[5]*S[7]) * inv_det;
	S_inv[1] = -(S[1]*S[8] - S[2]*S[7]) * inv_det;
	S_inv[2] =  (S[1]*S[5] - S[2]*S[4]) * inv_det;
	S_inv[3] = -(S[3]*S[8] - S[5]*S[6]) * inv_det;
	S_inv[4] =  (S[0]*S[8] - S[2]*S[6]) * inv_det;
	S_inv[5] = -(S[0]*S[5] - S[2]*S[3]) * inv_det;
	S_inv[6] =  (S[3]*S[7] - S[4]*S[6]) * inv_det;
	S_inv[7] = -(S[0]*S[7] - S[1]*S[6]) * inv_det;
	S_inv[8] =  (S[0]*S[4] - S[1]*S[3]) * inv_det;

	// K = P_pred*H^T * S_inv   (5x3)
	float K[15] = {0.0f};
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				K[i*3+j] += P_pred[i*5 + mi[k]] * S_inv[k*3+j];
			}
		}
	}

	// Innovation: y = z - H*x_pred
	const float y[3] = {
		z_v     - x_pred[mi[0]],
		z_tau   - x_pred[mi[1]],
		z_omega - x_pred[mi[2]]
	};

	// State update: x = x_pred + K*y
	float new_x[5];
	for (int i = 0; i < 5; i++) {
		new_x[i] = x_pred[i];
		for (int j = 0; j < 3; j++) {
			new_x[i] += K[i*3+j] * y[j];
		}
	}

	// Covariance update: P = (I - K*H) * P_pred
	float KH[25] = {0.0f};
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 3; j++) {
			KH[i*5 + mi[j]] += K[i*3+j];
		}
	}
	float new_P[25] = {0.0f};
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 5; j++) {
			for (int k = 0; k < 5; k++) {
				const float IKH_ik = (i == k ? 1.0f : 0.0f) - KH[i*5+k];
				new_P[i*5+j] += IKH_ik * P_pred[k*5+j];
			}
		}
	}

	// Write back updated state and covariance
	for (int i = 0; i < 5; i++) ekf_x[i] = new_x[i];
	for (int i = 0; i < 25; i++) ekf_P[i] = new_P[i];

	// Export estimated (filtered) signals
	bike_speed_estimated   = ekf_x[0]; // [m/s]
	extra_resistance_ekf   = ekf_x[1]; // [N]
	pedal_torque_estimated = ekf_x[2] / config.torque_sensor.nm_max;  // [%]
	pedal_speed_estimated  = ekf_x[3] * 60 / (2.0f * M_PI);           // [rpm]
	bike_accel_estimated   = ekf_x[4]; // [m/s^2]

	// Derivative signals
	wheel_speed_estimated  = bike_speed_estimated * 60.0f / (conf->si_wheel_diameter * M_PI);  // [rpm]
}

static void update_motor_control()
{
	profile_t *profile = get_profile();
	char log_text[64];
	float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	float torque_boosted;
	static uint32_t cnt = 0;

	if (command_line_speed >= 0){
		set_motor_speed(command_line_speed);
		sprintf(log_text, "RPM set to %4.0f", (double)(command_line_speed));
	} 
	else if (clutch_state == CLUTCH_STATE_SYNCING || clutch_state == CLUTCH_STATE_SYNCED || clutch_state == CLUTCH_STATE_OPENING_TMP) {
		float target_speed = MAX((wheel_speed - config.clutch.sync_rpm_diff), 0);
		set_motor_speed(target_speed);
		sprintf(log_text, "RPM set to %4.0f", (double)(target_speed));
	} 
	else if (clutch_state == CLUTCH_STATE_CLOSING || clutch_state == CLUTCH_STATE_CLOSING_TMP) {
		if (config.clutch.sync_while_closing){
			float target_speed = wheel_speed;
			set_motor_speed(target_speed);
			sprintf(log_text, "RPM set to %4.0f", (double)(target_speed));
		} else {
			mc_interface_set_current_rel(0.0);
			sprintf(log_text, "current set to %d%%", 0);
		}
	} 
	else if (clutch_state == CLUTCH_STATE_CLOSED_BRAKE) {
		static float brake_current = 0;
		static systime_t last_time = 0;
		apply_ramping(&brake_current, &last_time, pedal_brake_position_rel, config.back_pedal_brake.current_ramp_time, config.back_pedal_brake.current_ramp_time);
		mc_interface_set_brake_current_rel(brake_current);
		sprintf(log_text, "break current set to %d%%", (int)floor(brake_current*100));
	} 
	else if (clutch_state == CLUTCH_STATE_CLOSED_ASSIST) {
		switch (config.ctrl.ctrl_type){
			case CUSTOM_CTRL_TYPE_NONE:
				break;
			case CUSTOM_CTRL_TYPE_PID:
				set_motor_speed(pedal_speed);
				sprintf(log_text, "RPM set to %4.0f", (double)(pedal_speed));
				break;
			case CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED: 
				mc_interface_set_current_rel(pedal_speed_rel);
				sprintf(log_text, "current set to %d%%", (int)(pedal_speed_rel*100));
				break;
			case CUSTOM_CTRL_TYPE_CURRENT_PEDAL_TORQUE:
				if (config.ctrl.torque_exponent == 1.0f) {
					torque_boosted = pedal_torque_filtered_rel;
				} else {
					torque_boosted = pedal_torque_filtered_rel > 0 ? expf(config.ctrl.torque_exponent * logf(pedal_torque_filtered_rel)) : 0;
					// TODO: speed up with look-up table
				}
	    		motor_current_rel = (pedal_speed >= config.pedal_sensor.rpm_start && pedal_torque_filtered_rel > 0) ? (profile->torque_base_gain * torque_boosted) : 0;
				utils_truncate_number((float*)&motor_current_rel, 0.0, 1.0);
				plot_points(PLOT_MOTOR_CURRENT, timestamp, motor_current_rel*100);
				mc_interface_set_current_rel(motor_current_rel);
				break;
			case CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED_AND_TORQUE:
				if (config.ctrl.torque_exponent == 1.0f) {
					torque_boosted = pedal_torque_filtered_rel;
				} else {
					torque_boosted = pedal_torque_filtered_rel > 0 ? expf(config.ctrl.torque_exponent * logf(pedal_torque_filtered_rel)) : 0;
					// TODO: speed up with look-up table
				}
				motor_current_rel = (pedal_speed >= config.pedal_sensor.rpm_start && pedal_torque_filtered_rel > 0) ? (profile->torque_base_gain * (torque_boosted + config.ctrl.cadence_gain * pedal_speed_rel * torque_boosted)/2) : 0;
				utils_truncate_number((float*)&motor_current_rel, 0.0, 1.0);
				plot_points(PLOT_MOTOR_CURRENT, timestamp, motor_current_rel*100);
				mc_interface_set_current_rel(motor_current_rel);
				break;
			case CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED_AND_TORQUE_AUTO:
				if (config.ctrl.torque_exponent == 1.0f) {
					torque_boosted = pedal_torque_filtered_rel;
				} else {
					torque_boosted = pedal_torque_filtered_rel > 0 ? expf(config.ctrl.torque_exponent * logf(pedal_torque_filtered_rel)) : 0;
					// TODO: speed up with look-up table
				}
				motor_current_rel = (pedal_speed >= config.pedal_sensor.rpm_start && pedal_torque_filtered_rel > 0) ? (torque_gain * (torque_boosted + config.ctrl.cadence_gain * pedal_speed_rel * torque_boosted)/2) : 0;
				utils_truncate_number((float*)&motor_current_rel, 0.0, 1.0);
				plot_points(PLOT_MOTOR_CURRENT, timestamp, motor_current_rel*100);
				mc_interface_set_current_rel(motor_current_rel);
				break;
			default: 
				break;
		}
	} else {
		mc_interface_set_current_rel(0.0);
		sprintf(log_text, "current set to %d%%", 0);
	}

	if (cnt++ % (config.update_rate_hz / 10) == 0){
		print_log(LOG_GROUP_MOTOR,"[%4.2f] %s", (double)timestamp, log_text);
	}
}

static float notch_filter(float new_value, float *memory, float timeout, bool dual_mode) {
	//Filtering cyclic variations - caused by pedal physics - by removing estimated periodic component
	float A = memory[0];
	float B = memory[1];
	int   index = (int)memory[2];
	float inactivity_time = memory[3];
	float last_filtered = memory[4];
	float C = memory[5];
	float D = memory[6];
	uint8_t filter_size = config.pedal_sensor.magnets * 4;
	const float mu = 0.1;
	const float mu2 = 0.005;
	float filtered = 0;
	float filtered_tmp = 0;
	float y_estimated = 0;

	if (pedal_current_direction == 1) {
		float x1 = sin_lut[(index * 2) % filter_size];
		float x2 = sin_lut[(filter_size / 4 + index * 2) % filter_size];
		float x3 = sin_lut[index % filter_size];
    	float x4 = sin_lut[(filter_size / 4 + index) % filter_size];

		// Estimate next sample
		y_estimated = (A * x1) + (B * x2);

		// Calculate error.
		// This is also the filtered value (periodic component removed from raw value)
		filtered_tmp = new_value - y_estimated;

		// Update estimator params
		A = A + (mu * filtered_tmp * x1);
    	B = B + (mu * filtered_tmp * x2);

		if (dual_mode) {
			// Estimate next sample
			y_estimated = (C * x3) + (D * x4);

			// Calculate error.
			filtered = filtered_tmp - y_estimated;

			// Update estimator params
			C = C + (mu2 * filtered * x3);
			D = D + (mu2 * filtered * x4);
		} else {
			filtered = filtered_tmp;
		}

		// Advance phase
		index++;
		if (index >= filter_size) {
			index = 0;
		}
		inactivity_time = 0;

		// Compensate overshooting
		filtered *= 0.947;
	} else if (pedal_current_direction == -1) {
		// reset samples when changing direction to avoid applying average of one direction to the other direction
		A = 0;
		B = 0;
		C = 0;
		D = 0;
		index = 0;
		inactivity_time = 0;
		filtered = new_value;
	} else {
		inactivity_time += 1.0 / config.update_rate_hz;
		if (inactivity_time > timeout) {
			inactivity_time = timeout;
			A = 0;
			B = 0;
			C = 0;
			D = 0;
			index = 0;
			filtered = new_value;
		} else {
			// no movement, keep previous filtered value
			filtered = last_filtered;
		}
	}

	memory[0] = A;
	memory[1] = B;
	memory[2] = index;
	memory[3] = inactivity_time;
	memory[4] = filtered;
	memory[5] = C;
	memory[6] = D;
	return filtered;
}

static float biquad_filter(float new_value, float *memory, float cutoff_freq, bool derivator)
{
	// Matrix Columns: {b0, b1, b2, a1, a2}
	const float lpf_matrix[4][5] = {
		{0.000009825917, 0.000019651834, 0.000009825917, -1.991114292202, 0.991153595869}, // 0.5 Hz
		{0.000039130205, 0.000078260411, 0.000039130205, -1.982228929793, 0.982385450614}, // 1.0 Hz
		{0.000155148422, 0.000310296845, 0.000155148422, -1.964460580205, 0.965081173895}, // 2.0 Hz
		{0.000609854721, 0.001219709442, 0.000609854721, -1.928942259604, 0.931381678488}  // 4.0 Hz
	};
	// Matrix Columns: {b0, b1, b2, a1, a2}
	// Note: b1 is mathematically 0.0 and b2 is exactly -b0
	const float lpf_derivator_matrix[4][5] = {
		{0.009825916820, 0.0, -0.009825916820, -1.991114292202, 0.991153595869}, // 0.5 Hz
		{0.039130205399, 0.0, -0.039130205399, -1.982228929793, 0.982385450614}, // 1.0 Hz
		{0.155148422340, 0.0, -0.155148422340, -1.964460580205, 0.965081173895}, // 2.0 Hz
		{0.609854721182, 0.0, -0.609854721182, -1.928942259604, 0.931381678488}  // 4.0 Hz
	};
	float y;
	float b0, b1, b2;
	float a1, a2;
	int row = 0;

	if (cutoff_freq == 0.5f){
		row = 0;
	}
	else if (cutoff_freq == 1.0f){
		row = 1;
	}
	else if (cutoff_freq == 2.0f){
		row = 2;
	}
	else if (cutoff_freq == 4.0f){
		row = 3;
		}
	else {
		return 0;
	}

	if (derivator) {
		b0 = lpf_derivator_matrix[row][0];
		b1 = lpf_derivator_matrix[row][1];
		b2 = lpf_derivator_matrix[row][2];
		a1 = lpf_derivator_matrix[row][3];
		a2 = lpf_derivator_matrix[row][4];
		} else {
		b0 = lpf_matrix[row][0];
		b1 = lpf_matrix[row][1];
		b2 = lpf_matrix[row][2];
		a1 = lpf_matrix[row][3];
		a2 = lpf_matrix[row][4];
	}

	y = b0 * new_value +   // b0 * x[n]
		b1 * memory[0] +   // b1 * x[n-1]
		b2 * memory[1] -   // b2 * x[n-2]
		a1 * memory[2] -   // a1 * y[n-1]
		a2 * memory[3];    // a2 * y[n-2]

	memory[1] = memory[0];
	memory[0] = new_value;
	memory[3] = memory[2];
	memory[2] = y;

	return y;
}

// static void calibrate_wheel_sensor(float last_wheel_speed, float last_motor_speed) {
// 	// wait until motor is spinning up to start calibration
// 	if (calibration_active && calibration_step == 0 && abs(last_wheel_speed - config.wheel_sensor.calibration_rpm) < 1.0) {
// 		for (uint8_t i = 0; i < config.wheel_sensor.magnets; i++) {
// 			wheel_sensor_calibration_values[i] = 0;
// 		}
// 		wheel_sensor_calibration_values[0] += last_motor_speed / last_wheel_speed / CALIBRATION_ROUNDS;
// 		calibration_step = 1;
// 	}
// 	else
// 	// collect calibration values for each magnet (multiple rounds if configured) and average them
// 	if (calibration_active && calibration_step > 0 && calibration_step < CALIBRATION_ROUNDS * config.wheel_sensor.magnets) {
// 		wheel_sensor_calibration_values[calibration_step % config.wheel_sensor.magnets] += last_motor_speed / last_wheel_speed / CALIBRATION_ROUNDS;
// 		calibration_step++;
// 	}

// 	if (calibration_active && calibration_step >= CALIBRATION_ROUNDS * config.wheel_sensor.magnets) {
// 		calibration_active = false;
// 		command_line_speed = -1;
// 		printf("Wheel sensor calibration completed\n");
// 		for (uint8_t i = 0; i < config.wheel_sensor.magnets; i++) {
// 			print_log(LOG_GROUP_SENSOR,"Calibration value for magnet %d: %4.2f\n", i, (double)wheel_sensor_calibration_values[i]);
// 		}
// 	}
// }

// static float compensate_wheel_sensor(float last_wheel_speed, float last_motor_speed) {
// 	float diff;
// 	float min_diff = 10;
// 	uint8_t new_magnet_cntr = 0;

// 	// store the last speeds for each magnet to be able to detect the pattern
// 	for (uint8_t i = 0; i < config.wheel_sensor.magnets - 1; i++) {
// 		last_wheel_speeds[i] = last_wheel_speeds[i+1];
// 		last_motor_speeds[i] = last_motor_speeds[i+1];
// 	}
// 	last_wheel_speeds[config.wheel_sensor.magnets - 1] = last_wheel_speed; // store the uncompensated speed
// 	last_motor_speeds[config.wheel_sensor.magnets - 1] = last_motor_speed; // store the motorspeed for reference

// 	// if the wheel is not spinning fast enough, we won't apply any calibration
// 	if (last_wheel_speed < config.wheel_sensor.rpm_min) {
// 		compensation_active = false;
// 		wheel_sensor_magnet_cntr = 0;
// 		return last_wheel_speed;
// 	}

// 	// if calibration is active or we don't have any calibration values yet, we won't apply any calibration
// 	if (calibration_active || wheel_sensor_calibration_values[0] == 0) {
// 		compensation_active = false;
// 		wheel_sensor_magnet_cntr = 0;
// 		return last_wheel_speed;
// 	}

// 	// if motor runs together with the wheel, we can try to detect which magnet is currently triggering the sensor
// 	if (clutch_state == CLUTCH_STATE_SYNCED || clutch_state == CLUTCH_STATE_CLOSED_ASSIST || clutch_state == CLUTCH_STATE_CLOSED_BRAKE || clutch_state == CLUTCH_STATE_CLOSED_FLOAT) {
// 		// do this only once per turn of the wheel to avoid excessive calculations
// 		if (wheel_sensor_magnet_cntr == 0) {
// 			for (uint8_t i = 0; i < config.wheel_sensor.magnets; i++) {
// 				diff = 0;
// 				for (uint8_t j = 0; j < config.wheel_sensor.magnets; j++) {
// 					diff += (float)fabs((double)(last_motor_speeds[(i + j) % config.wheel_sensor.magnets] / last_wheel_speeds[(i + j) % config.wheel_sensor.magnets] - wheel_sensor_calibration_values[j]));
// 				}
// 				//print_log(LOG_GROUP_SENSOR,"%d -> %f\n", i, (double)diff);
// 				if (diff < (DIFF_THRESHOLD_TO_APPLY_COMPENSATION * config.wheel_sensor.magnets) && diff < min_diff) {
// 					min_diff = diff;
// 					new_magnet_cntr = (config.wheel_sensor.magnets - 1 - i) % config.wheel_sensor.magnets;
// 					compensation_active = true;
// 				}
// 			}
// 			if (new_magnet_cntr != wheel_sensor_magnet_cntr) {
// 				uint8_t shift = (new_magnet_cntr - wheel_sensor_magnet_cntr + config.wheel_sensor.magnets) % config.wheel_sensor.magnets;
// 				print_log(LOG_GROUP_SENSOR,"magnetshift: %d\n", shift > (config.wheel_sensor.magnets / 2) ? shift - config.wheel_sensor.magnets : shift);
// 				wheel_sensor_magnet_cntr = new_magnet_cntr;
// 			}
// 		}
// 	}

// 	// apply calibration value for the currently active magnet
// 	if (compensation_active) {
// 		float compensated_wheel_speed;
// 		compensated_wheel_speed = last_wheel_speed * wheel_sensor_calibration_values[wheel_sensor_magnet_cntr];
// 		wheel_sensor_magnet_cntr = (wheel_sensor_magnet_cntr + 1) % config.wheel_sensor.magnets;
// 		return compensated_wheel_speed;
// 	}

// 	return last_wheel_speed;
// }

static void record_clutch_operation(void)
{
	float current_time = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	
	// Add current operation to buffer
	clutch_operation_timestamps[clutch_operation_buffer_index] = current_time;
	clutch_operation_buffer_index = (clutch_operation_buffer_index + 1) % CLUTCH_OPERATION_BUFFER_SIZE;
	
	if (clutch_operation_count < CLUTCH_OPERATION_BUFFER_SIZE) {
		clutch_operation_count++;
	}
	
	// Count operations in the last N seconds
	uint32_t operations_in_last_n_seconds = 0;
	for (uint32_t i = 0; i < clutch_operation_count; i++) {
		if ((current_time - clutch_operation_timestamps[i]) <= config.clutch.error_period) {
			operations_in_last_n_seconds++;
		}
	}
	
	// Log error if more than M operations in the last N seconds
	if (operations_in_last_n_seconds > config.clutch.error_limit) {
		print_log(LOG_GROUP_CLUTCH, "[%4.2f] EXCESSIVE CLUTCH OPERATIONS: %d operations in last %4.0f seconds!", 
				  (double)current_time, operations_in_last_n_seconds, (double)config.clutch.error_period);
		new_clutch_state(CLUTCH_STATE_ERROR);
	}
}

static void open_clutch(void)
{
	if (config.clutch.mode != CLUTCH_MODE_CLOSED){ 
		record_clutch_operation();
		palWritePad(APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1, APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1, config.clutch.invert_direction ? 0 : 1);
		clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		clutch_state = CLUTCH_STATE_OPENING;
		print_log(LOG_GROUP_CLUTCH,"[%4.2f] OPENING...", (double)clutch_timestamp);
	}
}

static void sync_clutch(void)
{
	if (config.clutch.mode != CLUTCH_MODE_OPEN){ 
		clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		clutch_state = CLUTCH_STATE_SYNCING;
		print_log(LOG_GROUP_CLUTCH,"[%4.2f] SYNCING...", (double)clutch_timestamp);
	}
}

static void close_clutch(void)
{
	if (config.clutch.mode != CLUTCH_MODE_OPEN){ 
		record_clutch_operation();
		palWritePad(APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1, APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1, config.clutch.invert_direction ? 1 : 0);
		clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		clutch_state = CLUTCH_STATE_CLOSING;
		print_log(LOG_GROUP_CLUTCH,"[%4.2f] CLOSING...", (double)clutch_timestamp);
	}
}

static void limit_current(void)
{
	mc_configuration *mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();
	mcconf->l_current_max_scale = config.clutch.current_limit_closing;
	mcconf->l_current_min_scale = config.clutch.current_limit_closing;
	mc_interface_set_configuration(mcconf);
	mempools_free_mcconf(mcconf);
}

static void remove_current_limit(void)
{
	mc_configuration *mcconf = mempools_alloc_mcconf();
	*mcconf = *mc_interface_get_configuration();
	mcconf->l_current_max_scale = 1.0;
	mcconf->l_current_min_scale = 1.0;
	mc_interface_set_configuration(mcconf);
	mempools_free_mcconf(mcconf);
}

static void new_clutch_state(clutch_state_type cs)
{
	if (clutch_state == CLUTCH_STATE_CLOSING || clutch_state == CLUTCH_STATE_CLOSING_TMP) {
		remove_current_limit();
		last_close_time = APP_NOW_SEC;
	}

	if (cs == CLUTCH_STATE_OPENING) {
		open_clutch();
	} else 
	if (cs == CLUTCH_STATE_SYNCING) {
		sync_clutch();
	} else
	if (cs == CLUTCH_STATE_CLOSING) {
		limit_current();
		close_clutch();
	} else 
	if (cs == CLUTCH_STATE_OPENING_TMP) {
		open_clutch();
		clutch_state = CLUTCH_STATE_OPENING_TMP;
	} else
	if (cs == CLUTCH_STATE_CLOSING_TMP) {
		limit_current();
		close_clutch();
		clutch_state = CLUTCH_STATE_CLOSING_TMP;
	} else {
		char *clutch_state_str;
		switch (cs) {
			case CLUTCH_STATE_OPEN: clutch_state_str = "OPEN"; break;
			case CLUTCH_STATE_OPEN_ERROR: clutch_state_str = "OPEN (ERROR)"; clutch_open_error_counter++; break;
			case CLUTCH_STATE_OPENING: clutch_state_str = "OPENING"; break;
			case CLUTCH_STATE_OPENING_TMP: clutch_state_str = "OPENING TEMPORARILY"; break;
			case CLUTCH_STATE_WAITING: clutch_state_str = "WAITING"; break;
			case CLUTCH_STATE_SYNCING: clutch_state_str = "SYNCING"; break;
			case CLUTCH_STATE_SYNCED: clutch_state_str = "SYNCED"; break;
			case CLUTCH_STATE_CLOSING: clutch_state_str = "CLOSING"; break;
			case CLUTCH_STATE_CLOSING_TMP: clutch_state_str = "CLOSING TEMPORARILY"; break;
			case CLUTCH_STATE_CLOSED_FLOAT: clutch_state_str = "CLOSED (FLOAT)"; break;
			case CLUTCH_STATE_CLOSED_BRAKE: clutch_state_str = "CLOSED (BRAKE)"; break;
			case CLUTCH_STATE_CLOSED_ASSIST: clutch_state_str = "CLOSED (ASSIST)"; break;
			case CLUTCH_STATE_CLOSED_ERROR: clutch_state_str = "CLOSED (ERROR)"; clutch_close_error_counter++; break;
			case CLUTCH_STATE_ERROR: clutch_state_str = "ERROR"; break;
			default: clutch_state_str = "UNKNOWN"; break;
		}
	    clutch_timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	    clutch_state = cs;
	    print_log(LOG_GROUP_CLUTCH,"[%4.2f] CLUTCH %s", (double)clutch_timestamp, clutch_state_str);
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
	if (plots_enabled & (1 << PLOT_WHEEL_PRED_RPM)) {
		plot_numbers[PLOT_WHEEL_PRED_RPM] = plot_number++;
		commands_plot_add_graph("Predicted Wheel RPM");
	}
	if (plots_enabled & (1 << PLOT_TORQUE)) {
		plot_numbers[PLOT_TORQUE] = plot_number++;
		commands_plot_add_graph("Pedal Torque");
	}
	if (plots_enabled & (1 << PLOT_TORQUE2)) {
		plot_numbers[PLOT_TORQUE2] = plot_number++;
		commands_plot_add_graph("Pedal Torque 2");
	}
	if (plots_enabled & (1 << PLOT_MOTOR_CURRENT)) {
		plot_numbers[PLOT_MOTOR_CURRENT] = plot_number++;
		commands_plot_add_graph("Motor Current");
	}
	if (plots_enabled & (1 << PLOT_ASSIST_LEVEL)) {
		plot_numbers[PLOT_ASSIST_LEVEL] = plot_number++;
		commands_plot_add_graph("Assist Level");
	}
	if (plots_enabled & (1 << PLOT_ACCEL)) {
		plot_numbers[PLOT_ACCEL] = plot_number++;
		commands_plot_add_graph("Acceleration (m/s^2)");
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
	va_start(arg, format);

	if (log_groups_enabled & (1 << log_group)) {
		char buf[256];
		vsnprintf(buf, sizeof(buf), format, arg);
		commands_printf("%s", buf);
	}
	va_end(arg);
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

static uint8_t is_valid_start_byte(uint8_t b)
{
    return (b == PT_START) || (b == PT_READ) || (b == PT_WRITE);
}
 
static uint8_t get_data_len(uint8_t packet_type, uint8_t msg_type)
{
    if (packet_type == PT_WRITE) {
        if (msg_type == MT_PAS_LEVEL) return 1;
        if (msg_type == MT_UNKNOWN_1F) return 2;
    }
    return 0;
}

static void dispatch_packet(uint8_t packet_type, uint8_t msg_type,
                             const uint8_t *data, uint8_t len, uart_error_t error)
{
	(void)len;

    switch (msg_type) {
        case MT_START:        
			print_log(LOG_GROUP_UART, "pkt: start"); 															
			//handle_start(packet_type);                   
			break;
        case MT_VERSION:      
			print_log(LOG_GROUP_UART, "pkt: %s version %s", 
				(packet_type == PT_WRITE) ? "write" : "read", 
				(error == UART_ERROR_NONE) ? "" : (error == UART_ERROR_CHECKSUM_MISSING) ? "[checksum missing]" : "[checksum invalid]"); 
			//handle_version(packet_type, data, len);      
			break;
        case MT_PAS_LEVEL:    
			print_log(LOG_GROUP_UART, "pkt: %s pas_level: %02X", 
				(packet_type == PT_WRITE) ? "write" : "read", data[0],
				(error == UART_ERROR_NONE) ? "" : (error == UART_ERROR_CHECKSUM_MISSING) ? "[checksum missing]" : "[checksum invalid]"); 	
			    switch (data[0]) {
					case PAS_LEVEL_0: profile_to_use = 0; break;
					case PAS_LEVEL_1: profile_to_use = 1; break;
					case PAS_LEVEL_2: profile_to_use = 2; break;
					case PAS_LEVEL_3: profile_to_use = 3; break;
					case PAS_LEVEL_4: profile_to_use = 4; break;
					case PAS_LEVEL_5: profile_to_use = 5; break;
					//TODO: handle PAS_LEVEL_WALK
					default: profile_to_use = 0; break;
				}
			break;
        case MT_SPEED:        
			print_log(LOG_GROUP_UART, "pkt: %s speed %s", 
				(packet_type == PT_WRITE) ? "write" : "read", 
				(error == UART_ERROR_NONE) ? "" : (error == UART_ERROR_CHECKSUM_MISSING) ? "[checksum missing]" : "[checksum invalid]"); 		
			//handle_speed(packet_type, data, len);        
			break;
        case MT_BATTERY_SOC:  
			print_log(LOG_GROUP_UART, "pkt: %s battery_soc %s", 
				(packet_type == PT_WRITE) ? "write" : "read", 
				(error == UART_ERROR_NONE) ? "" : (error == UART_ERROR_CHECKSUM_MISSING) ? "[checksum missing]" : "[checksum invalid]"); 	
			//handle_battery_soc(packet_type, data, len);  
			break;
        case MT_MOVING:       
			print_log(LOG_GROUP_UART, "pkt: %s moving %s", 
				(packet_type == PT_WRITE) ? "write" : "read", 
				(error == UART_ERROR_NONE) ? "" : (error == UART_ERROR_CHECKSUM_MISSING) ? "[checksum missing]" : "[checksum invalid]"); 		
			//handle_moving(packet_type, data, len);       
			break;
        case MT_WHEEL_RPM:    
			print_log(LOG_GROUP_UART, "pkt: %s wheel_rpm %s", 
				(packet_type == PT_WRITE) ? "write" : "read", 
				(error == UART_ERROR_NONE) ? "" : (error == UART_ERROR_CHECKSUM_MISSING) ? "[checksum missing]" : "[checksum invalid]"); 	
			//handle_wheel_rpm(packet_type, data, len);    
			break;
        case MT_PEDAL_MOVING: 
			print_log(LOG_GROUP_UART, "pkt: %s pedal_moving %s", 
				(packet_type == PT_WRITE) ? "write" : "read", 
				(error == UART_ERROR_NONE) ? "" : (error == UART_ERROR_CHECKSUM_MISSING) ? "[checksum missing]" : "[checksum invalid]"); 	
			//handle_pedal_moving(packet_type, data, len); 
			break;
        case MT_AMPERES:      
			print_log(LOG_GROUP_UART, "pkt: %s amperes %s", 
				(packet_type == PT_WRITE) ? "write" : "read", 
				(error == UART_ERROR_NONE) ? "" : (error == UART_ERROR_CHECKSUM_MISSING) ? "[checksum missing]" : "[checksum invalid]"); 
			//handle_amperes(packet_type, data, len);      
			break;
        default:
            print_log(LOG_GROUP_UART, "pkt: %s unknown (%02X) %s", 
				(packet_type == PT_WRITE) ? "write" : "read", 
				msg_type,
				(error == UART_ERROR_NONE) ? "" : (error == UART_ERROR_CHECKSUM_MISSING) ? "[checksum missing]" : "[checksum invalid]"); 
			for (uint8_t i = 0; i < len; i++) {
				print_log(LOG_GROUP_UART, "  data[%d]: %02X", i, data[i]);
			}
			//handle_unknown(packet_type, msg_type, data, len);
            break;
    }
}

static void uart_parser_feed(uint8_t byte)
{
	static parser_state_t state = ST_WAIT_START; 
	static uint8_t cur_packet_type;
	static uint8_t cur_msg_type;
	static uint8_t cur_data[MAX_UART_DATA_LEN];
	static uint8_t cur_data_len;
	static uint8_t cur_data_idx;
	static uint8_t cur_checksum_acc;

    switch (state) {
 
    case ST_WAIT_START:
        if (is_valid_start_byte(byte)) {
            cur_packet_type   = byte;
            cur_checksum_acc  = byte;
            state = ST_WAIT_TYPE;
        }
        /* else: stray byte, stay in ST_WAIT_START */
        break;
 
    case ST_WAIT_TYPE:
        cur_msg_type      = byte;
        cur_checksum_acc  = (uint8_t)(cur_checksum_acc + byte);
        cur_data_len      = get_data_len(cur_packet_type, cur_msg_type);
        cur_data_idx      = 0;
        state = (cur_data_len == 0) ? ST_WAIT_CHECKSUM : ST_WAIT_DATA;
        break;
 
    case ST_WAIT_DATA:
        cur_data[cur_data_idx++] = byte;
        cur_checksum_acc = (uint8_t)(cur_checksum_acc + byte);
        if (cur_data_idx >= cur_data_len) {
            state = ST_WAIT_CHECKSUM;
        }
        break;
 
    case ST_WAIT_CHECKSUM:
        if (byte == cur_checksum_acc) {
            /* valid packet -> dispatch */
            dispatch_packet(cur_packet_type, cur_msg_type, cur_data, cur_data_len, UART_ERROR_NONE);
            state = ST_WAIT_START;
        } else if (is_valid_start_byte(byte)) {
            /* checksum missing: current packet dropped, this byte
               is the start of the next packet */
			dispatch_packet(cur_packet_type, cur_msg_type, cur_data, cur_data_len, UART_ERROR_CHECKSUM_MISSING);
            cur_packet_type  = byte;
            cur_checksum_acc = byte;
            state = ST_WAIT_TYPE;
        } else {
            /* wrong checksum: current packet dropped, byte consumed */
            dispatch_packet(cur_packet_type, cur_msg_type, cur_data, cur_data_len, UART_ERROR_CHECKSUM_INVALID);
            state = ST_WAIT_START;
        }
        break;
    }
}