/*
	Copyright 2020 Benjamin Vedder	benjamin@vedder.se

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

#ifndef APP_REGEN_TYPES_H_
#define APP_REGEN_TYPES_H_

typedef enum {
    LOG_GROUP_SENSOR = 0,
    LOG_GROUP_MOTOR,
    LOG_GROUP_CLUTCH,
    LOG_GROUP_ERROR,
    NUM_LOG_GROUPS
} log_group_t;

// Enum for plot indices
typedef enum {
    PLOT_PEDAL_RPM,
    PLOT_BRAKE_POS,
    PLOT_WHEEL_RPM,
    PLOT_HALL1,
    PLOT_HALL2,
    PLOT_HALL3,
    PLOT_MOTOR_RPM,
    PLOT_CLUTCH_STATE,
    PLOT_WHEEL_PRED_RPM,
    PLOT_TORQUE,
    PLOT_COUNT // This should always be the last element
} plot_index_t;

// ADC control types
typedef enum {
	CUSTOM_CTRL_TYPE_NONE = 0,
	CUSTOM_CTRL_TYPE_PID,
	CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED,
    CUSTOM_CTRL_TYPE_CURRENT_PEDAL_TORQUE,
    CUSTOM_CTRL_TYPE_CURRENT_PEDAL_SPEED_AND_TORQUE
} custom_control_type;

typedef enum {
    CLUTCH_STATE_OPEN = 0,
    CLUTCH_STATE_OPEN_ERROR = 5,
    CLUTCH_STATE_OPENING = 10,
    CLUTCH_STATE_OPENING_TMP = 12,
    CLUTCH_STATE_WAITING = 15,
    CLUTCH_STATE_SYNCING = 20,
    CLUTCH_STATE_SYNCED = 25,
    CLUTCH_STATE_CLOSING = 30,
    CLUTCH_STATE_CLOSING_TMP = 35,
    CLUTCH_STATE_CLOSED_FLOAT = 40,
    CLUTCH_STATE_CLOSED_BRAKE = 45,
    CLUTCH_STATE_CLOSED_ASSIST = 50,
    CLUTCH_STATE_CLOSED_ERROR = 60,
    CLUTCH_STATE_ERROR = 80
} clutch_state_type;

typedef enum {
    CLUTCH_MODE_CLOSED = 0,
    CLUTCH_MODE_OPEN,
    CLUTCH_MODE_AUTO,
    CLUTCH_MODE_MANUAL,
    CLUTCH_MODE_FULL_MANUAL
} clutch_mode_type;

typedef enum {
    SPEED_SENSOR_TYPE_SINGLE_POLL = 0,
    SPEED_SENSOR_TYPE_SINGLE_INTERRUPT,
	SPEED_SENSOR_TYPE_QUADRATURE_POLL,
    SPEED_SENSOR_TYPE_QUADRATURE_INTERRUPT,
    SPEED_SENSOR_TYPE_SINGLE_POLL_SINGLE_INTERRUPT
} speed_sensor_type;

typedef enum {
    TORQUE_SENSOR_TYPE_NONE = 0,
    TORQUE_SENSOR_TYPE_ADC
} torque_sensor_type;

typedef struct {
    speed_sensor_type sensor_type;
    float poll_to_int_rpm;
    uint8_t magnets; 
	float filter;
    float avg_above_rpm;
    float rpm_min;
	float rpm_start;
	float rpm_end;
    float rpm_max;
	float ramp_time_pos;
	float ramp_time_neg;
	bool  invert_direction;
    float skipped_magnet_threshold;
    float calibration_rpm;
} speed_sensor_config_type;

typedef struct {
    torque_sensor_type sensor_type;
	bool use_filter;
} torque_sensor_config_type;

typedef struct {
    float start_pos;
    float end_pos;
    float release_rpm;
    float wait_before_release;
    float sync_start_pos;
    float current_ramp_time;
    float reset_pos_percent;
} brake_config_type;

typedef struct {
    clutch_mode_type mode;
    float wait_before_open;
    float wait_before_sync;
    float wait_before_check;
    float wait_before_sync_loss;
    float sync_time;
    float desync_time;
    float sync_timeout;
    float closed_first_check_time;
    float sync_rpm_diff;
    float first_check_rpm_diff;
    float closed_check_rpm_diff;
    float open_check_rpm_diff;
    float min_rpm_open;
    float min_rpm_close;
    float max_rpm_open;
    float max_rpm_close;
    bool invert_direction;
    uint32_t error_limit;
    float error_period;
    bool sync_while_closing;
    float current_limit_closing;
} clutch_config_type;

typedef struct {
	custom_control_type ctrl_type;
    speed_sensor_config_type pedal_sensor;
    speed_sensor_config_type wheel_sensor;
    torque_sensor_config_type torque_sensor;
    brake_config_type back_pedal_brake;
    clutch_config_type clutch;
	//float current_scaling;
	uint32_t update_rate_hz;
} custom_config_type;

// Config parameter types
typedef enum {
    CONFIG_TYPE_FLOAT,
    CONFIG_TYPE_UINT32,
    CONFIG_TYPE_BOOL,
    CONFIG_TYPE_ENUM
} config_param_type_t;

// Config parameter structure
typedef struct {
    const char* name;                // Terminal command name
    const char* description;         // Help text description
    config_param_type_t type;        // Parameter type
    void* config_ptr;                // Pointer to config field
    uint16_t eeprom_addr;            // EEPROM address
    union {
        float float_default;
        uint32_t uint32_default;
        bool bool_default;
        uint32_t enum_default;
    } default_value;
    const char* enum_values;         // For enum types, comma-separated values
} config_param_t;

#endif /* APP_REGEN_TYPES_H_ */