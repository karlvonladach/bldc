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
    CLUTCH_STATE_OPENING = 0,
    CLUTCH_STATE_OPEN,
    CLUTCH_STATE_SYNCING,
    CLUTCH_STATE_SYNCED,
    CLUTCH_STATE_CLOSING,
    CLUTCH_STATE_CLOSED,
} clutch_state_type;

typedef enum {
    CLUTCH_MODE_CLOSED = 0,
    CLUTCH_MODE_OPEN,
    CLUTCH_MODE_AUTO,
    CLUTCH_MODE_MANUAL
} clutch_mode_type;

typedef enum {
    SPEED_SENSOR_TYPE_NONE = 0,
    SPEED_SENSOR_TYPE_SINGLE_INTERRUPT,
    SPEED_SENSOR_TYPE_SINGLE_POLL,
    SPEED_SENSOR_TYPE_QUADRATURE_INTERRUPT,
	SPEED_SENSOR_TYPE_QUADRATURE_POLL
} speed_sensor_type;

typedef enum {
    TORQUE_SENSOR_TYPE_NONE = 0,
    TORQUE_SENSOR_TYPE_ADC
} torque_sensor_type;

typedef struct {
    speed_sensor_type sensor_type;
    uint8_t magnets; 
	bool use_filter;
    float rpm_min;
	float rpm_start;
	float rpm_end;
    float rpm_max;
	float ramp_time_pos;
	float ramp_time_neg;
	bool invert_direction;
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
} brake_config_type;

typedef struct {
    clutch_mode_type mode;
    float wait_before_open;
    float wait_before_sync;
    float wait_before_check;
    float sync_rpm_diff;
    float check_rpm_diff;
    float min_rpm;
    float max_rpm_open;
    float max_rpm_close;
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

#endif /* APP_REGEN_TYPES_H_ */