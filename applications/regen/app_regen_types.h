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
    CLUTCH_STATE_DISCONNECTED = 0,
    CLUTCH_STATE_SYNCHRONIZING,
    CLUTCH_STATE_CONNECTED
} clutch_state_type;

typedef enum {
    SPEED_SENSOR_TYPE_NONE = 0,
    SPEED_SENSOR_TYPE_SINGLE,
	SPEED_SENSOR_TYPE_QUADRATURE
} speed_sensor_type;

typedef struct {
    speed_sensor_type sensor_type;
    uint8_t magnets; 
	bool use_filter;
	float rpm_start;
	float rpm_end;
	float ramp_time_pos;
	float ramp_time_neg;
	//bool invert_direction;
} speed_sensor_config_type;

typedef struct {
    float start_pos;
    float end_pos;
} brake_config_type;

typedef struct {
	//pas_control_type ctrl_type;
    speed_sensor_config_type pedal_sensor;
    speed_sensor_config_type wheel_sensor;
    brake_config_type back_pedal_brake;
	//float current_scaling;
	uint32_t update_rate_hz;
} custom_config_type;

#endif /* APP_REGEN_TYPES_H_ */