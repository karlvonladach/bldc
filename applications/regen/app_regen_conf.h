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

#ifndef APP_REGEN_CONF_H_
#define APP_REGEN_CONF_H_

#define DEBUG_PLOT

#define APP_CUSTOM_TO_USE				"regen/app_regen.c"

//uncomment to use custom app by default, regardless of settings:
//#define APPCONF_APP_TO_USE				APP_CUSTOM

#define APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE        SPEED_SENSOR_TYPE_QUADRATURE
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1       HW_UART_RX_PORT
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PIN1        HW_UART_RX_PIN
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PORT2       HW_UART_TX_PORT
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PIN2        HW_UART_TX_PIN
#define APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS     8u
#define APP_CUSTOM_CONF_PEDAL_SENSOR_USE_FILTER  1
#define APP_CUSTOM_CONF_PEDAL_RPM_START          10.0f
#define APP_CUSTOM_CONF_PEDAL_RPM_END            120.0f
#define APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS      0.6f
#define APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG      0.3f

#define APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE        SPEED_SENSOR_TYPE_SINGLE
#define APP_CUSTOM_CONF_WHEEL_SENSOR_PORT1       HW_HALL_ENC_GPIO1
#define APP_CUSTOM_CONF_WHEEL_SENSOR_PIN1        HW_HALL_ENC_PIN1
#define APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS     8u
#define APP_CUSTOM_CONF_WHEEL_SENSOR_USE_FILTER  1
#define APP_CUSTOM_CONF_WHEEL_RPM_START          50.0f
#define APP_CUSTOM_CONF_WHEEL_RPM_END            600.0f
#define APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS      0.6f
#define APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG      0.3f

#define APP_CUSTOM_CONF_UPDATE_RATE_HZ           500

#endif /* APP_REGEN_CONF_H_ */