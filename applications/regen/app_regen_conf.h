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

#include "app_regen_types.h"

//#define DEBUG_PLOT

#define APP_CUSTOM_TO_USE				"regen/app_regen.c"

//uncomment to use custom app by default, regardless of settings:
//#define APPCONF_APP_TO_USE				APP_CUSTOM

#define APP_CUSTOM_CONF_CTRL_TYPE                CUSTOM_CTRL_TYPE_CURRENT_PEDAL_TORQUE

#define APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE        SPEED_SENSOR_TYPE_QUADRATURE
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1       HW_HALL_ENC_GPIO1
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PIN1        HW_HALL_ENC_PIN1
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PORT2       HW_HALL_ENC_GPIO2
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PIN2        HW_HALL_ENC_PIN2
#define APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS     24u     // including "virtual magnets"
#define APP_CUSTOM_CONF_PEDAL_SENSOR_USE_FILTER  1       // 1/0 = enable/disable
#define APP_CUSTOM_CONF_PEDAL_RPM_START          10.0f   // CRMP
#define APP_CUSTOM_CONF_PEDAL_RPM_END            150.0f  // CRPM
#define APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS      0.3f    // CRPM/sec
#define APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG      0.15f   // CRPM/sec
#define APP_CUSTOM_CONF_PEDAL_INVERT_DIR         1       // 1/0 = invert/no invert

#define APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE        SPEED_SENSOR_TYPE_SINGLE
#define APP_CUSTOM_CONF_WHEEL_SENSOR_PORT1       HW_HALL_ENC_GPIO3
#define APP_CUSTOM_CONF_WHEEL_SENSOR_PIN1        HW_HALL_ENC_PIN3
#define APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS     12u     // including "virtual magnets"
#define APP_CUSTOM_CONF_WHEEL_SENSOR_USE_FILTER  1       // 1/0 = enable/disable
#define APP_CUSTOM_CONF_WHEEL_RPM_START          10.0f   // WRPM
#define APP_CUSTOM_CONF_WHEEL_RPM_END            600.0f  // WRPM
#define APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS      0.3f    // WRPM/sec
#define APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG      0.15f   // WRPM/sec
#define APP_CUSTOM_CONF_WHEEL_INVERT_DIR         1       // 1/0 = invert/no invert

#define APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE       TORQUE_SENSOR_TYPE_ADC
#define APP_CUSTOM_CONF_TORQUE_SENSOR_PORT1      HW_ADC_EXT_GPIO
#define APP_CUSTOM_CONF_TORQUE_SENSOR_PIN1       HW_ADC_EXT_PIN

#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS   45.0f   // degree mechanical
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS     90.0f  // degree mechanical

#define APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1        HW_UART_TX_PORT
#define APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1         HW_UART_TX_PIN
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN  1.0f    // seconds
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CLOSE 0.3f    // seconds
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK 0.2f    // seconds
#define APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF     (-5)    // WRPM
#define APP_CUSTOM_CONF_CLUTCH_CHECK_RPM_DIFF    2u      // WRPM

#define APP_CUSTOM_CONF_UPDATE_RATE_HZ           500

#endif /* APP_REGEN_CONF_H_ */