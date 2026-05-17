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

#define APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE        SPEED_SENSOR_TYPE_QUADRATURE_POLL
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PORT1       HW_HALL_ENC_GPIO1
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PIN1        HW_HALL_ENC_PIN1
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PORT2       HW_HALL_ENC_GPIO2
#define APP_CUSTOM_CONF_PEDAL_SENSOR_PIN2        HW_HALL_ENC_PIN2
#define APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS      24u    // including "virtual magnets"
#define APP_CUSTOM_CONF_PEDAL_SENSOR_FILTER        0.8   // Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
#define APP_CUSTOM_CONF_PEDAL_AVG_ABOVE_RPM       10.0f  // CRPM - average last two samples above this value
#define APP_CUSTOM_CONF_PEDAL_RPM_MIN             10.0f  // CRPM - set 0 CRPM below this value
#define APP_CUSTOM_CONF_PEDAL_RPM_START           10.0f  // CRMP - start of boost
#define APP_CUSTOM_CONF_PEDAL_RPM_END            150.0f  // CRPM - end of boost
#define APP_CUSTOM_CONF_PEDAL_RPM_MAX            200.0f  // CRPM - raise error above this value
#define APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS        0.1f  // sec/fullscale (min to max)
#define APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG        0.1f  // sec/fullscale (min to max)
#define APP_CUSTOM_CONF_PEDAL_INVERT_DIR           0     // 1/0 = invert/no invert

#define APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE        SPEED_SENSOR_TYPE_NONE
#define APP_CUSTOM_CONF_WHEEL_SENSOR_PORT1       HW_HALL_ENC_GPIO3
#define APP_CUSTOM_CONF_WHEEL_SENSOR_PIN1        HW_HALL_ENC_PIN3
#define APP_CUSTOM_CONF_WHEEL_POLL_TO_INT_RPM     100.0f  // WRPM at which to switch from poll to interrupt mode
#define APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS       12u    // including "virtual magnets"
#define APP_CUSTOM_CONF_WHEEL_SENSOR_FILTER         0.8   // Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
#define APP_CUSTOM_CONF_WHEEL_AVG_ABOVE_RPM        35.0f  // WRPM - average last two samples above this value
#define APP_CUSTOM_CONF_WHEEL_PROGRESSIVE_AVG_RPM   0.0f  // WRPM - if > 0, use progressive averaging, adding one more sample to the average for every multiple of this RPM
#define APP_CUSTOM_CONF_WHEEL_RPM_MIN               5.0f  // WRPM - set 0 WRPM below this value
#define APP_CUSTOM_CONF_WHEEL_RPM_MAX            1000.0f  // WRPM - raise error above this value
#define APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS         0.3f  // sec/fullscale (min to max)
#define APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG         0.3f  // sec/fullscale (min to max)
#define APP_CUSTOM_CONF_WHEEL_INVERT_DIR            0     // 1/0 = invert/no invert
#define APP_CUSTOM_CONF_WHEEL_SKIPPED_MAGNET_THR    1.8f  // sec/sec - skipped period / normal period 
#define APP_CUSTOM_CONF_WHEEL_CALIBRATION_RPM      120.0f // WRPM - wheel calibration RPM
#define WHEEL_SENSOR_CALIBRATION_VALUES_COUNT      36u    // Number of values to store for wheel sensor calibration, should be equal to the number of magnets

#define APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE       TORQUE_SENSOR_TYPE_ADC
#define APP_CUSTOM_CONF_TORQUE_SENSOR_PORT1      HW_ADC_EXT_GPIO
#define APP_CUSTOM_CONF_TORQUE_SENSOR_PIN1       HW_ADC_EXT_PIN
#define APP_CUSTOM_CONF_TORQUE_SENSOR_PORT2      HW_ADC_EXT2_GPIO
#define APP_CUSTOM_CONF_TORQUE_SENSOR_PIN2       HW_ADC_EXT2_PIN
#define APP_CUSTOM_CONF_TORQUE_CUTOFF_RPM          215.0f // WRPM - set torque to 0 above this value
#define APP_CUSTOM_CONF_TORQUE_DECREASE_INTERVAL    20.0f // WRPM - start decreasing torque before cutoff RPM by this interval
#define APP_CUSTOM_CONF_TORQUE_SENSOR_FILTER         0.8f // Range 0.0 to 1.0, where 1.0 gives the unfiltered value.

#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS           45.0f  // degree mechanical
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS             80.0f  // degree mechanical
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RELEASE_RPM         15.0f  // WRPM
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_WAIT_BEFORE_RELEASE  0.02f // seconds
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_SYNC_START_POS      30.0f  // degree mechanical
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_CURRENT_RAMP_TIME    0.2f  // sec/fullscale (min to max)
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RESET_POS_PERCENT    0.8f  // percent of (end_pos-start_pos)

#define APP_CUSTOM_CONF_CLUTCH_MODE                  CLUTCH_MODE_CLOSED
#define APP_CUSTOM_CONF_CLUTCH_CTRL_PORT1            HW_UART_RX_PORT
#define APP_CUSTOM_CONF_CLUTCH_CTRL_PIN1             HW_UART_RX_PIN
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN        0.5f  // seconds
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC        0.1f  // seconds
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK       0.25f // seconds
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC_LOSS   0.1f  // seconds
#define APP_CUSTOM_CONF_CLUTCH_SYNC_TIMEOUT            2.0f  // seconds
#define APP_CUSTOM_CONF_CLUTCH_CLOSED_FIRST_CHECK_TIME 0.5f  // seconds
#define APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF           1.0f  // WRPM
#define APP_CUSTOM_CONF_CLUTCH_FIRST_CHECK_RPM_DIFF    8.0f  // WRPM
#define APP_CUSTOM_CONF_CLUTCH_CLOSED_CHECK_RPM_DIFF  40.0f  // WRPM
#define APP_CUSTOM_CONF_CLUTCH_OPEN_CHECK_RPM_DIFF     5.0f  // WRPM
#define APP_CUSTOM_CONF_CLUTCH_MIN_RPM_OPEN           55.0f  // WRPM
#define APP_CUSTOM_CONF_CLUTCH_MIN_RPM_CLOSE          45.0f  // WRPM
#define APP_CUSTOM_CONF_CLUTCH_MAX_RPM_OPEN          400.0f  // WRPM
#define APP_CUSTOM_CONF_CLUTCH_MAX_RPM_CLOSE         350.0f  // WRPM
#define APP_CUSTOM_CONF_CLUTCH_INVERT_DIR              0     // 1/0 = invert/no invert
#define APP_CUSTOM_CONF_CLUTCH_ERROR_LIMIT             6     // max errors in defined period before disabling clutch
#define CLUTCH_OPERATION_BUFFER_SIZE                  10     // must be larger than APP_CUSTOM_CONF_CLUTCH_ERROR_LIMIT
#define APP_CUSTOM_CONF_CLUTCH_ERROR_PERIOD           10.0f  // seconds - period for counting clutch errors
#define APP_CUSTOM_CONF_CLUTCH_DESYNC_TIME             0.3f  // seconds - time for the motor to slow down after opening
#define APP_CUSTOM_CONF_CLUTCH_SYNC_TIME               0.02f // seconds - time for ensuring stable sync
#define APP_CUSTOM_CONF_CLUTCH_SYNC_WHILE_CLOSING      1     // 1/0 = enable/disable motor to wheel sync while clutch is closing
#define APP_CUSTOM_CONF_CLUTCH_CURRENT_LIMIT_CLOSING   0.01f // relative current limit when clutch is closing (0.0 to 1.0)

#define APP_CUSTOM_CONF_UPDATE_RATE_HZ               500     // Hz - sensor signal processing and clutch control

#define APP_CUSTOM_CONF_CTRL_TYPE_ADDR                        0
#define APP_CUSTOM_CONF_PEDAL_SENSOR_MAGNETS_ADDR             1
#define APP_CUSTOM_CONF_PEDAL_SENSOR_FILTER_ADDR              2
#define APP_CUSTOM_CONF_PEDAL_RPM_START_ADDR                  3
#define APP_CUSTOM_CONF_PEDAL_RPM_END_ADDR                    4
#define APP_CUSTOM_CONF_PEDAL_INVERT_DIR_ADDR                 5
#define APP_CUSTOM_CONF_WHEEL_SENSOR_MAGNETS_ADDR             6
#define APP_CUSTOM_CONF_WHEEL_SENSOR_FILTER_ADDR              7
#define APP_CUSTOM_CONF_WHEEL_INVERT_DIR_ADDR                 8
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS_ADDR       9
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS_ADDR        10
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN_ADDR         11
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC_ADDR         12
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK_ADDR        13
#define APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF_ADDR            14
#define APP_CUSTOM_CONF_CLUTCH_CLOSED_CHECK_RPM_DIFF_ADDR    15
#define APP_CUSTOM_CONF_UPDATE_RATE_HZ_ADDR                  16
#define APP_CUSTOM_PLOTS_ENABLED_ADDR                        17
#define APP_CUSTOM_LOG_GROUPS_ENABLED_ADDR                   18
#define APP_CUSTOM_CONF_PEDAL_SENSOR_TYPE_ADDR               19
#define APP_CUSTOM_CONF_WHEEL_SENSOR_TYPE_ADDR               20
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_WAIT_BEFORE_RELEASE_ADDR   21
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RELEASE_RPM_ADDR           22
#define APP_CUSTOM_CONF_PEDAL_RAMP_TIME_POS_ADDR             23
#define APP_CUSTOM_CONF_PEDAL_RAMP_TIME_NEG_ADDR             24
#define APP_CUSTOM_CONF_WHEEL_RAMP_TIME_POS_ADDR             25
#define APP_CUSTOM_CONF_WHEEL_RAMP_TIME_NEG_ADDR             26
#define APP_CUSTOM_CONF_CLUTCH_MIN_RPM_CLOSE_ADDR            27
#define APP_CUSTOM_CONF_CLUTCH_MAX_RPM_OPEN_ADDR             28
#define APP_CUSTOM_CONF_CLUTCH_MAX_RPM_CLOSE_ADDR            29
#define APP_CUSTOM_CONF_CLUTCH_MODE_ADDR                     30
#define APP_CUSTOM_CONF_PEDAL_RPM_MIN_ADDR                   31
#define APP_CUSTOM_CONF_PEDAL_RPM_MAX_ADDR                   32
#define APP_CUSTOM_CONF_WHEEL_RPM_MIN_ADDR                   33
#define APP_CUSTOM_CONF_WHEEL_RPM_MAX_ADDR                   34
#define APP_CUSTOM_CONF_PEDAL_AVG_ABOVE_RPM_ADDR             35
#define APP_CUSTOM_CONF_WHEEL_AVG_ABOVE_RPM_ADDR             36
#define APP_CUSTOM_CONF_WHEEL_POLL_TO_INT_RPM_ADDR           37
#define APP_CUSTOM_CONF_CLUTCH_INVERT_DIR_ADDR               38
//#define APP_CUSTOM_CONF_CLUTCH_ENABLE_CHECK_ADDR             39
#define APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC_LOSS_ADDR    40
#define APP_CUSTOM_CONF_WHEEL_SKIPPED_MAGNET_THR_ADDR        41
#define APP_CUSTOM_CONF_CLUTCH_FIRST_CHECK_RPM_DIFF_ADDR     42
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_SYNC_START_POS_ADDR 43
#define APP_CUSTOM_CONF_CLUTCH_SYNC_TIMEOUT_ADDR             44
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_CURRENT_RAMP_TIME_ADDR 45
#define APP_CUSTOM_CONF_CLUTCH_ERROR_LIMIT_ADDR              46
#define APP_CUSTOM_CONF_CLUTCH_ERROR_PERIOD_ADDR             47
#define APP_CUSTOM_CONF_CLUTCH_OPEN_CHECK_RPM_DIFF_ADDR      48
#define APP_CUSTOM_CONF_TORQUE_SENSOR_TYPE_ADDR              49
#define APP_CUSTOM_CONF_CLUTCH_MIN_RPM_OPEN_ADDR             50
#define APP_CUSTOM_CONF_CLUTCH_DESYNC_TIME_ADDR              51
#define APP_CUSTOM_CONF_CLUTCH_SYNC_TIME_ADDR                52
#define APP_CUSTOM_CONF_CLUTCH_SYNC_WHILE_CLOSING_ADDR       53
#define APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_RESET_POS_PERCENT_ADDR  54
#define APP_CUSTOM_CONF_CLUTCH_CURRENT_LIMIT_CLOSING_ADDR    55
#define APP_CUSTOM_CONF_CLUTCH_CLOSED_FIRST_CHECK_TIME_ADDR  56
#define APP_CUSTOM_CONF_WHEEL_CALIBRATION_RPM_ADDR           57
#define APP_CUSTOM_CONF_WHEEL_PROGRESSIVE_AVG_RPM_ADDR       58
#define APP_CUSTOM_CONF_TORQUE_DECREASE_INTERVAL_ADDR        59
#define APP_CUSTOM_CONF_TORQUE_CUTOFF_RPM_ADDR               60
#define APP_CUSTOM_CONF_TORQUE_SENSOR_FILTER_ADDR            61

#endif /* APP_REGEN_CONF_H_ */