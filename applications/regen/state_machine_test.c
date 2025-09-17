/******************************************************************
 * This file is not part of the VESC firmware.
 * It is a standalone test program for the regen app state machine.
 ******************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

#include "app_regen_types.h"
#include "app_regen_conf.h"

#define TRUE 1
#define FALSE 0
#define MAX(a,b) (((a)>(b))?(a):(b))

typedef enum{
    WRPM_0,
    WRPM_BELOW_MIN_CLOSE,
    WRPM_MIN_CLOSE_TO_MIN_OPEN,
    WRPM_MIN_OPEN_TO_MAX_CLOSE,
    WRPM_MAX_CLOSE_TO_MAX_OPEN,
    WRPM_ABOVE_MAX_OPEN,
    NUM_WRPM_CASES
} wrmp_cases;

typedef enum{
    CLUTCH_OPEN,
    CLUTCH_OPEN_ERROR,
    CLUTCH_OPENING,
    CLUTCH_OPENING_TMP,
    CLUTCH_WAITING,
    CLUTCH_SYNCING,
    CLUTCH_SYNCED,
    CLUTCH_CLOSING,
    CLUTCH_CLOSING_TMP,
    CLUTCH_CLOSED_FLOAT,
    CLUTCH_CLOSED_BRAKE,
    CLUTCH_CLOSED_ASSIST,
    CLUTCH_CLOSED_ERROR,
    NUM_CLUTCH_CASES
} clutch_cases;

typedef enum{
    MWRPM_0,
    MWRPM_BELOW_MIN,
    MWRPM_BELOW_CLOSED_DIFF,
    MWRPM_CLOSED_TO_FIRST_DIFF,
    MWRPM_FIRST_TO_OPEN_DIFF,
    MWRPM_EQUALS_TARGET,
    MWRPM_OPEN_TO_FIRST_DIFF,
    MWRPM_FIRST_TO_CLOSED_DIFF,
    MWRPM_ABOVE_CLOSED_DIFF,
    NUM_MWRPM_CASES
} mwrpm_cases;

typedef enum{
    BRAKE_POS_0,
    BRAKE_POS_BELOW_SYNC_START,
    BRAKE_POS_SYNC_START_TO_START,
    BRAKE_POS_ABOVE_START,
    NUM_BRAKE_POS_CASES
} brake_pos_cases;

typedef enum{
    //PEDAL_SPEED_0,
    PEDAL_SPEED_BELOW_MIN,
    PEDAL_SPEED_ABOVE_MIN,
    NUM_PEDAL_SPEED_CASES
} pedal_speed_cases;

typedef enum{
    //PEDAL_TORQUE_0,
    PEDAL_TORQUE_BELOW_MIN,
    PEDAL_TORQUE_ABOVE_MIN,
    NUM_PEDAL_TORQUE_CASES
} pedal_torque_cases;

static custom_config_type config;

//// State variables
static float time = 0;
static float clutch_timestamp = 0;
static float pedal_torque = 0;
static float pedal_speed  = 0;    //CRPM
static float pedal_brake_position = 0;
static float wheel_speed  = 0;    //WRPM
static float motor_speed  = 0;    //MWRPM
static float target_speed = 0;
static clutch_state_type clutch_state;

static char* clutch_state_str;
static clutch_state_type last_clutch_state;
static int transition_counter;
static float last_transition_time;
static clutch_state_type clutch_state_record[50];
static char linebuf[500];

static void init()
{
    memset(&config, 0, sizeof(config));
    // Wheel sensor config
    config.wheel_sensor.rpm_min = APP_CUSTOM_CONF_WHEEL_RPM_MIN;
    config.wheel_sensor.rpm_max = APP_CUSTOM_CONF_WHEEL_RPM_MAX;

    // Clutch config
    config.clutch.min_rpm_open = APP_CUSTOM_CONF_CLUTCH_MIN_RPM_OPEN;
    config.clutch.min_rpm_close = APP_CUSTOM_CONF_CLUTCH_MIN_RPM_CLOSE;
    config.clutch.max_rpm_open = APP_CUSTOM_CONF_CLUTCH_MAX_RPM_OPEN;
    config.clutch.max_rpm_close = APP_CUSTOM_CONF_CLUTCH_MAX_RPM_CLOSE;
    config.clutch.sync_rpm_diff = APP_CUSTOM_CONF_CLUTCH_SYNC_RPM_DIFF;
    config.clutch.first_check_rpm_diff = APP_CUSTOM_CONF_CLUTCH_FIRST_CHECK_RPM_DIFF;
    config.clutch.open_check_rpm_diff = APP_CUSTOM_CONF_CLUTCH_OPEN_CHECK_RPM_DIFF;
    config.clutch.closed_check_rpm_diff = APP_CUSTOM_CONF_CLUTCH_CLOSED_CHECK_RPM_DIFF;
    config.clutch.wait_before_sync_loss = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC_LOSS;
    config.clutch.wait_before_sync = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_SYNC;
    config.clutch.sync_timeout = APP_CUSTOM_CONF_CLUTCH_SYNC_TIMEOUT;
    config.clutch.wait_before_check = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_CHECK;
    config.clutch.wait_before_open = APP_CUSTOM_CONF_CLUTCH_WAIT_BEFORE_OPEN;
    //config.clutch.mode = APP_CUSTOM_CONF_CLUTCH_MODE;
    config.clutch.mode = CLUTCH_MODE_AUTO;

    // Back pedal brake config
    config.back_pedal_brake.start_pos = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_START_POS;
    config.back_pedal_brake.sync_start_pos = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_SYNC_START_POS;
    config.back_pedal_brake.end_pos = APP_CUSTOM_CONF_BACK_PEDAL_BRAKE_END_POS;

    // Pedal speed config
    config.pedal_sensor.rpm_min = APP_CUSTOM_CONF_PEDAL_RPM_MIN;
    config.pedal_sensor.rpm_max = APP_CUSTOM_CONF_PEDAL_RPM_MAX;
}

static void new_clutch_state(clutch_state_type cs)
{
    clutch_timestamp = time;
    clutch_state = cs;
}

static void update_clutch_state(void)
{
	float timestamp = time;
	float elapsed_time = time - clutch_timestamp;

	if (config.clutch.mode == CLUTCH_MODE_FULL_MANUAL) {
		if (wheel_speed < config.wheel_sensor.rpm_min && motor_speed < config.wheel_sensor.rpm_min) {
			clutch_state = CLUTCH_STATE_CLOSING;
		}

		if (clutch_state == CLUTCH_STATE_OPENING) {
			clutch_state = CLUTCH_STATE_OPEN;
		} else if (clutch_state == CLUTCH_STATE_SYNCING) {
			clutch_state = CLUTCH_STATE_SYNCED;
		} else if (clutch_state == CLUTCH_STATE_CLOSING) {
			clutch_state = CLUTCH_STATE_CLOSED_FLOAT;
		}
		return;
	}

	bool stopped           = (wheel_speed < config.wheel_sensor.rpm_min && motor_speed < config.wheel_sensor.rpm_min);
	bool too_slow          = (wheel_speed < config.clutch.min_rpm_close);
	bool not_too_slow      = (wheel_speed > config.clutch.min_rpm_open);
	bool not_too_fast      = (wheel_speed < config.clutch.max_rpm_close);
	bool too_fast          = (wheel_speed > config.clutch.max_rpm_open);
	bool diff_to_target_small_enough = (abs(MAX((wheel_speed - config.clutch.sync_rpm_diff), 0) - motor_speed) < config.clutch.first_check_rpm_diff);
	bool diff_small_enough = (abs(wheel_speed - motor_speed) < config.clutch.first_check_rpm_diff);
	bool diff_large_enough = (abs(wheel_speed - motor_speed) > config.clutch.open_check_rpm_diff);
	bool diff_too_small    = (abs(wheel_speed - motor_speed) < config.clutch.open_check_rpm_diff) && (wheel_speed > config.clutch.open_check_rpm_diff);
	bool diff_too_large    = (abs(wheel_speed - motor_speed) > config.clutch.closed_check_rpm_diff);
	bool pedaling          = (pedal_speed > 0 && pedal_torque > 0);
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
			else if (diff_too_small) { // got stuck closed
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
			else if (pedaling && not_too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (braking && not_too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (manual_mode) {
				new_clutch_state(CLUTCH_STATE_CLOSING);
			}
			else if (elapsed_time > config.clutch.sync_timeout && auto_mode) {
				//update_pedal_speed_and_position(TRUE); // reset brake position to avoid immediate re-sync
                pedal_brake_position = 0;
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
			else if (diff_too_large) { // got out of closed
				new_clutch_state(CLUTCH_STATE_CLOSED_ERROR);
			}
			else if (pedaling) {
				new_clutch_state(CLUTCH_STATE_CLOSED_ASSIST);
			}
			else if (braking) {
				new_clutch_state(CLUTCH_STATE_CLOSED_BRAKE);
			}
			else if (elapsed_time > config.clutch.wait_before_open && not_too_slow && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPENING);
			}
			break;
		case CLUTCH_STATE_CLOSED_BRAKE:
			if (too_fast && auto_mode) {
				new_clutch_state(CLUTCH_STATE_OPENING);
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

int main()
{
    int c = 1;
    int last_c = 1;

    init();

    printf(" ##  | Wheel RPM     | Motor RPM     | Clutch State  | Brake Pos     | Pedal Speed   | Pedal Torque   || Result                                | Transitions\n");
    printf("-----------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------\n");

    for (int i=0; i<NUM_WRPM_CASES; i++) {
        for (int k=0; k<NUM_MWRPM_CASES; k++) {
            for (int j=0; j<NUM_CLUTCH_CASES; j++) {
                for (int l=0; l<NUM_BRAKE_POS_CASES; l++) {
                    for (int m=0; m<NUM_PEDAL_SPEED_CASES; m++) {
                        for (int n=0; n<NUM_PEDAL_TORQUE_CASES; n++) {

                            if (l != BRAKE_POS_0 && m != PEDAL_SPEED_BELOW_MIN) {
                                // Skip impossible case of braking and pedaling
                                continue;
                            }

                            //if (l != BRAKE_POS_0 && n != PEDAL_TORQUE_BELOW_MIN) {
                            //    // Skip unnecessary case of braking and throttle
                            //    continue;
                            //}

                            snprintf(linebuf, sizeof(linebuf), "%4d |", c);

                            // Set test inputs based on case enums
                            switch (i) {
                                case WRPM_0:
                                    wheel_speed = 0;
                                    //printf(" 0 (STOPPED)   ");
                                    //printf("  ____________ ");
                                    break;
                                case WRPM_BELOW_MIN_CLOSE:
                                    wheel_speed = config.clutch.min_rpm_close / 2;
                                    //printf(" < MIN_CLOSE    ");
                                    //printf("  <C___________ ");
                                    break;
                                case WRPM_MIN_CLOSE_TO_MIN_OPEN:
                                    wheel_speed = (config.clutch.min_rpm_close + config.clutch.min_rpm_open) / 2;
                                    //printf(" MIN_CL..MIN_OP ");
                                    //printf("  _C<O_________ ");
                                    break;
                                case WRPM_MIN_OPEN_TO_MAX_CLOSE:
                                    wheel_speed = (config.clutch.min_rpm_open + config.clutch.max_rpm_close) / 2;
                                    //printf(" MIN_OP..MAX_CL ");
                                    //printf("  ___O  <  C___ ");
                                    break;
                                case WRPM_MAX_CLOSE_TO_MAX_OPEN:
                                    wheel_speed = (config.clutch.max_rpm_close + config.clutch.max_rpm_open) / 2;
                                    //printf(" MAX_CL..MAX_OP ");
                                    //printf("  _________C<O_ ");
                                    break;
                                case WRPM_ABOVE_MAX_OPEN:
                                    wheel_speed = (config.clutch.max_rpm_open + config.wheel_sensor.rpm_max) / 2;
                                    //printf(" > MAX_OP       ");
                                    //printf("  ___________O< ");
                                    break;
                                default:
                                    break;
                            }
                            snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), "      %4.0f      ", wheel_speed);

                            target_speed = MAX(wheel_speed - config.clutch.sync_rpm_diff, 0);
                            switch (k) {
                                case MWRPM_0:
                                    motor_speed = 0;
                                    //printf(" 0 (STOPPED)   ");
                                    //printf("______________ ");
                                    break;
                                case MWRPM_BELOW_MIN:
                                    motor_speed = 1;
                                    //printf(" < MIN          ");
                                    //printf("<_____________ ");
                                    break;
                                case MWRPM_BELOW_CLOSED_DIFF:
                                    motor_speed = MAX((target_speed - config.clutch.closed_check_rpm_diff) / 2, 0);
                                    if (motor_speed <= 1) { continue; } // skip duplicate case
                                    //printf(" < CLOSED_DIFF  ");
                                    //printf("<C_____________ ");
                                    break;
                                case MWRPM_CLOSED_TO_FIRST_DIFF:
                                    motor_speed = MAX(target_speed - (config.clutch.closed_check_rpm_diff + config.clutch.first_check_rpm_diff) / 2, 0);
                                    if (motor_speed <= 1) { continue; } // skip duplicate case
                                    //printf(" CLOSED..FIRST  ");
                                    //printf("_C<F___________ ");
                                    break;
                                case MWRPM_FIRST_TO_OPEN_DIFF:
                                    motor_speed = MAX(target_speed - (config.clutch.first_check_rpm_diff + config.clutch.open_check_rpm_diff) / 2, 0);
                                    if (motor_speed <= 1) { continue; } // skip duplicate case
                                    //printf(" FIRST..OPEN    ");
                                    //printf("___F<O_________ ");
                                    break;
                                case MWRPM_EQUALS_TARGET:
                                    motor_speed = target_speed;
                                    if (motor_speed <= 1) { continue; } // skip duplicate case
                                    //printf(" = TARGET       ");
                                    //printf("______=T_______ ");
                                    break;
                                case MWRPM_OPEN_TO_FIRST_DIFF:
                                    motor_speed = MAX(target_speed + (config.clutch.first_check_rpm_diff + config.clutch.open_check_rpm_diff) / 2, 0);
                                    if (motor_speed <= 1) { continue; } // skip duplicate case
                                    //printf(" OPEN..FIRST    ");
                                    //printf("_________O<F___ ");
                                    break;
                                case MWRPM_FIRST_TO_CLOSED_DIFF:
                                    motor_speed = MAX(target_speed + (config.clutch.closed_check_rpm_diff + config.clutch.first_check_rpm_diff) / 2, 0);
                                    if (motor_speed <= 1) { continue; } // skip duplicate case
                                    //printf(" FIRST..CLOSED  ");
                                    //printf("___________F<C_ ");
                                    break;
                                case MWRPM_ABOVE_CLOSED_DIFF:
                                    motor_speed = MAX(target_speed + config.clutch.closed_check_rpm_diff + 10, 0);
                                    if (motor_speed <= 1) { continue; } // skip duplicate case
                                    //printf(" > CLOSED_DIFF  ");
                                    //printf("_____________C< ");
                                    break;
                                default:
                                    break;
                            }
                            snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), "      %4.0f      ", motor_speed);

                            switch (j) {
                                case CLUTCH_OPEN:
                                    clutch_state = CLUTCH_STATE_OPEN;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " OPEN (OPEN)    ");
                                    break;
                                case CLUTCH_OPEN_ERROR:
                                    clutch_state = CLUTCH_STATE_OPEN_ERROR;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " OPEN (ERROR)   ");
                                    break;
                                case CLUTCH_OPENING:
                                    clutch_state = CLUTCH_STATE_OPENING;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " OPEN (OPENING) ");
                                    break;
                                case CLUTCH_OPENING_TMP:
                                    clutch_state = CLUTCH_STATE_OPENING_TMP;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " OPEN (OP_TMP)  ");
                                    break;
                                case CLUTCH_WAITING:
                                    clutch_state = CLUTCH_STATE_WAITING;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " OPEN (WAITING) ");
                                    break;
                                case CLUTCH_SYNCING:
                                    clutch_state = CLUTCH_STATE_SYNCING;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " OPEN (SYNCING) ");
                                    break;
                                case CLUTCH_SYNCED:
                                    clutch_state = CLUTCH_STATE_SYNCED;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " OPEN (SYNCED)  ");
                                    break;
                                case CLUTCH_CLOSING:
                                    clutch_state = CLUTCH_STATE_CLOSING;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " CLOSE (CLOSING)");
                                    break;
                                case CLUTCH_CLOSING_TMP:
                                    clutch_state = CLUTCH_STATE_CLOSING_TMP;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " CLOSE (CL_TMP) ");
                                    break;
                                case CLUTCH_CLOSED_FLOAT:
                                    clutch_state = CLUTCH_STATE_CLOSED_FLOAT;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " CLOSE (FLOAT)  ");
                                    break;
                                case CLUTCH_CLOSED_BRAKE:
                                    clutch_state = CLUTCH_STATE_CLOSED_BRAKE;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " CLOSE (BRAKE)  ");
                                    break;
                                case CLUTCH_CLOSED_ASSIST:
                                    clutch_state = CLUTCH_STATE_CLOSED_ASSIST;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " CLOSE (ASSIST) ");
                                    break;
                                case CLUTCH_CLOSED_ERROR:
                                    clutch_state = CLUTCH_STATE_CLOSED_ERROR;
                                    snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), " CLOSE (ERROR)  ");
                                    break;
                                default:
                                    break;
                            }

                            switch (l) {
                                case BRAKE_POS_0:
                                    pedal_brake_position = 0;
                                    //printf(" 0 (NO BRAKE)  ");
                                    //printf(" _____________ ");
                                    break;
                                case BRAKE_POS_BELOW_SYNC_START:
                                    pedal_brake_position = config.back_pedal_brake.sync_start_pos / 2;
                                    //printf(" < SYNC_START   ");
                                    //printf(" <SYNC_________ ");
                                    break;
                                case BRAKE_POS_SYNC_START_TO_START:
                                    pedal_brake_position = (config.back_pedal_brake.sync_start_pos + config.back_pedal_brake.start_pos) / 2;
                                    //printf(" SYNC_ST..START ");
                                    //printf(" _SYNC<START___ ");
                                    break;
                                case BRAKE_POS_ABOVE_START:
                                    pedal_brake_position = (config.back_pedal_brake.start_pos + config.back_pedal_brake.end_pos) / 2;
                                    //printf(" > START        ");
                                    //printf(" ______START<__ ");
                                    break;
                                default:
                                    break;
                            }
                            snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), "      %4.0f      ", pedal_brake_position);

                            switch (m) {
                                case PEDAL_SPEED_BELOW_MIN:
                                    pedal_speed = 0;
                                    //printf(" < MIN_________ ");
                                    break;
                                case PEDAL_SPEED_ABOVE_MIN:
                                    pedal_speed = (config.pedal_sensor.rpm_min + config.pedal_sensor.rpm_max) / 2;
                                    //printf(" __MIN < MAX___ ");
                                    break;
                                default:
                                    break;
                            }
                            snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), "      %4.0f      ", pedal_speed);

                            switch (n) {
                                case PEDAL_TORQUE_BELOW_MIN:
                                    pedal_torque = 0;
                                    //printf(" < MIN_________ ");
                                    break;
                                case PEDAL_TORQUE_ABOVE_MIN:
                                    pedal_torque = 0.5;
                                    //printf(" __MIN < MAX___ ");
                                    break;
                                default:
                                    break;
                            }
                            snprintf(linebuf + strlen(linebuf), sizeof(linebuf) - strlen(linebuf), "      %4.0f      ", pedal_torque*100);

                            // Print inputs
                            printf("%s", linebuf);

                            // Reset time and clutch timestamp
                            time = 0;
                            clutch_timestamp = 0;
                            last_clutch_state = clutch_state;
                            transition_counter = 0;
                            last_transition_time = 0;

                            // Simulate time progression and update clutch state
                            for (time=0; time<5; time+=0.1) {
                                clutch_state_record[(int)(time*10)] = clutch_state;
                                update_clutch_state();
                                if (clutch_state != last_clutch_state) {
                                    transition_counter++;
                                    last_transition_time = time;
                                }
                                last_clutch_state = clutch_state;
                            }

                            if (last_transition_time > 3) {
                                printf("|| unstable    (%2d transitions in 5 sec) | ", transition_counter);
                            } else {
                                // Print result
                                switch (clutch_state) {
                                    case CLUTCH_STATE_OPEN:
                                        clutch_state_str = "OPEN          ";
                                        break;
                                    case CLUTCH_STATE_OPEN_ERROR:
                                        clutch_state_str = "OPEN_ERROR    ";
                                        break;
                                    case CLUTCH_STATE_OPENING:
                                        clutch_state_str = "OPENING       ";
                                        break;
                                    case CLUTCH_STATE_OPENING_TMP:
                                        clutch_state_str = "OPENING_TMP   ";
                                        break;
                                    case CLUTCH_STATE_WAITING:
                                        clutch_state_str = "WAITING       ";
                                        break;
                                    case CLUTCH_STATE_SYNCING:
                                        clutch_state_str = "SYNCING       ";
                                        break;
                                    case CLUTCH_STATE_SYNCED:
                                        clutch_state_str = "SYNCED        ";
                                        break;
                                    case CLUTCH_STATE_CLOSING:
                                        clutch_state_str = "CLOSING       ";
                                        break;
                                    case CLUTCH_STATE_CLOSING_TMP:
                                        clutch_state_str = "CLOSING_TMP   ";
                                        break;
                                    case CLUTCH_STATE_CLOSED_FLOAT:
                                        clutch_state_str = "CLOSED_FLOAT  ";
                                        break;
                                    case CLUTCH_STATE_CLOSED_BRAKE:
                                        clutch_state_str = "CLOSED_BRAKE  ";
                                        break;
                                    case CLUTCH_STATE_CLOSED_ASSIST:
                                        clutch_state_str = "CLOSED_ASSIST ";
                                        break;
                                    case CLUTCH_STATE_CLOSED_ERROR:
                                        clutch_state_str = "CLOSED_ERROR  ";
                                        break;
                                    default:
                                        clutch_state_str = "UNKNOWN       ";
                                        break;
                                }
                                printf("|| %s (after %2d transitions) | ", clutch_state_str, transition_counter);
                            }
                            for (int h=0; h<50; h++) {
                                printf("%2d", clutch_state_record[h]);
                                if (h<49) {
                                    printf("-");
                                } else {
                                    printf("\n");
                                }
                            }
                            c++;
                        }
                    }
                }
                if (c > last_c) {
                    printf("     |                                                                                                ||                       \n");
                    last_c = c;
                }
            }
        }
    }

    return 0;
}