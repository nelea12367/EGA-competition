#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include "rc_input.h"

/* Receiver layout. Adjust only this file after receiver channel calibration. */
#define VEHICLE_CH_ENABLE             6U
#define VEHICLE_CH_MODE               7U
#define VEHICLE_CH_STEPPER            2U
#define VEHICLE_CH_LOWER_ARM          3U
#define VEHICLE_CH_WRIST              0U
#define VEHICLE_CH_GRIPPER            8U
#define VEHICLE_ENABLE_LEVEL          RC_SW_UP
#define VEHICLE_ARM_MODE_LEVEL        RC_SW_DOWN
#define VEHICLE_GRIPPER_OPEN_LEVEL    RC_SW_UP
#define VEHICLE_GRIPPER_CLOSE_LEVEL   RC_SW_DOWN
#define VEHICLE_REFERENCE_HOLD_MS     500U
#define VEHICLE_RC_DEADBAND           0.08f

/*
 * Temporary direct DM430 interface:
 *   DM-MC02 5V -> PUL+/DIR+, PD15 -> PUL-, PD14 -> DIR-.
 * PD15/PD14 are configured as open-drain sinks, not 3.3V push-pull outputs.
 */
#define VEHICLE_STEPPER_DIRECT_DM430        1
#define VEHICLE_BOARD_5V_ENABLE_ACTIVE_HIGH 1

/* DM430 pulse/direction axis. Position is counted in pulses from manual zero. */
#define VEHICLE_STEPPER_MIN_PULSE       (-12000)
#define VEHICLE_STEPPER_MAX_PULSE       12000
#define VEHICLE_STEPPER_MAX_HZ          1000.0f /* Conservative direct-connection bench-test limit. */
#define VEHICLE_STEPPER_ACCEL_HZ_S      15000.0f
#define VEHICLE_STEPPER_DIR_POSITIVE    1
#define VEHICLE_STEPPER_USE_ENA         0
#define VEHICLE_STEPPER_ENA_ACTIVE_HIGH 0

/* ID-5 C610/M2006 lower arm. Angles are output-shaft radians from manual zero. */
#define VEHICLE_LOWER_ARM_ID                0x205U
#define VEHICLE_LOWER_ARM_GEAR_RATIO         36.0f
#define VEHICLE_LOWER_ARM_MIN_RAD            (-1.20f)
#define VEHICLE_LOWER_ARM_MAX_RAD            1.20f
#define VEHICLE_LOWER_ARM_MAX_RATE_RAD_S     1.00f
#define VEHICLE_LOWER_ARM_KP                 4500.0f
#define VEHICLE_LOWER_ARM_CURRENT_MAX        2500
#define VEHICLE_LOWER_ARM_TIMEOUT_MS         100U
#define VEHICLE_LOWER_ARM_REVERSED           0

/* Servo mechanical windows. Measure and tighten these before competition use. */
#define VEHICLE_WRIST_MIN_DEG          (-45.0f)
#define VEHICLE_WRIST_MAX_DEG          45.0f
#define VEHICLE_WRIST_NEUTRAL_DEG      0.0f
#define VEHICLE_WRIST_MIN_US           1100U
#define VEHICLE_WRIST_MAX_US           1900U
#define VEHICLE_WRIST_RATE_DEG_S       70.0f

#define VEHICLE_GRIPPER_MIN_DEG        0.0f
#define VEHICLE_GRIPPER_MAX_DEG        60.0f
#define VEHICLE_GRIPPER_NEUTRAL_DEG    25.0f
#define VEHICLE_GRIPPER_MIN_US         1050U
#define VEHICLE_GRIPPER_MAX_US         1950U
#define VEHICLE_GRIPPER_RATE_DEG_S     90.0f

#endif /* VEHICLE_CONFIG_H */
