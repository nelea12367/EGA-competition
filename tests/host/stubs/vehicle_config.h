#ifndef TEST_VEHICLE_CONFIG_H
#define TEST_VEHICLE_CONFIG_H

#include "rc_input.h"

/* Mirrors Core/Inc/vehicle_config.h. Keep the two in sync. */
#define VEHICLE_CH_MODE 7U
#define VEHICLE_CH_STEPPER 2U
#define VEHICLE_CH_LOWER_ARM 1U
#define VEHICLE_CH_WRIST 3U
#define VEHICLE_CH_GRIPPER 6U
#define VEHICLE_CH_POSE_C 8U
#define VEHICLE_CH_POSE_D 9U
#define VEHICLE_CH_CALIBRATE 0U

#define VEHICLE_ARM_MODE_LEVEL RC_SW_DOWN
#define VEHICLE_GRIPPER_OPEN_LEVEL RC_SW_UP
#define VEHICLE_GRIPPER_CLOSE_LEVEL RC_SW_DOWN
#define VEHICLE_RC_DEADBAND 0.04f
#define VEHICLE_HOLD_ARM_IN_DRIVE 1

#define VEHICLE_POSE_DEBOUNCE_MS 200U
#define VEHICLE_CAL_TRIGGER_LEVEL 0.90f
#define VEHICLE_CAL_RELEASE_LEVEL 0.10f
#define VEHICLE_CAL_HOLD_MS 1000U

/* Stepper: 1600 microsteps, 50:1 reduction -> 222.2 pulses per output degree */
#define VEHICLE_STEPPER_PULSES_PER_REV 1600.0f
#define VEHICLE_STEPPER_GEAR_RATIO 50.0f
#define VEHICLE_STEPPER_PULSES_PER_DEG 222.222f
#define VEHICLE_STEPPER_MAX_HZ 25000.0f
#define VEHICLE_STEPPER_MANUAL_DPS 90.0f

/* Lower-arm servo: 1500us == 0 deg, 7.407us per degree */
#define VEHICLE_LOWER_ARM_ZERO_US 1500.0f
#define VEHICLE_LOWER_ARM_ZERO_DEG 0.0f
#define VEHICLE_LOWER_ARM_US_PER_DEG 7.407f
#define VEHICLE_LOWER_ARM_SERVO_RATE_US_S 1000.0f
#define VEHICLE_LOWER_ARM_MANUAL_DPS 84.0f

/* Wrist */
#define VEHICLE_WRIST_MANUAL_DPS 172.0f
#define VEHICLE_WRIST_MAX_RATE_RAD_S 4.00f

/* Calibration reference angles */
#define VEHICLE_CAL_STEPPER_DEG 0.0f
#define VEHICLE_CAL_WRIST_DEG 0.0f

typedef struct
{
    float stepper_deg;
    float lower_arm_deg;
    float wrist_deg;
} VehiclePose;

/* Pose index = C position * 2 + D position. Values chosen so the assertions can
 * tell the poses apart (pose 0 = 0 deg, pose 3 = 90 deg). */
#define VEHICLE_POSE_COUNT 6U
#define VEHICLE_POSE_TABLE                     \
{                                              \
    {    0.0f, 0.0f, 0.0f },  /* 0 C up,   D up   */ \
    {   10.0f, 0.0f, 0.0f },  /* 1 C up,   D down */ \
    {   20.0f, 0.0f, 0.0f },  /* 2 C mid,  D up   */ \
    {   90.0f, 0.0f, 0.0f },  /* 3 C mid,  D down */ \
    {  -90.0f, 0.0f, 0.0f },  /* 4 C down, D up   */ \
    {  -50.0f, 0.0f, 0.0f }   /* 5 C down, D down */ \
}

#define VEHICLE_POSE_STEPPER_FULL_DEG 150.0f
#define VEHICLE_POSE_STEPPER_DEADBAND 0.5f
#define VEHICLE_POSE_WRIST_FULL_DEG 15.0f
#define VEHICLE_POSE_WRIST_DEADBAND 0.5f
#define VEHICLE_POSE_SERVO_FULL_DEG 20.0f
#define VEHICLE_POSE_SERVO_DEADBAND 1.0f

#endif
