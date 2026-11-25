#ifndef TEST_VEHICLE_CONFIG_H
#define TEST_VEHICLE_CONFIG_H

#include "rc_input.h"

#define VEHICLE_CH_MODE 7U
#define VEHICLE_CH_STEPPER 2U
#define VEHICLE_CH_LOWER_ARM 3U
#define VEHICLE_CH_WRIST 0U
#define VEHICLE_CH_GRIPPER_AXIS 1U
#define VEHICLE_CH_GRIPPER 8U
#define VEHICLE_CH_ACTION_BIT0 6U
#define VEHICLE_CH_ACTION_BIT1 9U
#define VEHICLE_ARM_MODE_LEVEL RC_SW_DOWN
#define VEHICLE_GRIPPER_OPEN_LEVEL RC_SW_UP
#define VEHICLE_GRIPPER_CLOSE_LEVEL RC_SW_DOWN
#define VEHICLE_ACTION_STABLE_MS 200U
#define VEHICLE_REFERENCE_HOLD_MS 500U
#define VEHICLE_RC_DEADBAND 0.04f
#define VEHICLE_HOLD_ARM_IN_DRIVE 1

typedef struct
{
    float lower_arm_us;
    float wrist_rad;
} VehiclePose;

#define VEHICLE_POSE_COUNT 4U
#define VEHICLE_POSE_TABLE                     \
{                                              \
    { 0.0f, 0.0f },                            \
    { 0.0f, 0.0f },                            \
    { 0.0f, 0.0f },                            \
    { 0.0f, 0.0f }                             \
}

#define VEHICLE_POSE_WRIST_FULL_RAD 0.50f
#define VEHICLE_POSE_WRIST_DEADBAND 0.02f
#define VEHICLE_POSE_SERVO_FULL_US 200.0f
#define VEHICLE_POSE_SERVO_DEADBAND 10.0f

#endif
