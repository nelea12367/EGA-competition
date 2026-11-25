#include "vehicle_control.h"

#include "main.h"
#include "arm_2006.h"
#include "arm_presets.h"
#include "arm_stepper.h"
#include "chassis.h"
#include "rc_input.h"
#include "servo_pair.h"
#include "vehicle_config.h"
#include "vehicle_math.h"

static VehicleControlState s_state;
static uint32_t s_last_ms;
static uint32_t s_reference_since_ms;
static bool s_reference_gesture_latched;

static float VehicleAxis(const RcData *rc, uint8_t channel)
{
    float value = rc->analog[channel];
    return (Vehicle_AbsF(value) <= VEHICLE_RC_DEADBAND) ? 0.0f : value;
}

static bool VehicleInputsCentered(const RcData *rc)
{
    return (VehicleAxis(rc, VEHICLE_CH_STEPPER) == 0.0f) &&
           (VehicleAxis(rc, VEHICLE_CH_LOWER_ARM) == 0.0f) &&
           (VehicleAxis(rc, VEHICLE_CH_WRIST) == 0.0f);
}

static bool VehicleReferenceGesture(const RcData *rc)
{
    return VehicleInputsCentered(rc) &&
           (rc->switch_pos[VEHICLE_CH_PRESET_A] == RC_SW_UP) &&
           (rc->switch_pos[VEHICLE_CH_PRESET_D] == RC_SW_UP);
}

static void VehicleUpdateGripper(const RcData *rc, float dt_s)
{
    if (rc->switch_pos[VEHICLE_CH_GRIPPER] == VEHICLE_GRIPPER_OPEN_LEVEL)
        ServoPair_SetGripperRate(1.0f, dt_s);
    else if (rc->switch_pos[VEHICLE_CH_GRIPPER] == VEHICLE_GRIPPER_CLOSE_LEVEL)
        ServoPair_SetGripperRate(-1.0f, dt_s);
    else
        ServoPair_HoldSafe();
}

static void VehicleApplyPose(const ArmPreset *pose, uint32_t now_ms, float dt_s)
{
    ArmStepper_SetTargetPulse(pose->stepper_pulse, true, dt_s);
    Arm2006_SetTargetRad(pose->wrist_rad);
    Arm2006_Update(now_ms, dt_s, true);
    ServoPair_SetLowerArmTargetUs(pose->lower_arm_us, dt_s);
}

static void VehicleCaptureReference(const RcData *rc, uint32_t now_ms)
{
    if ((!s_state.wrist_online) || (!VehicleReferenceGesture(rc)))
    {
        s_reference_since_ms = 0U;
        s_reference_gesture_latched = false;
        return;
    }
    if (s_reference_gesture_latched) return;
    if (s_reference_since_ms == 0U) s_reference_since_ms = now_ms;
    if ((uint32_t)(now_ms - s_reference_since_ms) < VEHICLE_REFERENCE_HOLD_MS) return;

    ArmStepper_SetReference();
    Arm2006_SetReference();
    s_state.stepper_reference_valid = ArmStepper_GetState()->reference_valid;
    s_state.wrist_reference_valid = Arm2006_GetState()->reference_valid;
    s_reference_gesture_latched = true;
}

void VehicleControl_Init(void)
{
    s_state.mode = VEHICLE_MODE_DRIVE;
    s_state.enabled = false;
    s_state.stepper_reference_valid = false;
    s_state.wrist_reference_valid = false;
    s_state.wrist_online = false;
    s_last_ms = HAL_GetTick();
    s_reference_since_ms = 0U;
    s_reference_gesture_latched = false;
    ArmStepper_Init();
    Arm2006_Init();
    ServoPair_Init();
}

void VehicleControl_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len)
{
    Arm2006_OnCanRx(id, data, len);
}

void VehicleControl_ForceStop(void)
{
    Chassis_ForceStop();
    ArmStepper_Stop();
    Arm2006_Stop();
    ServoPair_HoldSafe();
    s_state.enabled = false;
    s_state.stepper_reference_valid = false;
    s_state.wrist_reference_valid = false;
    s_reference_since_ms = 0U;
    s_reference_gesture_latched = false;
}

void VehicleControl_Update(uint32_t now_ms)
{
    const RcData *rc = Rc_Get();
    float dt_s = (float)(uint32_t)(now_ms - s_last_ms) * 0.001f;
    bool references_valid;

    if (dt_s <= 0.0f) return;
    if (dt_s > 0.05f) dt_s = 0.05f;
    s_last_ms = now_ms;
    s_state.mode = (rc->switch_pos[VEHICLE_CH_MODE] == VEHICLE_ARM_MODE_LEVEL) ?
                   VEHICLE_MODE_ARM : VEHICLE_MODE_DRIVE;
    s_state.enabled = Rc_IsSignalValid();
    s_state.wrist_online = Arm2006_IsOnline(now_ms);

    if (!s_state.enabled)
    {
        VehicleControl_ForceStop();
        return;
    }

    references_valid = s_state.stepper_reference_valid && s_state.wrist_reference_valid;
    VehicleUpdateGripper(rc, dt_s);

    if (s_state.mode == VEHICLE_MODE_DRIVE)
    {
        if (references_valid)
        {
            uint8_t index = ArmPreset_DecodeIndex(
                rc->switch_pos[VEHICLE_CH_PRESET_A] == RC_SW_UP,
                rc->switch_pos[VEHICLE_CH_PRESET_D] == RC_SW_UP);
            VehicleApplyPose(ArmPreset_Get(index), now_ms, dt_s);
        }
        else
        {
            ArmStepper_Stop();
            Arm2006_Stop();
            ServoPair_HoldSafe();
        }
        Chassis_Update(now_ms);
        return;
    }

    Chassis_ForceStop();
    VehicleCaptureReference(rc, now_ms);
    references_valid = s_state.stepper_reference_valid && s_state.wrist_reference_valid;
    if (!references_valid)
    {
        ArmStepper_Stop();
        Arm2006_Stop();
        return;
    }

    ArmStepper_SetVelocity(VehicleAxis(rc, VEHICLE_CH_STEPPER), true, dt_s);
    Arm2006_SetVelocity(VehicleAxis(rc, VEHICLE_CH_WRIST));
    Arm2006_Update(now_ms, dt_s, true);
    ServoPair_SetLowerArmRate(VehicleAxis(rc, VEHICLE_CH_LOWER_ARM), dt_s);
}

const VehicleControlState *VehicleControl_GetState(void)
{
    return &s_state;
}
