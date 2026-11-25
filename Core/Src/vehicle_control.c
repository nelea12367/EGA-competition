#include "vehicle_control.h"

#include "main.h"
#include "arm_2006.h"
#include "arm_presets.h"
#include "arm_stepper.h"
#include "chassis.h"
#include "rc_input.h"
#include "servo_pair.h"
#include "vehicle_config.h"

static VehicleControlState s_state;
static uint32_t s_last_ms;
static uint32_t s_reference_since_ms;
static uint8_t s_last_swc;

static void VehicleApplyPose(const ArmPreset *pose, uint32_t now_ms, float dt_s)
{
    ArmStepper_SetTargetPulse(pose->stepper_pulse, true, dt_s);
    Arm2006_SetTargetRad(pose->wrist_rad);
    Arm2006_Update(now_ms, dt_s, true);
    ServoPair_SetLowerArmTargetUs(pose->lower_arm_us, dt_s);
}

static void VehicleHoldPose(uint32_t now_ms, float dt_s, bool entered)
{
    if (entered)
    {
        ArmStepper_HoldCurrentPosition();
        Arm2006_HoldCurrentPosition();
    }
    Arm2006_Update(now_ms, dt_s, true);
    ServoPair_HoldSafe();
}

static bool VehicleCaptureReference(uint32_t now_ms)
{
    if (!s_state.wrist_online)
    {
        s_reference_since_ms = 0U;
        ArmStepper_Stop();
        Arm2006_Stop();
        return false;
    }

    if (s_reference_since_ms == 0U) s_reference_since_ms = now_ms;
    if ((uint32_t)(now_ms - s_reference_since_ms) < VEHICLE_REFERENCE_HOLD_MS)
    {
        ArmStepper_Stop();
        Arm2006_Stop();
        return false;
    }

    if (!s_state.stepper_reference_valid)
    {
        ArmStepper_SetReference();
        s_state.stepper_reference_valid = ArmStepper_GetState()->reference_valid;
    }
    if (!s_state.wrist_reference_valid)
    {
        Arm2006_SetReference();
        s_state.wrist_reference_valid = Arm2006_GetState()->reference_valid;
    }
    return s_state.stepper_reference_valid && s_state.wrist_reference_valid;
}

static void VehicleUpdateGripper(const RcData *rc, float dt_s)
{
    if (rc->switch_pos[VEHICLE_CH_SWB] == VEHICLE_GRIPPER_OPEN_LEVEL)
        ServoPair_SetGripperRate(1.0f, dt_s);
    else if (rc->switch_pos[VEHICLE_CH_SWB] == VEHICLE_GRIPPER_CLOSE_LEVEL)
        ServoPair_SetGripperRate(-1.0f, dt_s);
    else
        ServoPair_HoldSafe();
}

void VehicleControl_Init(void)
{
    s_state.mode = VEHICLE_MODE_ARM;
    s_state.enabled = false;
    s_state.stepper_reference_valid = false;
    s_state.wrist_reference_valid = false;
    s_state.wrist_online = false;
    s_state.holding = true;
    s_state.returning_zero = false;
    s_state.active_preset = 0U;
    s_last_ms = HAL_GetTick();
    s_reference_since_ms = 0U;
    s_last_swc = 0xFFU;
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
    s_state.holding = true;
    s_state.returning_zero = false;
    s_reference_since_ms = 0U;
}

void VehicleControl_Update(uint32_t now_ms)
{
    const RcData *rc = Rc_Get();
    float dt_s = (float)(uint32_t)(now_ms - s_last_ms) * 0.001f;
    uint8_t swc;
    bool entered;

    if (dt_s <= 0.0f) return;
    if (dt_s > 0.05f) dt_s = 0.05f;
    s_last_ms = now_ms;
    s_state.mode = VEHICLE_MODE_ARM;
    s_state.wrist_online = Arm2006_IsOnline(now_ms);

    if (!Rc_IsSignalValid())
    {
        VehicleControl_ForceStop();
        return;
    }

    Chassis_ForceStop();
    VehicleUpdateGripper(rc, dt_s);
    swc = rc->switch_pos[VEHICLE_CH_SWC];
    entered = (swc != s_last_swc);
    s_last_swc = swc;

    if (swc == VEHICLE_SWC_ENABLE_LEVEL)
    {
        s_state.enabled = true;
        s_state.holding = false;
        s_state.returning_zero = false;
        if (!s_state.stepper_reference_valid || !s_state.wrist_reference_valid)
        {
            (void)VehicleCaptureReference(now_ms);
            return;
        }

        s_state.active_preset = ArmPreset_DecodeIndex(
            rc->switch_pos[VEHICLE_CH_SWA] == VEHICLE_SWA_UP_LEVEL,
            rc->switch_pos[VEHICLE_CH_SWD] == VEHICLE_SWD_UP_LEVEL);
        VehicleApplyPose(ArmPreset_Get(s_state.active_preset), now_ms, dt_s);
        return;
    }

    s_state.enabled = true;
    if (swc == VEHICLE_SWC_HOLD_LEVEL)
    {
        s_state.holding = true;
        s_state.returning_zero = false;
        if (s_state.stepper_reference_valid && s_state.wrist_reference_valid)
            VehicleHoldPose(now_ms, dt_s, entered);
        else
        {
            ArmStepper_Stop();
            Arm2006_Stop();
        }
        return;
    }

    if (swc == VEHICLE_SWC_RESET_LEVEL)
    {
        s_state.holding = false;
        s_state.returning_zero = true;
        if (s_state.stepper_reference_valid && s_state.wrist_reference_valid)
            VehicleApplyPose(ArmPreset_Get(0U), now_ms, dt_s);
        else
        {
            ArmStepper_Stop();
            Arm2006_Stop();
        }
        return;
    }

    s_state.holding = true;
    s_state.returning_zero = false;
    VehicleHoldPose(now_ms, dt_s, entered);
}

const VehicleControlState *VehicleControl_GetState(void)
{
    return &s_state;
}

