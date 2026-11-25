#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "arm_2006.h"
#include "arm_presets.h"
#include "arm_stepper.h"
#include "rc_input.h"

static RcData g_rc;
static bool g_rc_valid;
static bool g_wrist_online;
static uint32_t g_now;
static ArmStepperState g_stepper;
static Arm2006State g_wrist;
static unsigned g_stepper_reference_calls;
static unsigned g_wrist_reference_calls;
static unsigned g_stepper_target_calls;
static unsigned g_wrist_target_calls;
static unsigned g_stepper_hold_calls;
static unsigned g_wrist_hold_calls;
static ArmPreset g_preset = { 0, 1500U, 0.0f };

#include "../../Core/Src/vehicle_control.c"

uint32_t HAL_GetTick(void) { return g_now; }
const RcData *Rc_Get(void) { return &g_rc; }
bool Rc_IsSignalValid(void) { return g_rc_valid; }
uint8_t ArmPreset_DecodeIndex(bool swa_up, bool swd_up)
{
    return (uint8_t)((swa_up ? 0U : 2U) | (swd_up ? 0U : 1U));
}
const ArmPreset *ArmPreset_Get(uint8_t index) { (void)index; return &g_preset; }

void Chassis_Update(uint32_t now_ms) { (void)now_ms; }
void Chassis_ForceStop(void) { }
void ArmStepper_Init(void) { memset(&g_stepper, 0, sizeof(g_stepper)); }
void ArmStepper_SetReference(void) { g_stepper.reference_valid = true; g_stepper_reference_calls++; }
void ArmStepper_SetVelocity(float velocity_normalized, bool enabled, float dt_s)
{
    (void)velocity_normalized; (void)enabled; (void)dt_s;
}
void ArmStepper_SetTargetPulse(int32_t target_pulse, bool enabled, float dt_s)
{
    (void)target_pulse; (void)enabled; (void)dt_s; g_stepper_target_calls++;
}
void ArmStepper_HoldCurrentPosition(void) { g_stepper_hold_calls++; }
void ArmStepper_Stop(void) { }
const ArmStepperState *ArmStepper_GetState(void) { return &g_stepper; }
void Arm2006_Init(void) { memset(&g_wrist, 0, sizeof(g_wrist)); }
void Arm2006_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len)
{
    (void)id; (void)data; (void)len;
}
void Arm2006_SetReference(void) { g_wrist.reference_valid = true; g_wrist_reference_calls++; }
void Arm2006_SetVelocity(float normalized_velocity) { (void)normalized_velocity; }
void Arm2006_SetTargetRad(float target_rad) { (void)target_rad; g_wrist_target_calls++; }
void Arm2006_HoldCurrentPosition(void) { g_wrist_hold_calls++; }
void Arm2006_Update(uint32_t now_ms, float dt_s, bool enabled)
{
    (void)now_ms; (void)dt_s; (void)enabled;
}
void Arm2006_Stop(void) { }
bool Arm2006_IsOnline(uint32_t now_ms) { (void)now_ms; return g_wrist_online; }
const Arm2006State *Arm2006_GetState(void) { return &g_wrist; }
void ServoPair_Init(void) { }
void ServoPair_SetLowerArmRate(float normalized_rate, float dt_s)
{
    (void)normalized_rate; (void)dt_s;
}
void ServoPair_SetLowerArmTargetUs(uint16_t target_us, float dt_s)
{
    (void)target_us; (void)dt_s;
}
void ServoPair_SetGripperRate(float normalized_rate, float dt_s) { (void)normalized_rate; (void)dt_s; }
void ServoPair_HoldSafe(void) { }

int main(void)
{
    g_rc_valid = true;
    g_wrist_online = true;
    g_rc.switch_pos[VEHICLE_CH_SWA] = RC_SW_UP;
    g_rc.switch_pos[VEHICLE_CH_SWB] = VEHICLE_GRIPPER_OPEN_LEVEL;
    g_rc.switch_pos[VEHICLE_CH_SWC] = VEHICLE_SWC_ENABLE_LEVEL;
    g_rc.switch_pos[VEHICLE_CH_SWD] = RC_SW_UP;
    VehicleControl_Init();

    g_now = 1U;
    VehicleControl_Update(g_now);
    g_now = VEHICLE_REFERENCE_HOLD_MS + 1U;
    VehicleControl_Update(g_now);
    assert(g_stepper_reference_calls == 1U);
    assert(g_wrist_reference_calls == 1U);

    g_now++;
    VehicleControl_Update(g_now);
    assert(g_stepper_target_calls > 0U);
    assert(g_wrist_target_calls > 0U);

    g_rc.switch_pos[VEHICLE_CH_SWC] = VEHICLE_SWC_HOLD_LEVEL;
    g_now++;
    VehicleControl_Update(g_now);
    assert(g_stepper_hold_calls == 1U);
    assert(g_wrist_hold_calls == 1U);

    g_rc.switch_pos[VEHICLE_CH_SWC] = VEHICLE_SWC_RESET_LEVEL;
    g_now++;
    VehicleControl_Update(g_now);
    assert(g_stepper_target_calls > 1U);
    assert(g_wrist_target_calls > 1U);
    return 0;
}

