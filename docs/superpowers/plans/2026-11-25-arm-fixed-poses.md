# Arm Fixed-Position Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Control the arm through four SWA/SWD-selected poses; SWC is enable/hold/return-zero and SWB controls the gripper.

**Architecture:** A small `arm_presets` module provides four configuration records. `vehicle_control` captures the manually placed stepper/M2006 reference once, then turns receiver states into bounded targets for existing joint drivers.

**Tech Stack:** STM32H723 HAL C11, PowerShell/GCC host tests.

**Spec:** `docs/superpowers/specs/2026-11-25-arm-fixed-poses-design.md`

## Global Constraints

- SWC up captures zero only while unreferenced; middle holds torque; down returns to zero.
- Lower-arm zero is the configured calibrated pulse width, never an observed physical reading.
- A possible collision/step loss requires power-cycle plus manual re-reference.
- Default pose records are all zero; physical measurements replace them before automatic motion.

### Task 1: Add preset records and decoder

**Files:** Create `Core/Inc/arm_presets.h`, `Core/Src/arm_presets.c`, `tests/host/test_arm_presets.c`; modify `Core/Inc/vehicle_config.h`, `tests/host/run_tests.ps1`.

- [ ] **Step 1: Write the failing decoder test**

```c
assert(ArmPreset_DecodeIndex(true, true) == 0U);
assert(ArmPreset_DecodeIndex(true, false) == 1U);
assert(ArmPreset_DecodeIndex(false, true) == 2U);
assert(ArmPreset_DecodeIndex(false, false) == 3U);
```

- [ ] **Step 2: Run it and observe the missing-module failure**

Run: `powershell -ExecutionPolicy Bypass -File .\tests\host\run_tests.ps1`

- [ ] **Step 3: Implement the minimal table**

```c
uint8_t ArmPreset_DecodeIndex(bool swa_up, bool swd_up)
{
    return (uint8_t)((swa_up ? 0U : 2U) | (swd_up ? 0U : 1U));
}
```

Expose a bounded getter; all four default records are `{ 0, VEHICLE_LOWER_ARM_SERVO_ZERO_US, 0.0f }`.

- [ ] **Step 4: Run the host suite and verify green**

Run: `powershell -ExecutionPolicy Bypass -File .\tests\host\run_tests.ps1`

### Task 2: Add target APIs for the three joints

**Files:** Modify `arm_stepper.[ch]`, `arm_2006.[ch]`, `servo_pair.[ch]`; create `tests/host/test_arm_targets.c`; modify `tests/host/run_tests.ps1`.

- [ ] **Step 1: Write failing clamp tests**

```c
ArmStepper_SetTargetPulse(VEHICLE_STEPPER_MAX_PULSE + 1, true, 0.01f);
assert(ArmStepper_GetState()->target_pulse == VEHICLE_STEPPER_MAX_PULSE);
Arm2006_SetTargetRad(VEHICLE_WRIST_MAX_RAD + 1.0f);
assert(Arm2006_GetState()->target_rad == VEHICLE_WRIST_MAX_RAD);
```

- [ ] **Step 2: Run the suite and observe the missing API failure**

Run: `powershell -ExecutionPolicy Bypass -File .\tests\host\run_tests.ps1`

- [ ] **Step 3: Implement bounded targets**

```c
float error = (float)bounded_target - (float)s_state.position_pulse;
float request = (error > 0.0f) ? 1.0f : ((error < 0.0f) ? -1.0f : 0.0f);
ArmStepper_SetVelocity(request, enabled, dt_s);
```

Stop within tolerance; wrist reuses its existing position loop; lower servo reuses its pulse slew.

- [ ] **Step 4: Run the complete host suite**

Run: `powershell -ExecutionPolicy Bypass -File .\tests\host\run_tests.ps1`

### Task 3: Integrate receiver state machine

**Files:** Modify `Core/Inc/vehicle_config.h`, `Core/Inc/vehicle_control.h`, `Core/Src/vehicle_control.c`; create `tests/host/test_fixed_pose_control.c`; modify `tests/host/run_tests.ps1`.

- [ ] **Step 1: Write the failing SWC test**

```c
g_rc.switch_pos[VEHICLE_CH_SWC] = RC_SW_UP;
VehicleControl_Update(VEHICLE_REFERENCE_HOLD_MS + 1U);
assert(g_stepper_reference_calls == 1U);
g_rc.switch_pos[VEHICLE_CH_SWC] = RC_SW_DOWN;
VehicleControl_Update(VEHICLE_REFERENCE_HOLD_MS + 2U);
assert(g_stepper_target_calls > 0U);
```

- [ ] **Step 2: Run the suite and observe the missing routing failure**

Run: `powershell -ExecutionPolicy Bypass -File .\tests\host\run_tests.ps1`

- [ ] **Step 3: Route the switch state**

```c
switch (rc->switch_pos[VEHICLE_CH_SWC]) {
case RC_SW_UP:  VehicleFixedPoseEnable(rc, now_ms, dt_s); break;
case RC_SW_MID: VehicleFixedPoseHold(dt_s); break;
default:        VehicleFixedPoseReturnZero(dt_s); break;
}
```

Up records only after M2006 feedback is online; later up calls selected preset, middle holds targets, down requests zero, and SWB opens/closes the gripper.

- [ ] **Step 4: Run all host tests**

Run: `powershell -ExecutionPolicy Bypass -File .\tests\host\run_tests.ps1`

### Task 4: Verify and commission

**Files:** Modify `README_整车控制上手.md`, `README_FOR_NEXT_AGENT.md`.

- [ ] **Step 1: Document mapping, startup reference, servo calibration, and collision re-reference.**

- [ ] **Step 2: Run host suite and firmware build**

Run: `powershell -ExecutionPolicy Bypass -File .\tests\host\run_tests.ps1`

Run: `powershell -ExecutionPolicy Bypass -File .\build.ps1 -ToolchainBin '<configured STM32 toolchain path>' -OutName DM_MC02_Vehicle`

