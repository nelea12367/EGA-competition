# Arm Fixed-Position Control Design

## Goal

Operate the arm only through four predefined poses referenced to a manually established startup zero. Keep the lower-arm servo at a pre-calibrated pulse-width zero. The user controls pose selection, hold, and return-to-zero from the receiver switches.

## Scope

- Two binary receiver switches, SWA and SWD, select one of four poses.
- SWB opens or closes the gripper.
- SWC selects the controller state:
  - Up: fixed-pose mode. Before a reference exists, capture the manually placed large-arm and wrist startup positions as zero. Afterwards move to the selected pose.
  - Middle: cancel any motion and hold the current pose.
  - Down: cancel any motion and execute the configured zero pose.
- The lower-arm servo zero is the calibrated `VEHICLE_LOWER_ARM_SERVO_ZERO_US`; it is not captured from physical motion.
- Fixed poses store the large-arm step count, lower-arm servo pulse width, and wrist output-shaft angle relative to the startup zero.

## Safety constraints

- No automatic recovery is allowed after a collision, step loss, manual repositioning, or suspected mechanical fault. The reference must be invalidated and re-established by manually placing the arm at its mechanical zero marks.
- Zero capture requires wrist CAN feedback to be online and a deliberate stable command interval; no fixed-pose motion occurs before it completes.
- The SWC middle position is a hold, not motor power-off: the stepper keeps holding torque, the wrist holds its current angle through the position controller, and the servo retains its current pulse.
- Returning to zero and moving to a preset must obey each axis's software limits and rate limits. A fixed pose change is started only when its selection changes; it is not restarted continuously while a switch is held.
- Receiver-loss behavior remains a force stop as in the existing firmware. A hardware power cut or emergency stop remains required because the requested switch allocation has no dedicated remote e-stop.

## Architecture

Add an `arm_presets` module which owns the fixed-pose table, switch-combination decoding, and a simple trajectory state: idle, moving to zero, or moving to selected pose. It exposes a selected target to `vehicle_control` and reports whether a target is active.

Extend each joint driver with an explicit target interface:

- `ArmStepper_SetTargetPulse()` produces a bounded pulse-frequency motion profile and considers its counted pulse position as its relative position. The existing manual velocity interface is retained only if tests or diagnostics still require it, but it is not used by the fixed-pose controller.
- `Arm2006_SetTargetRad()` sets an output-shaft target directly; the existing 2006 position loop continues to generate current commands.
- `ServoPair_SetLowerArmTargetUs()` slews the lower-arm command toward a calibrated pulse-width target.

`vehicle_control` maps SWA/SWD/SWB/SWC to the new controller. It captures startup references on the first stable SWC-up activation, holds on SWC-middle, and commands the zero target on SWC-down.

## Pose and calibration data

The configuration file holds four records in this form:

```c
typedef struct {
    int32_t stepper_pulse;
    uint16_t lower_arm_us;
    float wrist_rad;
} ArmPreset;
```

All values are relative to the manually captured large-arm/wrist zero and the calibrated lower-arm servo zero. The checked-in default table sets every record to the zero pose; commissioning replaces each record with measured values before automatic motion is enabled for that pose.

Calibration procedure:

1. With power removed or the relevant joint safe, align the large arm and wrist to repeatable mechanical zero marks.
2. Power the controller, wait for M2006 CAN feedback, and move SWC to up to capture that reference.
3. In a controlled test, measure large-arm pulse counts per desired pose, lower-arm safe PWM pulse widths, and M2006 output-shaft angles.
4. Enter those values into the four pose entries, verify one axis at a time, then verify each complete transition at reduced speed.

## Testing

Host tests must prove:

- SWA/SWD maps to all four preset indexes.
- SWC-up captures reference before accepting a preset.
- SWC-middle stops target motion while retaining the held joint targets.
- SWC-down selects the zero pose.
- No target can be commanded before both large-arm and wrist references are valid.
- Per-axis target APIs stop at configured software bounds.

Hardware commissioning remains required for switch-channel mapping, movement directions, current limits, servo safe range, and collision-free pose transitions.

