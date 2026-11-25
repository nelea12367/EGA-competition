# Hybrid Chassis, Manual Arm, and Pose Control Plan

**Goal:** Preserve the original chassis/manual-arm control and add four selectable arm poses plus a protected manual zero-capture gesture.

**Design:** SWB remains the chassis/manual-arm mode selector. In chassis mode, sticks drive the chassis and SWA/SWD choose one of four arm poses. In manual-arm mode, sticks control the three arm axes; SWA/SWD both up plus all three arm sticks centered for 1.5 seconds records the manually aligned zero. SWA/SWD both up is pose 0 (home) in chassis mode. SWC remains gripper control.

### Task 1: Preset configuration and target APIs

- [ ] Write failing tests for SWA/SWD decoding and clamped targets.
- [ ] Add an `arm_presets` table, zero-pose defaults, and explicit target APIs for the stepper, wrist, and lower-arm servo.
- [ ] Verify target bounds and build.

### Task 2: Hybrid state machine

- [ ] Write failing host/stub tests for chassis mode, manual-arm mode, pose selection, and guarded zero capture.
- [ ] In chassis mode call `Chassis_Update()` and command the selected pose only after zero exists.
- [ ] In manual-arm mode force-stop chassis, retain joystick joint control, and accept zero capture only after the full 1.5-second gesture.
- [ ] Verify host checks and firmware build.

### Task 3: Commissioning documentation

- [ ] Document the switch mapping, zero-capture gesture, pose-0 home behavior, collision re-reference requirement, and pose calibration sequence.
- [ ] Run final static checks and the STM32 build.
