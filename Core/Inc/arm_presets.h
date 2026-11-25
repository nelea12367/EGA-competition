#ifndef ARM_PRESETS_H
#define ARM_PRESETS_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_PRESET_COUNT 4U

typedef struct { int32_t stepper_pulse; uint16_t lower_arm_us; float wrist_rad; } ArmPreset;
uint8_t ArmPreset_DecodeIndex(bool preset_a_up, bool preset_d_up);
const ArmPreset *ArmPreset_Get(uint8_t index);

#endif
