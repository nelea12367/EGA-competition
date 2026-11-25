#ifndef TEST_ARM_PRESETS_H
#define TEST_ARM_PRESETS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t stepper_pulse;
    uint16_t lower_arm_us;
    float wrist_rad;
} ArmPreset;

uint8_t ArmPreset_DecodeIndex(bool swa_up, bool swd_up);
const ArmPreset *ArmPreset_Get(uint8_t index);

#endif

