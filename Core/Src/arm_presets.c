#include "arm_presets.h"

#include "vehicle_config.h"

static const ArmPreset s_presets[ARM_PRESET_COUNT] =
{
    { VEHICLE_PRESET_0_STEPPER_PULSE, VEHICLE_PRESET_0_LOWER_ARM_US, VEHICLE_PRESET_0_WRIST_RAD },
    { VEHICLE_PRESET_1_STEPPER_PULSE, VEHICLE_PRESET_1_LOWER_ARM_US, VEHICLE_PRESET_1_WRIST_RAD },
    { VEHICLE_PRESET_2_STEPPER_PULSE, VEHICLE_PRESET_2_LOWER_ARM_US, VEHICLE_PRESET_2_WRIST_RAD },
    { VEHICLE_PRESET_3_STEPPER_PULSE, VEHICLE_PRESET_3_LOWER_ARM_US, VEHICLE_PRESET_3_WRIST_RAD }
};

uint8_t ArmPreset_DecodeIndex(bool swa_up, bool swd_up)
{
    return (uint8_t)((swa_up ? 0U : 2U) | (swd_up ? 0U : 1U));
}

const ArmPreset *ArmPreset_Get(uint8_t index)
{
    if (index >= ARM_PRESET_COUNT) index = 0U;
    return &s_presets[index];
}

