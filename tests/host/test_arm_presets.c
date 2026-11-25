#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm_presets.h"
#include "vehicle_config.h"

int main(void)
{
    assert(ArmPreset_DecodeIndex(true, true) == 0U);
    assert(ArmPreset_DecodeIndex(true, false) == 1U);
    assert(ArmPreset_DecodeIndex(false, true) == 2U);
    assert(ArmPreset_DecodeIndex(false, false) == 3U);
    assert(ArmPreset_Get(0U)->stepper_pulse == 0);
    assert(ArmPreset_Get(0U)->lower_arm_us == VEHICLE_LOWER_ARM_SERVO_ZERO_US);
    assert(ArmPreset_Get(0U)->wrist_rad == 0.0f);
    assert(ArmPreset_Get(ARM_PRESET_COUNT + 1U) == ArmPreset_Get(0U));
    return 0;
}

