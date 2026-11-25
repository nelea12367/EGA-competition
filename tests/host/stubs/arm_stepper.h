#ifndef TEST_ARM_STEPPER_H
#define TEST_ARM_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool reference_valid;
} ArmStepperState;

void ArmStepper_Init(void);
void ArmStepper_SetReference(void);
void ArmStepper_SetVelocity(float velocity_normalized, bool enabled, float dt_s);
void ArmStepper_SetTargetPulse(int32_t target_pulse, bool enabled, float dt_s);
void ArmStepper_HoldCurrentPosition(void);
void ArmStepper_Stop(void);
const ArmStepperState *ArmStepper_GetState(void);

#endif

