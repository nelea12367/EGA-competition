#ifndef ARM_STEPPER_H
#define ARM_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t position_pulse;
    float command_hz;
    bool reference_valid;
    bool running;
    bool direction_positive;
} ArmStepperState;

void ArmStepper_Init(void);
void ArmStepper_SetReference(void);
void ArmStepper_SetVelocity(float velocity_normalized, bool enabled, float dt_s);
void ArmStepper_Stop(void);
void ArmStepper_OnTimerUpdate(void);
const ArmStepperState *ArmStepper_GetState(void);

#endif /* ARM_STEPPER_H */
