#ifndef SERVO_PAIR_H
#define SERVO_PAIR_H

#include <stdint.h>

typedef struct
{
    float lower_arm_command_us;
    float gripper_deg;
    uint16_t lower_arm_us;
    uint16_t gripper_us;
} ServoPairState;

void ServoPair_Init(void);
void ServoPair_SetLowerArmRate(float normalized_rate, float dt_s);
void ServoPair_SetLowerArmTargetUs(uint16_t target_us, float dt_s);
void ServoPair_SetGripperRate(float normalized_rate, float dt_s);
void ServoPair_HoldSafe(void);
const ServoPairState *ServoPair_GetState(void);

#endif /* SERVO_PAIR_H */
