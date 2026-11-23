#ifndef SERVO_PAIR_H
#define SERVO_PAIR_H

#include <stdint.h>

typedef struct
{
    float wrist_command_us;
    float gripper_deg;
    uint16_t wrist_us;
    uint16_t gripper_us;
} ServoPairState;

void ServoPair_Init(void);
void ServoPair_SetWristRate(float normalized_rate, float dt_s);
void ServoPair_SetGripperRate(float normalized_rate, float dt_s);
void ServoPair_HoldSafe(void);
const ServoPairState *ServoPair_GetState(void);

#endif /* SERVO_PAIR_H */
