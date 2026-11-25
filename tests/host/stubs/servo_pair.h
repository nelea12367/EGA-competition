#ifndef TEST_SERVO_PAIR_H
#define TEST_SERVO_PAIR_H

void ServoPair_Init(void);
void ServoPair_SetLowerArmRate(float normalized_rate, float dt_s);
void ServoPair_SetGripperRate(float normalized_rate, float dt_s);
void ServoPair_HoldSafe(void);

#endif
