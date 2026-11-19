#ifndef VEHICLE_MATH_H
#define VEHICLE_MATH_H

#include <stdint.h>

float Vehicle_ClampF(float value, float lower, float upper);
float Vehicle_AbsF(float value);
float Vehicle_SlewF(float current, float target, float maximum_rate, float dt_s);
uint32_t Vehicle_PulseUsToCompare(uint16_t pulse_us, uint32_t timer_hz);
void Vehicle_PackCurrentFrame(int16_t slot0, int16_t slot1, int16_t slot2, int16_t slot3,
                              uint8_t data[8]);

#endif /* VEHICLE_MATH_H */
