#ifndef TEST_VEHICLE_MATH_H
#define TEST_VEHICLE_MATH_H

float Vehicle_AbsF(float value);
float Vehicle_ClampF(float value, float min_value, float max_value);
float Vehicle_SlewF(float current, float target, float rate_per_s, float dt_s);

#endif
