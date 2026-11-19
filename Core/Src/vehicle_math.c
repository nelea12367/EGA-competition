#include "vehicle_math.h"

float Vehicle_ClampF(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

float Vehicle_AbsF(float value)
{
    return (value < 0.0f) ? -value : value;
}

float Vehicle_SlewF(float current, float target, float maximum_rate, float dt_s)
{
    float limit = maximum_rate * dt_s;
    if (limit < 0.0f) limit = 0.0f;
    return Vehicle_ClampF(target, current - limit, current + limit);
}

uint32_t Vehicle_PulseUsToCompare(uint16_t pulse_us, uint32_t timer_hz)
{
    return ((uint32_t)pulse_us * timer_hz) / 1000000U;
}

void Vehicle_PackCurrentFrame(int16_t slot0, int16_t slot1, int16_t slot2, int16_t slot3,
                              uint8_t data[8])
{
    const int16_t slots[4] = {slot0, slot1, slot2, slot3};
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        uint16_t value = (uint16_t)slots[i];
        data[2U * i] = (uint8_t)(value >> 8U);
        data[2U * i + 1U] = (uint8_t)value;
    }
}
