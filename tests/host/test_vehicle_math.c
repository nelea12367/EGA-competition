#include "vehicle_math.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

int main(void)
{
    uint8_t data[8] = {0};

    /* A missing clamp would allow an arm target to exceed a software stop. */
    assert(Vehicle_ClampF(12.0f, -5.0f, 5.0f) == 5.0f);
    assert(Vehicle_ClampF(-12.0f, -5.0f, 5.0f) == -5.0f);

    /* A missing slew limit would cause an unsafe commanded velocity jump. */
    assert(Vehicle_SlewF(0.0f, 10.0f, 2.0f, 0.5f) == 1.0f);
    assert(Vehicle_SlewF(1.0f, -10.0f, 2.0f, 0.5f) == 0.0f);

    /* TIM counters at 1 MHz must preserve the requested microsecond pulse. */
    assert(Vehicle_PulseUsToCompare(1500U, 1000000U) == 1500U);

    /* C610 ID 5 owns only slot 0 of the 0x1FF frame. */
    Vehicle_PackCurrentFrame(1234, 0, 0, 0, data);
    assert(data[0] == 0x04U && data[1] == 0xD2U);
    assert(data[2] == 0U && data[7] == 0U);

    puts("PASS test_vehicle_math");
    return 0;
}
