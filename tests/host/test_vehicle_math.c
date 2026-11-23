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

    /* Lower-arm servo control is expressed directly in microseconds. */
    assert(Vehicle_UpdatePulseUs(1500.0f, 0.5f, 600.0f, 0.1f, 1200.0f, 1700.0f) == 1530.0f);
    assert(Vehicle_UpdatePulseUs(1500.0f, -0.5f, 600.0f, 0.1f, 1200.0f, 1700.0f) == 1470.0f);

    /* Software stops must prevent the commanded pulse from reaching a hard stop. */
    assert(Vehicle_UpdatePulseUs(1690.0f, 1.0f, 600.0f, 0.1f, 1200.0f, 1700.0f) == 1700.0f);
    assert(Vehicle_UpdatePulseUs(1210.0f, -1.0f, 600.0f, 0.1f, 1200.0f, 1700.0f) == 1200.0f);

    /* Fractional microseconds must accumulate instead of being truncated each 1 ms loop. */
    assert(Vehicle_AbsF(Vehicle_UpdatePulseUs(1500.0f, 1.0f, 600.0f, 0.001f,
                                             1200.0f, 1700.0f) - 1500.6f) < 0.001f);

    /* C610 ID 5 owns only slot 0 of the 0x1FF frame. */
    Vehicle_PackCurrentFrame(1234, 0, 0, 0, data);
    assert(data[0] == 0x04U && data[1] == 0xD2U);
    assert(data[2] == 0U && data[7] == 0U);

    puts("PASS test_vehicle_math");
    return 0;
}
