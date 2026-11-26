#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm_2006.h"
#include "main.h"

static uint32_t g_now;
static int16_t g_last_current;

#include "../../Core/Src/arm_2006.c"

uint32_t HAL_GetTick(void) { return g_now; }

float Vehicle_AbsF(float value) { return value < 0.0f ? -value : value; }
float Vehicle_ClampF(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}
float Vehicle_SlewF(float current, float target, float rate_per_s, float dt_s)
{
    float step = rate_per_s * dt_s;
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

bool Can_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
    (void)id;
    assert(len == 8U);
    g_last_current = (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
    return true;
}
void Vehicle_PackCurrentFrame(int16_t motor_1, int16_t motor_2,
                              int16_t motor_3, int16_t motor_4,
                              uint8_t out[8])
{
    (void)motor_2;
    (void)motor_3;
    (void)motor_4;
    out[0] = (uint8_t)((uint16_t)motor_1 >> 8U);
    out[1] = (uint8_t)motor_1;
    out[2] = out[3] = out[4] = out[5] = out[6] = out[7] = 0U;
}

static void FeedWrist(uint16_t encoder, int16_t speed_rpm)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)(encoder >> 8U);
    data[1] = (uint8_t)encoder;
    data[2] = (uint8_t)((uint16_t)speed_rpm >> 8U);
    data[3] = (uint8_t)speed_rpm;
    data[6] = 30U;
    Arm2006_OnCanRx(VEHICLE_WRIST_ID, data, 8U);
}

static void AssertNear(float actual, float expected, float tolerance)
{
    assert(Vehicle_AbsF(actual - expected) <= tolerance);
}

/* Catches a return to target_rad += velocity * dt: manual motion must never
 * accumulate a position target, and releasing the stick must latch the then
 * measured angle for the position hold loop. */
static void TestManualVelocityDoesNotIntegrateAndReleaseLatchesPosition(void)
{
    const Arm2006State *state;

    g_now = 1U;
    Arm2006_Init();
    FeedWrist(100U, 0);
    Arm2006_SetReference();

    Arm2006_SetVelocity(1.0f);
    g_now = 2U;
    Arm2006_Update(g_now, 0.010f, true);
    state = Arm2006_GetState();
    AssertNear(state->target_rad, state->angle_rad, 0.0001f);
    assert(g_last_current > 0);

    /* Simulate motion before the stick is released. */
    g_now = 3U;
    FeedWrist(1100U, 0);
    Arm2006_SetVelocity(0.0f);
    Arm2006_Update(g_now, 0.010f, true);
    state = Arm2006_GetState();
    AssertNear(state->target_rad, state->angle_rad, 0.0001f);
}

/* Catches a regression where the new manual velocity mode prevents a preset
 * from requesting a concrete wrist angle. */
static void TestPositionTargetRemainsAvailableForPresets(void)
{
    const Arm2006State *state;

    g_now = 10U;
    Arm2006_Init();
    FeedWrist(100U, 0);
    Arm2006_SetReference();
    Arm2006_SetTargetAngle(0.50f);
    g_now = 12U;
    Arm2006_Update(g_now, 0.010f, true);
    state = Arm2006_GetState();
    AssertNear(state->target_rad, 0.50f, 0.0001f);
    assert(g_last_current > 0);
}

int main(void)
{
    TestManualVelocityDoesNotIntegrateAndReleaseLatchesPosition();
    TestPositionTargetRemainsAvailableForPresets();
    return 0;
}
