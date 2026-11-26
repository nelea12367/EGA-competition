#ifndef ARM_2006_H
#define ARM_2006_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool feedback_seen;
    bool reference_valid;
    uint8_t temperature_c;
    float angle_rad;
    float target_rad;
    float velocity_rad_s;
    int16_t current_command;
    uint32_t last_rx_ms;
} Arm2006State;

void Arm2006_Init(void);
void Arm2006_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len);
void Arm2006_SetReference(void);
/* 手动模式：归一化速度请求。推杆时走速度环，松杆后锁当前位置。 */
void Arm2006_SetVelocity(float normalized_velocity);
/* 预设模式：指定相对软件零点的目标角度（弧度），走位置保持环。 */
void Arm2006_SetTargetAngle(float target_rad);
void Arm2006_Update(uint32_t now_ms, float dt_s, bool enabled);
void Arm2006_Stop(void);
bool Arm2006_IsOnline(uint32_t now_ms);
const Arm2006State *Arm2006_GetState(void);

#endif /* ARM_2006_H */
