#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    VEHICLE_MODE_DRIVE = 0,
    VEHICLE_MODE_ARM
} VehicleMode;

typedef struct
{
    VehicleMode mode;
    bool enabled;
    bool stepper_reference_valid;
    bool wrist_reference_valid;
    bool wrist_online;
    bool holding;
    bool returning_zero;
    uint8_t active_preset;
} VehicleControlState;

void VehicleControl_Init(void);
void VehicleControl_Update(uint32_t now_ms);
void VehicleControl_OnCanRx(uint32_t id, const uint8_t *data, uint8_t len);
void VehicleControl_ForceStop(void);
const VehicleControlState *VehicleControl_GetState(void);

#endif /* VEHICLE_CONTROL_H */

