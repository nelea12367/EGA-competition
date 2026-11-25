#ifndef TEST_RC_INPUT_H
#define TEST_RC_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#define RC_SW_MID 0U
#define RC_SW_UP 1U
#define RC_SW_DOWN 2U

typedef struct
{
    float analog[16];
    uint8_t switch_pos[16];
} RcData;

const RcData *Rc_Get(void);
bool Rc_IsSignalValid(void);

#endif
