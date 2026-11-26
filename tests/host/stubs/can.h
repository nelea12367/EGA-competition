#ifndef TEST_CAN_H
#define TEST_CAN_H

#include <stdbool.h>
#include <stdint.h>

bool Can_Send(uint32_t id, const uint8_t *data, uint8_t len);
void Vehicle_PackCurrentFrame(int16_t motor_1, int16_t motor_2,
                              int16_t motor_3, int16_t motor_4,
                              uint8_t out[8]);

#endif
