#pragma once
#include <stdint.h>

int roboclaw_estop_init(void);
void roboclaw_estop_assert(void);
void roboclaw_estop_deassert(void);
int roboclaw_estop_clear(int fd, uint8_t addr);