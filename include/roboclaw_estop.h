// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
//
// This file is part of balance_bot, licensed under the PolyForm
// Noncommercial License 1.0.0. You may use, study, modify, and share
// it for any noncommercial purpose. Commercial use requires a separate
// license from the author -- contact ryan.lush@gmail.com.
// Full license text: see the LICENSE file in the project root, or
// https://polyformproject.org/licenses/noncommercial/1.0.0/

#pragma once
#include <stdint.h>

// Resolves the e-stop pin from the running kernel's gpiochip layout and
// deasserts it. Returns 0 on success, -1 if the pin could not be resolved or
// exported — in which case assert/deassert become logged no-ops and there is
// NO working e-stop. Callers must check this.
int roboclaw_estop_init(void);
void roboclaw_estop_assert(void);
void roboclaw_estop_deassert(void);
// 1 = line high (deasserted), 0 = low (asserted), -1 = unresolved/unreadable.
int roboclaw_estop_get(void);
// Resolved sysfs GPIO number, or -1. For diagnostics and telemetry.
int roboclaw_estop_gpio(void);
int roboclaw_estop_clear(int fd, uint8_t addr);