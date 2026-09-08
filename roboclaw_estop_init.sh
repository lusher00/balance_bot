#!/bin/bash
# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
#
# This file is part of balance_bot, licensed under the PolyForm
# Noncommercial License 1.0.0. You may use, study, modify, and share
# it for any noncommercial purpose. Commercial use requires a separate
# license from the author -- contact ryan.lush@gmail.com.
# Full license text: see the LICENSE file in the project root, or
# https://polyformproject.org/licenses/noncommercial/1.0.0/

# Initialize RoboClaw E-stop GPIO 57
# Active-low: 0=estop, 1=release

GPIO=57
GPIO_PATH=/sys/class/gpio/gpio${GPIO}

# Export if not already exported
if [ ! -d "$GPIO_PATH" ]; then
    echo $GPIO > /sys/class/gpio/export
    sleep 0.1
fi

echo out > ${GPIO_PATH}/direction
echo 1 > ${GPIO_PATH}/value  # Release e-stop on start

exit 0
