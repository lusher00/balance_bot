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

GPIO=/sys/class/gpio/gpio537
if [ ! -d "$GPIO" ]; then
    echo 537 > /sys/class/gpio/export
    sleep 0.15
fi
echo out > ${GPIO}/direction
echo 1   > ${GPIO}/value
echo "E-stop cleared, GPIO537=1"
