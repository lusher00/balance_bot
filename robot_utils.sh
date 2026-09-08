# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
#
# This file is part of balance_bot, licensed under the PolyForm
# Noncommercial License 1.0.0. You may use, study, modify, and share
# it for any noncommercial purpose. Commercial use requires a separate
# license from the author -- contact ryan.lush@gmail.com.
# Full license text: see the LICENSE file in the project root, or
# https://polyformproject.org/licenses/noncommercial/1.0.0/

estop_init() {
    if [ ! -d /sys/class/gpio/gpio537 ]; then
        sudo sh -c 'echo 537 > /sys/class/gpio/export'
        sleep 0.2
    fi
    echo out > /sys/class/gpio/gpio537/direction
    echo 1   > /sys/class/gpio/gpio537/value
}
estop_active() {
    echo 0 > /sys/class/gpio/gpio537/value
}
estop_release() {
    echo 1 > /sys/class/gpio/gpio537/value
}
