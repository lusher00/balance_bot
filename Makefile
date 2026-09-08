# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2025-2026 Ryan Lush <ryan.lush@gmail.com>
#
# This file is part of balance_bot, licensed under the PolyForm
# Noncommercial License 1.0.0. You may use, study, modify, and share
# it for any noncommercial purpose. Commercial use requires a separate
# license from the author -- contact ryan.lush@gmail.com.
# Full license text: see the LICENSE file in the project root, or
# https://polyformproject.org/licenses/noncommercial/1.0.0/

# balance_bot Makefile
# Self-balancing robot with iPhone app integration

TARGET = balance_bot
CC = gcc
CFLAGS = -Wall -Wextra -O2 -Iinclude
LDFLAGS = -lm -lpthread -lncurses

# Motor HAL drive mode and QPPS are now runtime-tunable via the IPC
# set_motor_config command — no compile-time flags needed.
# Defaults are set by MOTOR_HAL_MODE_DEFAULT / MOTOR_QPPS_MAX_DEFAULT
# / MOTOR_ACCEL_QPPS_DEFAULT in balance_bot.h.

# Source files
# Motor HAL — RoboClaw packet-serial over UART
MOTOR_HAL = src/motor_hal_roboclaw.c

SRCS = src/main.c \
       src/robot.c \
       src/display.c \
       src/pid.c \
       src/robot_config.c \
       src/uart_input.c \
       src/roboclaw.c \
       src/roboclaw_estop.c \
	   src/ipc_server.c \
       src/telemetry.c \
       src/input_xbox.c \
       src/input_sbus.c \
       src/imu_config.c \
       src/mpu_dmp.c \
       src/dmp_firmware.c \
       $(MOTOR_HAL)

# Object files
OBJS = $(patsubst src/%.c,$(OBJDIR)/%.o,$(SRCS))

# Binary and object directories
BINDIR = bin
OBJDIR = obj

.PHONY: all clean install install-oled uninstall uninstall-oled test

# Default target
all: $(BINDIR)/$(TARGET)

# Link
$(BINDIR)/$(TARGET): $(OBJS)
	@mkdir -p $(BINDIR)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)
	@echo ""
	@echo "✅ Build complete: $(BINDIR)/$(TARGET)"
	@echo "   Run: sudo ./$(BINDIR)/$(TARGET)"
	@echo "   Motor mode/QPPS tunable at runtime via set_motor_config IPC"
	@echo ""

# Compile
# -MMD -MP writes a .d file per object listing the headers it used, and the
# -include below feeds those back to make. Without this, changing a header does
# NOT rebuild anything: the rule only depends on the .c.
#
# That is not a slow-build annoyance, it is a correctness bug. Adding a field to
# a struct in balance_bot.h changes the layout of everything containing it, so a
# stale object reads the wrong offsets and you get silent garbage -- garbled
# motor_config in telemetry, pol_l reading 0 and the motors going dead, and a
# display.c type error that stayed hidden behind a stale obj/display.o.
$(OBJDIR)/%.o: src/%.c
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

-include $(OBJS:.o=.d)

# Clean
clean:
	rm -rf $(OBJDIR) $(BINDIR)/$(TARGET)
	@echo "Cleaned build artifacts"

# Service name
SERVICE = balance_bot.service
SERVER_SERVICE = balance_bot_server.service
UNIT_DIR = /etc/systemd/system
DEFAULTS = /etc/default/balance_bot
ROBOT_CONF = robot.conf

# ── OLED status display ───────────────────────────────────────────────────────
# Part of this project, but deliberately not dependent on it: bbb_oled.py draws
# the IP, service state, battery and time from I2C and /run/batt_status.json, so
# it is useful on a board where balance_bot is stopped or not installed at all.
# That is the whole point of it — it is what tells you the board is alive when
# the robot is not.
OLED_DIR = oled-utils
OLED_SERVICE = bbb_oled.service

# ── Machine state ────────────────────────────────────────────────────────────
# Files that live outside the repo but are expensive to recreate. The IMU
# calibration in particular is derived empirically (balance the bot, zero_imu,
# then trim) and exists nowhere else — losing it costs an hour of bench work.
#
# These are snapshots of THIS board, not defaults. save-config pulls the live
# values in so you can commit them; install-config pushes them back to a fresh
# board but never clobbers a file that is already there.
save-config:
	@mkdir -p config/machine
	@if [ -f $(ROBOT_CONF) ]; then \
		cp $(ROBOT_CONF) config/machine/robot.conf; \
		echo "saved $(ROBOT_CONF)"; \
	else echo "WARN: $(ROBOT_CONF) not found (run this on the bot)"; fi
	@if [ -f $(DEFAULTS) ]; then \
		cp $(DEFAULTS) config/machine/balance_bot.default; \
		echo "saved $(DEFAULTS)"; \
	else echo "WARN: $(DEFAULTS) not found"; fi
	@echo "Now commit config/machine/ — this is this board's calibration."

install-config:
	@mkdir -p config/machine
	@if [ ! -f $(ROBOT_CONF) ] && [ -f config/machine/robot.conf ]; then \
		cp config/machine/robot.conf $(ROBOT_CONF); \
		echo "restored $(ROBOT_CONF)"; \
	else \
		echo "Kept existing $(ROBOT_CONF) — refusing to overwrite a live calibration."; \
		echo "  (delete it first if you really want the committed one)"; \
	fi
	@if [ ! -f $(DEFAULTS) ] && [ -f config/machine/balance_bot.default ]; then \
		sudo cp config/machine/balance_bot.default $(DEFAULTS); \
		echo "restored $(DEFAULTS)"; \
	else echo "Kept existing $(DEFAULTS)"; fi

# Install the unit files from systemd/ — these are the versioned copies.
# Anything machine-specific (device paths, baud, input mode) belongs in
# $(DEFAULTS), which is deliberately NOT overwritten if it already exists.
install-units:
	@echo "Installing unit files to $(UNIT_DIR)..."
	sudo cp systemd/$(SERVICE) $(UNIT_DIR)/$(SERVICE)
	sudo cp systemd/$(SERVER_SERVICE) $(UNIT_DIR)/$(SERVER_SERVICE)
	@if [ ! -f $(DEFAULTS) ]; then \
		sudo cp systemd/balance_bot.default.example $(DEFAULTS); \
		echo "Created $(DEFAULTS) from example — EDIT IT for this board"; \
	else \
		echo "Kept existing $(DEFAULTS) (edit by hand; not managed by make)"; \
	fi
	sudo systemctl daemon-reload

# Install to system
#
# This used to stop and restart $(SERVICE)/$(SERVER_SERVICE) unconditionally,
# every time -- including a `make install` where every object was already up
# to date and the link step had nothing to do. That bounces a robot that is
# armed and balancing for zero reason: no new code reached it, just a few
# seconds of motors off and a fresh robot.conf load. "Nothing to build" must
# mean "nothing to restart". Only bounce the services when the binary that
# would land in /usr/local/bin actually differs from what is already there.
install: $(BINDIR)/$(TARGET) install-units
	@if sudo cmp -s $(BINDIR)/$(TARGET) /usr/local/bin/$(TARGET) 2>/dev/null; then \
		echo "$(TARGET) unchanged — $(SERVICE) and $(SERVER_SERVICE) left running."; \
	else \
		echo "Stopping $(SERVICE) and $(SERVER_SERVICE)..."; \
		sudo systemctl stop $(SERVICE) || true; \
		sudo systemctl stop $(SERVER_SERVICE) || true; \
		echo "Installing $(TARGET) to /usr/local/bin/..."; \
		sudo cp $(BINDIR)/$(TARGET) /usr/local/bin/$(TARGET); \
		echo "Restarting $(SERVICE) and $(SERVER_SERVICE)..."; \
		sudo systemctl start $(SERVICE) || true; \
		sudo systemctl start $(SERVER_SERVICE) || true; \
		echo ""; \
		echo "✅ Installed and restarted: /usr/local/bin/$(TARGET)"; \
		echo ""; \
	fi
	@# No config seeding here. balance_bot writes robot.conf itself on first
	@# run, migrating from pidconfig.txt + /etc/balance_bot_imu.conf if present.
	@# The old block here wrote a positional file with gains that were not this
	@# robot's, which the migration would then have adopted.
	@# OLED last, and deliberately non-fatal. install.sh refuses to proceed if
	@# luma.oled/pillow are missing for the service user, and a status display
	@# failing its dependency check must not leave the robot uninstalled — by
	@# this point balance_bot is already copied and running. Run
	@# `make install-oled` on its own to see the failure and act on it.
	@$(MAKE) --no-print-directory install-oled || { \
		echo ""; \
		echo "⚠️  OLED display NOT installed — see the messages above."; \
		echo "   $(TARGET) itself is installed and running."; \
		echo "   Fix the dependencies, then: make install-oled"; \
		echo ""; }

# Install the OLED status display: bbb_oled.py -> /usr/local/bin, unit ->
# $(UNIT_DIR), enabled so it comes up at boot, then started.
#
# Delegates to $(OLED_DIR)/install.sh rather than reimplementing it here. That
# script is the single source of truth and does things a Makefile recipe would
# do badly: it import-checks as the user the service actually runs as (not as
# root, whose site-packages differ), adds that user to the i2c group, warns if
# /dev/i2c-N is absent, and runs systemd-analyze verify before enabling. It also
# never clobbers an existing /etc/default/bbb_oled.
#
# It cds to its own directory, so it works from here without one.
install-oled:
	@echo "Installing OLED status display ($(OLED_SERVICE))..."
	sudo $(OLED_DIR)/install.sh
	@echo "✅ $(OLED_SERVICE): installed to /usr/local/bin, enabled at boot, started"

# Uninstall
uninstall:
	sudo systemctl stop $(SERVICE) || true
	sudo rm -f /usr/local/bin/$(TARGET)
	@echo "Uninstalled $(TARGET)"

# Remove the OLED display. Leaves /etc/default/bbb_oled alone — that file holds
# your panel options, and install.sh will not overwrite it on a reinstall either.
uninstall-oled:
	sudo $(OLED_DIR)/install.sh --uninstall

# Test build (don't run, just compile)
test: all
	@echo "Build test passed ✓"

# Help
help:
	@echo "balance_bot Makefile"
	@echo ""
	@echo "Targets:"
	@echo "  make          - Build balance_bot"
	@echo "  make install  - Stop service, install to /usr/local/bin, restart service"
	@echo "                  (also installs the OLED display; non-fatal if it fails)"
	@echo "  make install-oled   - Install/enable the OLED status display only"
	@echo "  make uninstall-oled - Remove the OLED display (keeps /etc/default/bbb_oled)"
	@echo "  make clean    - Remove build artifacts"
	@echo "  make uninstall- Stop service and remove from system"
	@echo "  make test     - Test build only"
	@echo "  make help     - Show this help"
	@echo ""