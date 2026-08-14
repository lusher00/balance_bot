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
       src/pid_config.c \
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

.PHONY: all clean install uninstall test

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
$(OBJDIR)/%.o: src/%.c
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

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
install: $(BINDIR)/$(TARGET) install-units
	@echo "Stopping $(SERVICE) and $(SERVER_SERVICE)..."
	sudo systemctl daemon-reload
	sudo systemctl stop $(SERVICE) || true
	sudo systemctl stop $(SERVER_SERVICE) || true
	@echo "Installing $(TARGET) to /usr/local/bin/..."
	sudo cp $(BINDIR)/$(TARGET) /usr/local/bin/$(TARGET)
	@# No config seeding here. balance_bot writes robot.conf itself on first
	@# run, migrating from pidconfig.txt + /etc/balance_bot_imu.conf if present.
	@# The old block here wrote a positional file with gains that were not this
	@# robot's, which the migration would then have adopted.
	@echo "Restarting $(SERVICE) and $(SERVER_SERVICE)..."
	sudo systemctl start $(SERVICE) || true
	sudo systemctl start $(SERVER_SERVICE) || true
	@echo ""
	@echo "✅ Installed and restarted: /usr/local/bin/$(TARGET)"
	@echo ""

# Uninstall
uninstall:
	sudo systemctl stop $(SERVICE) || true
	sudo rm -f /usr/local/bin/$(TARGET)
	@echo "Uninstalled $(TARGET)"

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
	@echo "  make clean    - Remove build artifacts"
	@echo "  make uninstall- Stop service and remove from system"
	@echo "  make test     - Test build only"
	@echo "  make help     - Show this help"
	@echo ""