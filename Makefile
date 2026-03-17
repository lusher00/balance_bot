# balance_bot Makefile
# Self-balancing robot with iPhone app integration

TARGET = balance_bot
CC = gcc
CFLAGS = -Wall -Wextra -O2 -Iinclude
LDFLAGS = -lrobotcontrol -lm -lpthread -lncurses

# Source files
# Motor HAL — RoboClaw packet-serial over UART
MOTOR_HAL = src/motor_hal_roboclaw.c

SRCS = src/main.c \
       src/robot.c \
       src/display.c \
       src/pid.c \
       src/uart_input.c \
       src/roboclaw.c \
       src/ipc_server.c \
       src/telemetry.c \
       src/pid_config.c \
       src/input_xbox.c \
       src/input_sbus.c \
       src/imu_config.c \
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

# Install to system
install: $(BINDIR)/$(TARGET)
	@echo "Stopping $(SERVICE)..."
	sudo systemctl stop $(SERVICE) || true
	@echo "Installing $(TARGET) to /usr/local/bin/..."
	sudo cp $(BINDIR)/$(TARGET) /usr/local/bin/$(TARGET)
	@if [ ! -f pidconfig.txt ]; then \
		echo "0" > pidconfig.txt; \
		echo "0.02" >> pidconfig.txt; \
		echo "40.0 0.0 5.0" >> pidconfig.txt; \
		echo "20.0 0.5 2.0" >> pidconfig.txt; \
		echo "15.0 0.0 1.5" >> pidconfig.txt; \
		echo "Created default pidconfig.txt"; \
	fi
	@echo "Restarting $(SERVICE)..."
	sudo systemctl start $(SERVICE) || true
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
