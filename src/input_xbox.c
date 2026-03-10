/**
 * @file input_xbox.c
 * @brief Xbox controller input with hot-plug support
 * 
 * Automatically detects and connects to Xbox controllers.
 * No device path required - scans /dev/input/js* for controllers.
 */

#include "cat_follower.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <dirent.h>
#include <errno.h>
#include <sys/ioctl.h>

#define DEAD_ZONE 3000
#define MAX_AXIS_VALUE 32767.0f

// Controller state
static struct {
    int fd;
    bool connected;
    bool enabled;
    
    // Axes
    float drive;
    float turn;
    
    // Buttons
    bool arm_button_pressed;
    bool mode_button_pressed;
    bool start_button_pressed;
    
    // Previous button states for edge detection
    bool prev_arm;
    bool prev_mode;
    bool prev_start;
    
    char device_path[512];
} xbox_state = {
    .fd = -1,
    .connected = false,
    .enabled = true,
    .drive = 0.0f,
    .turn = 0.0f,
    .arm_button_pressed = false,
    .mode_button_pressed = false,
    .start_button_pressed = false,
    .prev_arm = false,
    .prev_mode = false,
    .prev_start = false
};

/**
 * @brief Scan /dev/input for joystick devices
 * 
 * @param path Output buffer for device path
 * @return 0 if found, -1 if not found
 */
static int find_joystick(char* path, size_t path_len) {
    DIR* dir = opendir("/dev/input");
    if (!dir) {
        return -1;
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "js", 2) == 0) {
            snprintf(path, path_len, "/dev/input/%.244s", entry->d_name);
            closedir(dir);
            return 0;
        }
    }
    
    closedir(dir);
    return -1;
}

/**
 * @brief Try to connect to controller
 * 
 * @return 0 on success, -1 on failure
 */
static int xbox_connect(void) {
    char device_path[512];
    
    if (find_joystick(device_path, sizeof(device_path)) < 0) {
        return -1;
    }
    
    int fd = open(device_path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }
    
    // Get device name
    char name[128];
    if (ioctl(fd, JSIOCGNAME(sizeof(name)), name) < 0) {
        close(fd);
        return -1;
    }
    
    xbox_state.fd = fd;
    xbox_state.connected = true;
    strncpy(xbox_state.device_path, device_path, sizeof(xbox_state.device_path) - 1);
    xbox_state.device_path[sizeof(xbox_state.device_path) - 1] = '\0';
    
    LOG_INFO("Xbox controller connected: %s (%s)", device_path, name);
    
    return 0;
}

/**
 * @brief Disconnect from controller
 */
static void xbox_disconnect(void) {
    if (xbox_state.fd >= 0) {
        close(xbox_state.fd);
        xbox_state.fd = -1;
    }
    
    xbox_state.connected = false;
    xbox_state.drive = 0.0f;
    xbox_state.turn = 0.0f;
    
    LOG_INFO("Xbox controller disconnected");
}

/**
 * @brief Initialize Xbox controller system
 * 
 * No longer requires device path - will auto-detect
 * 
 * @param device Ignored (kept for compatibility)
 * @return Always returns 0
 */
int xbox_init(const char* device __attribute__((unused))) {
    LOG_INFO("Xbox controller system initialized (hot-plug mode)");
    
    // Try to connect immediately
    xbox_connect();
    
    return 0;
}

/**
 * @brief Update controller state
 * 
 * Reads joystick events and updates state.
 * Automatically reconnects if disconnected.
 * 
 * @return 0 on success, -1 on error
 */
int xbox_update(void) {
    // If not enabled, return immediately
    if (!xbox_state.enabled) {
        return 0;
    }
    
    // Try to connect if not connected
    if (!xbox_state.connected) {
        static uint64_t last_connect_attempt = 0;
        uint64_t now = rc_nanos_since_boot() / 1000000;
        
        // Try reconnect every 2 seconds
        if (now - last_connect_attempt > 2000) {
            xbox_connect();
            last_connect_attempt = now;
        }
        
        return -1;
    }
    
    struct js_event event;
    
    // Read all pending events
    while (read(xbox_state.fd, &event, sizeof(event)) == sizeof(event)) {
        // Ignore init events
        if (event.type & JS_EVENT_INIT) {
            continue;
        }
        
        if (event.type == JS_EVENT_AXIS) {
            switch (event.number) {
                case 0:  // Left stick X (turn)
                    if (abs(event.value) < DEAD_ZONE) {
                        xbox_state.turn = 0.0f;
                    } else {
                        xbox_state.turn = event.value / MAX_AXIS_VALUE;
                    }
                    break;
                    
                case 1:  // Left stick Y (drive)
                    if (abs(event.value) < DEAD_ZONE) {
                        xbox_state.drive = 0.0f;
                    } else {
                        xbox_state.drive = -event.value / MAX_AXIS_VALUE;  // Invert Y
                    }
                    break;
            }
        }
        else if (event.type == JS_EVENT_BUTTON) {
            bool pressed = event.value;
            
            switch (event.number) {
                case 0:  // A button (arm toggle)
                    xbox_state.arm_button_pressed = pressed && !xbox_state.prev_arm;
                    xbox_state.prev_arm = pressed;
                    break;
                    
                case 1:  // B button (mode toggle)
                    xbox_state.mode_button_pressed = pressed && !xbox_state.prev_mode;
                    xbox_state.prev_mode = pressed;
                    break;
                    
                case 7:  // Start button (exit)
                    xbox_state.start_button_pressed = pressed && !xbox_state.prev_start;
                    xbox_state.prev_start = pressed;
                    break;
            }
        }
    }
    
    // Check for disconnect
    if (errno == ENODEV) {
        xbox_disconnect();
        return -1;
    }
    
    return 0;
}

/**
 * @brief Get drive command from left stick Y
 * 
 * @return Drive value (-1.0 to 1.0)
 */
float xbox_get_drive(void) {
    return xbox_state.enabled && xbox_state.connected ? xbox_state.drive : 0.0f;
}

/**
 * @brief Get turn command from left stick X
 * 
 * @return Turn value (-1.0 to 1.0)
 */
float xbox_get_turn(void) {
    return xbox_state.enabled && xbox_state.connected ? xbox_state.turn : 0.0f;
}

/**
 * @brief Check if A button was pressed (edge trigger)
 * 
 * @return 1 if pressed this frame, 0 otherwise
 */
int xbox_get_arm_button(void) {
    if (!xbox_state.enabled || !xbox_state.connected) {
        return 0;
    }
    
    bool pressed = xbox_state.arm_button_pressed;
    xbox_state.arm_button_pressed = false;  // Clear edge
    return pressed ? 1 : 0;
}

/**
 * @brief Check if B button was pressed (edge trigger)
 * 
 * @return 1 if pressed this frame, 0 otherwise
 */
int xbox_get_mode_button(void) {
    if (!xbox_state.enabled || !xbox_state.connected) {
        return 0;
    }
    
    bool pressed = xbox_state.mode_button_pressed;
    xbox_state.mode_button_pressed = false;  // Clear edge
    return pressed ? 1 : 0;
}

/**
 * @brief Check if Start button was pressed (edge trigger)
 * 
 * @return 1 if pressed this frame, 0 otherwise
 */
int xbox_get_start_button(void) {
    if (!xbox_state.enabled || !xbox_state.connected) {
        return 0;
    }
    
    bool pressed = xbox_state.start_button_pressed;
    xbox_state.start_button_pressed = false;  // Clear edge
    return pressed ? 1 : 0;
}

/**
 * @brief Enable/disable Xbox controller input
 * 
 * @param enabled true to enable, false to disable
 */
void xbox_set_enabled(bool enabled) {
    xbox_state.enabled = enabled;
    
    if (!enabled) {
        xbox_state.drive = 0.0f;
        xbox_state.turn = 0.0f;
    }
    
    LOG_INFO("Xbox controller %s", enabled ? "enabled" : "disabled");
}

/**
 * @brief Check if Xbox controller is connected
 * 
 * @return true if connected, false otherwise
 */
bool xbox_is_connected(void) {
    return xbox_state.connected;
}

/**
 * @brief Check if Xbox controller is enabled
 * 
 * @return true if enabled, false otherwise
 */
bool xbox_is_enabled(void) {
    return xbox_state.enabled;
}

/**
 * @brief Cleanup Xbox controller
 */
void xbox_cleanup(void) {
    xbox_disconnect();
    LOG_INFO("Xbox controller system cleaned up");
}
