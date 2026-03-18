/**
 * @file ipc_server.c
 * @brief Unix domain socket server for iPhone app communication
 * 
 * This module provides a Unix socket server that allows the Node.js
 * web server to communicate with the balance_bot C application.
 * 
 * Communication is bidirectional:
 * - C app → Node.js: Telemetry data (JSON)
 * - Node.js → C app: Commands from iPhone (JSON)
 * 
 * Socket path: /tmp/balance_bot.sock
 */

#include "debug_config.h"
#include "balance_bot.h"
#include "motor_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <robotcontrol.h>

#define SOCKET_PATH "/tmp/balance_bot.sock"
#define MAX_CLIENTS 5
#define BUFFER_SIZE 4096

// Forward declarations
static void* client_handler_thread(void* arg);
static int handle_command(const char* json_cmd, char* response, size_t response_len);
static int parse_json_command(const char* json_cmd);
static void build_telemetry_json(char* buffer, size_t size);

// Client connection tracking
typedef struct {
    int socket_fd;
    bool active;
    pthread_t thread;
} client_connection_t;

static client_connection_t clients[MAX_CLIENTS] = {0};
static pthread_mutex_t clients_mutex = PTHREAD_MUTEX_INITIALIZER;

static int server_socket = -1;
static pthread_t server_thread;
static bool server_running = false;

/**
 * @brief Main server thread - accepts new connections
 */
static void* server_thread_func(void* arg __attribute__((unused))) {
    int client_fd;
    
    LOG_INFO("IPC server thread started");
    
    while (server_running) {
        // Accept new connection
        client_fd = accept(server_socket, NULL, NULL);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            LOG_ERROR("Accept failed: %s", strerror(errno));
            break;
        }
        
        LOG_INFO("New client connected (fd=%d)", client_fd);
        
        // Find free slot for client
        pthread_mutex_lock(&clients_mutex);
        bool added = false;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i].active) {
                clients[i].socket_fd = client_fd;
                clients[i].active = true;
                
                // Spawn handler thread
                if (pthread_create(&clients[i].thread, NULL, 
                                   client_handler_thread, &clients[i]) != 0) {
                    LOG_ERROR("Failed to create client handler thread");
                    close(client_fd);
                    clients[i].active = false;
                } else {
                    pthread_detach(clients[i].thread);
                    added = true;
                }
                break;
            }
        }
        pthread_mutex_unlock(&clients_mutex);
        
        if (!added) {
            LOG_WARN("Max clients reached, rejecting connection");
            close(client_fd);
        }
    }
    
    LOG_INFO("IPC server thread stopped");
    return NULL;
}

/**
 * @brief Client handler thread - reads commands from one client
 */
static void* client_handler_thread(void* arg) {
    client_connection_t* client = (client_connection_t*)arg;
    char buffer[BUFFER_SIZE];
    char response[BUFFER_SIZE];
    int bytes_read;
    
    LOG_DEBUG("Client handler started for fd=%d", client->socket_fd);
    
    while (client->active) {
        bytes_read = read(client->socket_fd, buffer, sizeof(buffer) - 1);
        
        if (bytes_read <= 0) {
            if (bytes_read < 0) {
                LOG_ERROR("Read error on fd=%d: %s", client->socket_fd, strerror(errno));
            }
            break;
        }
        
        buffer[bytes_read] = '\0';
        LOG_DEBUG("Received command: %s", buffer);
        
        // Process command
        if (handle_command(buffer, response, sizeof(response)) == 0) {
            // Send response
            ssize_t written = write(client->socket_fd, response, strlen(response));
            (void)written; // Suppress unused warning
        }
    }
    
    LOG_INFO("Client disconnected (fd=%d)", client->socket_fd);
    close(client->socket_fd);
    
    pthread_mutex_lock(&clients_mutex);
    client->active = false;
    pthread_mutex_unlock(&clients_mutex);
    
    return NULL;
}

/**
 * @brief Handle incoming JSON command from iPhone
 * 
 * @param json_cmd Null-terminated JSON command string
 * @param response Buffer to write JSON response
 * @param response_len Size of response buffer
 * @return 0 on success, -1 on error
 */
static int handle_command(const char* json_cmd, char* response, size_t response_len) {
    // Parse and execute command
    if (parse_json_command(json_cmd) < 0) {
        snprintf(response, response_len, "{\"status\":\"error\",\"message\":\"Invalid command\"}\n");
        return -1;
    }
    
    snprintf(response, response_len, "{\"status\":\"ok\"}\n");
    return 0;
}

/**
 * @brief Parse JSON command and update configuration
 * 
 * Supported commands:
 * - {"type":"set_controller","controller":"D1_balance","enabled":true}
 * - {"type":"set_pid","controller":"D1_balance","kp":40.0,"ki":0.0,"kd":5.0}
 * - {"type":"set_telemetry","encoders":true,"imu_full":false,...}
 * - {"type":"arm","value":true}
 * - {"type":"set_mode","value":1}
 * 
 * @param json_cmd JSON command string
 * @return 0 on success, -1 on error
 */
static int parse_json_command(const char* json_cmd) {
    // Simple JSON parsing (in production, use json-c library)
    // For now, we'\''ll do basic string matching
    
    // Example: {"type":"set_controller","controller":"D1_balance","enabled":true}
    if (strstr(json_cmd, "\"type\":\"set_controller\"")) {
        if (strstr(json_cmd, "\"controller\":\"D1_balance\"")) {
            if (strstr(json_cmd, "\"enabled\":true")) {
                g_debug_config.controllers.D1_balance = true;
                LOG_INFO("D1_balance enabled");
            } else if (strstr(json_cmd, "\"enabled\":false")) {
                g_debug_config.controllers.D1_balance = false;
                LOG_WARN("D1_balance disabled - robot will fall!");
            }
        }
        else if (strstr(json_cmd, "\"controller\":\"D2_drive\"")) {
            if (strstr(json_cmd, "\"enabled\":true")) {
                g_debug_config.controllers.D2_drive = true;
                LOG_INFO("D2_drive enabled");
            } else {
                g_debug_config.controllers.D2_drive = false;
                LOG_INFO("D2_drive disabled");
            }
        }
        else if (strstr(json_cmd, "\"controller\":\"D3_steering\"")) {
            if (strstr(json_cmd, "\"enabled\":true")) {
                g_debug_config.controllers.D3_steering = true;
                LOG_INFO("D3_steering enabled");
            } else {
                g_debug_config.controllers.D3_steering = false;
                LOG_INFO("D3_steering disabled");
            }
        }
        return 0;
    }
    
    // {"type":"set_telemetry","encoders":true,...}
    if (strstr(json_cmd, "\"type\":\"set_telemetry\"")) {
        if (strstr(json_cmd, "\"encoders\":true")) {
            g_debug_config.telemetry.encoders = true;
        } else if (strstr(json_cmd, "\"encoders\":false")) {
            g_debug_config.telemetry.encoders = false;
        }
        
        if (strstr(json_cmd, "\"imu_full\":true")) {
            g_debug_config.telemetry.imu_full = true;
            LOG_WARN("imu_full enabled - HIGH BANDWIDTH");
        } else if (strstr(json_cmd, "\"imu_full\":false")) {
            g_debug_config.telemetry.imu_full = false;
        }
        
        if (strstr(json_cmd, "\"pid_states\":true")) {
            g_debug_config.telemetry.pid_states = true;
        } else if (strstr(json_cmd, "\"pid_states\":false")) {
            g_debug_config.telemetry.pid_states = false;
        }
        
        LOG_INFO("Telemetry configuration updated");
        return 0;
    }
    
    // {"type":"arm","value":true}  or  {"type":"arm","value":false}
    if (strstr(json_cmd, "\"type\":\"arm\"")) {
        if (strstr(json_cmd, "\"value\":true")) {
            if (fabsf(state.theta - state.theta_offset) > 14.0f) {
                LOG_WARN("iPhone ARM REJECTED — effective angle too large (%.1f deg)", state.theta - state.theta_offset);
                return -1;
            }
            state.armed = 1;
            rc_led_set(RC_LED_GREEN, 1);
            LOG_INFO("iPhone: ARMED (theta=%.2f deg)", state.theta);
        } else {
            state.armed = 0;
            rc_led_set(RC_LED_GREEN, 0);
            LOG_INFO("iPhone: DISARMED");
        }
        return 0;
    }

    // {"type":"zero_imu"}  — set balance trim to current pitch, save to pidconfig.txt
    if (strstr(json_cmd, "\"type\":\"zero_encoders\"")) {
        motor_hal_encoder_reset_all();
        state.enc_left  = 0;
        state.enc_right = 0;
        state.phi_left  = 0.0f;
        state.phi_right = 0.0f;
        LOG_INFO("iPhone: encoders zeroed");
        return 0;
    }

    if (strstr(json_cmd, "\"type\":\"zero_imu\"")) {
        // Capture current raw angle as the mounting offset — stored in radians
        // inside imu_config so apply_transform outputs 0 when upright.
        imu_offsets_calibrate(&mpu_data, &g_imu_offsets);
        imu_offsets_save(&g_imu_offsets);
        // Reset the PID balance trim — the mounting offset now handles zeroing.
        state.theta_offset = 0.0f;
        pid_config_file_t cfg;
        pid_config_get_current(&cfg);
        pid_config_save(NULL, &cfg);
        LOG_INFO("iPhone: IMU zeroed, saved");
        return 0;
    }

    // {"type":"set_theta_offset","value":1.5}  — runtime balance trim, saved to pidconfig.txt
    if (strstr(json_cmd, "\"type\":\"set_theta_offset\"")) {
        const char *p = strstr(json_cmd, "\"value\":");
        if (!p) return -1;
        float val = 0.0f;
        if (sscanf(p, "\"value\":%f", &val) != 1) return -1;
        if (val >  30.0f) val =  30.0f;
        if (val < -30.0f) val = -30.0f;
        state.theta_offset = val;
        pid_config_file_t cfg;
        pid_config_get_current(&cfg);
        pid_config_save(NULL, &cfg);
        LOG_INFO("iPhone: theta_offset = %.2f deg, saved", val);
        return 0;
    }

    // {"type":"set_mode","value":1}  0=idle 1=balance 2=ext_input 3=manual
    if (strstr(json_cmd, "\"type\":\"set_mode\"")) {
        const char *p = strstr(json_cmd, "\"value\":");
        if (!p) return -1;
        int val = 0;
        if (sscanf(p, "\"value\":%d", &val) != 1) return -1;
        if (val < 0 || val > 3) return -1;
        state.mode = (robot_mode_t)val;
        LOG_INFO("iPhone: mode → %d", val);
        return 0;
    }

    // {"type":"set_pid","controller":"D1_balance","kp":40.0,"ki":0.5,"kd":5.0}
    // {"type":"set_pid","controller":"D3_steering","kp":1.0,"ki":0.0,"kd":0.1}
    if (strstr(json_cmd, "\"type\":\"set_pid\"")) {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;
        const char *p;

        p = strstr(json_cmd, "\"kp\":"); if (p) sscanf(p, "\"kp\":%f", &kp);
        p = strstr(json_cmd, "\"ki\":"); if (p) sscanf(p, "\"ki\":%f", &ki);
        p = strstr(json_cmd, "\"kd\":"); if (p) sscanf(p, "\"kd\":%f", &kd);

        if (strstr(json_cmd, "\"controller\":\"D1_balance\"")) {
            pid_set_gains(&balance_pid, kp, ki, kd);
            pid_reset(&balance_pid);
            LOG_INFO("iPhone: D1_balance kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);
            return 0;
        }
        if (strstr(json_cmd, "\"controller\":\"D3_steering\"")) {
            pid_set_gains(&steering_pid, kp, ki, kd);
            pid_reset(&steering_pid);
            LOG_INFO("iPhone: D3_steering kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);
            return 0;
        }
        LOG_WARN("set_pid: unknown controller in: %s", json_cmd);
        return -1;
    }

    // {"type":"save_pid"}  — write current gains to pidconfig.txt
    if (strstr(json_cmd, "\"type\":\"save_pid\"")) {
        pid_config_file_t cfg;
        pid_config_get_current(&cfg);
        if (pid_config_save(NULL, &cfg) == 0) {
            LOG_INFO("iPhone: PID config saved to pidconfig.txt");
        } else {
            LOG_WARN("iPhone: failed to save PID config");
        }
        return 0;
    }

    LOG_WARN("Unknown command type");
    return -1;
}

/**
 * @brief Build JSON telemetry packet
 * 
 * Constructs a JSON string containing all enabled telemetry data.
 * Only includes fields that are enabled in g_debug_config.
 * 
 * @param buffer Output buffer for JSON string
 * @param size Size of output buffer
 */
static void build_telemetry_json(char* buffer, size_t size) {
    size_t pos = 0;
    
    pos += snprintf(buffer + pos, size - pos, "{");
    pos += snprintf(buffer + pos, size - pos, "\"type\":\"telemetry\",");
    pos += snprintf(buffer + pos, size - pos, "\"timestamp\":%llu,", 
                   (unsigned long long)(rc_nanos_since_boot() / 1000));
    
    // System status (always included)
    if (g_debug_config.telemetry.system_status) {
        pos += snprintf(buffer + pos, size - pos,
                       "\"system\":{\"battery\":%.2f,\"armed\":%s,\"mode\":%d,\"loop_hz\":%.1f,\"theta_offset\":%.4f,"
                       "\"batt_voltage\":%.3f,\"batt_status\":%d},",
                       g_telemetry_data.system.battery_voltage,
                       g_telemetry_data.system.armed ? "true" : "false",
                       g_telemetry_data.system.mode,
                       g_telemetry_data.system.loop_hz,
                       state.theta_offset,
                       g_telemetry_data.system.batt_voltage,
                       (int)g_telemetry_data.system.batt_status);
    }
    
    // Encoders
    if (g_debug_config.telemetry.encoders) {
        pos += snprintf(buffer + pos, size - pos,
                       "\"encoders\":{\"left_ticks\":%d,\"right_ticks\":%d,"
                       "\"left_rad\":%.3f,\"right_rad\":%.3f,"
                       "\"left_vel\":%.3f,\"right_vel\":%.3f},",
                       g_telemetry_data.encoders.left_ticks,
                       g_telemetry_data.encoders.right_ticks,
                       g_telemetry_data.encoders.left_rad,
                       g_telemetry_data.encoders.right_rad,
                       g_telemetry_data.encoders.left_vel,
                       g_telemetry_data.encoders.right_vel);
    }
    
    // IMU attitude (for 3D visualization)
    if (g_debug_config.telemetry.imu_attitude) {
        pos += snprintf(buffer + pos, size - pos,
                       "\"imu\":{\"theta\":%.4f,\"phi\":%.4f,\"psi\":%.4f,"
                       "\"theta_dot\":%.4f,\"phi_dot\":%.4f,\"psi_dot\":%.4f,"
                       "\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f",
                       g_telemetry_data.imu.theta,
                       g_telemetry_data.imu.phi,
                       g_telemetry_data.imu.psi,
                       g_telemetry_data.imu.theta_dot,
                       g_telemetry_data.imu.phi_dot,
                       g_telemetry_data.imu.psi_dot,
                       g_telemetry_data.imu.qw,
                       g_telemetry_data.imu.qx,
                       g_telemetry_data.imu.qy,
                       g_telemetry_data.imu.qz);
        
        // Add full IMU data if enabled
        if (g_debug_config.telemetry.imu_full) {
            pos += snprintf(buffer + pos, size - pos,
                           ",\"accel_x\":%.3f,\"accel_y\":%.3f,\"accel_z\":%.3f,"
                           "\"gyro_x\":%.3f,\"gyro_y\":%.3f,\"gyro_z\":%.3f",
                           g_telemetry_data.imu.accel_x,
                           g_telemetry_data.imu.accel_y,
                           g_telemetry_data.imu.accel_z,
                           g_telemetry_data.imu.gyro_x,
                           g_telemetry_data.imu.gyro_y,
                           g_telemetry_data.imu.gyro_z);
        }
        
        pos += snprintf(buffer + pos, size - pos, "},");
    }
    
    // PID states
    if (g_debug_config.telemetry.pid_states) {
        pos += snprintf(buffer + pos, size - pos,
                       "\"D1_balance\":{\"enabled\":%s,\"setpoint\":%.4f,"
                       "\"measurement\":%.4f,\"error\":%.4f,\"output\":%.4f,"
                       "\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},",
                       g_telemetry_data.D1_balance.enabled ? "true" : "false",
                       g_telemetry_data.D1_balance.setpoint,
                       g_telemetry_data.D1_balance.measurement,
                       g_telemetry_data.D1_balance.error,
                       g_telemetry_data.D1_balance.output,
                       g_telemetry_data.D1_balance.kp,
                       g_telemetry_data.D1_balance.ki,
                       g_telemetry_data.D1_balance.kd);

        pos += snprintf(buffer + pos, size - pos,
                       "\"D3_steering\":{\"enabled\":%s,\"setpoint\":%.4f,"
                       "\"measurement\":%.4f,\"error\":%.4f,\"output\":%.4f,"
                       "\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f},",
                       g_telemetry_data.D3_steering.enabled ? "true" : "false",
                       g_telemetry_data.D3_steering.setpoint,
                       g_telemetry_data.D3_steering.measurement,
                       g_telemetry_data.D3_steering.error,
                       g_telemetry_data.D3_steering.output,
                       g_telemetry_data.D3_steering.kp,
                       g_telemetry_data.D3_steering.ki,
                       g_telemetry_data.D3_steering.kd);
    }
    
    // Cat position
    if (g_debug_config.telemetry.ext_input && g_telemetry_data.ext_input.valid) {
        pos += snprintf(buffer + pos, size - pos,
                       "\"ext_input\":{\"valid\":true,\"x\":%.3f,\"y\":%.3f,\"confidence\":%.2f},",
                       g_telemetry_data.ext_input.x,
                       g_telemetry_data.ext_input.y,
                       g_telemetry_data.ext_input.confidence);
    }
    
    // Remove trailing comma
    if (buffer[pos - 1] == ',') pos--;
    
    pos += snprintf(buffer + pos, size - pos, "}\n");
}

/**
 * @brief Broadcast telemetry to all connected clients
 * 
 * Call this periodically from the main control loop to send
 * telemetry data to connected iPhone apps.
 */
void ipc_broadcast_telemetry(void) {
    char buffer[BUFFER_SIZE];
    
    // Build JSON telemetry
    build_telemetry_json(buffer, sizeof(buffer));
    
    // Send to all active clients
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].active) {
            ssize_t written = write(clients[i].socket_fd, buffer, strlen(buffer));
            if (written < 0) {
                LOG_ERROR("Failed to send telemetry to client %d", i);
            }
        }
    }
    pthread_mutex_unlock(&clients_mutex);
}

/**
 * @brief Initialize IPC server
 * 
 * Creates Unix domain socket and starts server thread.
 * 
 * @return 0 on success, -1 on error
 */
int ipc_server_init(void) {
    struct sockaddr_un addr;
    
    LOG_INFO("Initializing IPC server at %s", SOCKET_PATH);
    
    // Remove old socket file if it exists
    unlink(SOCKET_PATH);
    
    // Create socket
    server_socket = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_socket < 0) {
        LOG_ERROR("Failed to create socket: %s", strerror(errno));
        return -1;
    }
    
    // Bind socket
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
    
    if (bind(server_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_ERROR("Failed to bind socket: %s", strerror(errno));
        close(server_socket);
        return -1;
    }

    // Allow any user (e.g. Node.js running as debian) to connect
    chmod(SOCKET_PATH, 0777);
    
    // Listen
    if (listen(server_socket, 5) < 0) {
        LOG_ERROR("Failed to listen: %s", strerror(errno));
        close(server_socket);
        return -1;
    }
    
    // Start server thread
    server_running = true;
    if (pthread_create(&server_thread, NULL, server_thread_func, NULL) != 0) {
        LOG_ERROR("Failed to create server thread");
        close(server_socket);
        return -1;
    }
    
    pthread_detach(server_thread);
    
    LOG_INFO("IPC server started successfully");
    return 0;
}

/**
 * @brief Cleanup IPC server
 * 
 * Closes all client connections and server socket.
 */
void ipc_server_cleanup(void) {
    LOG_INFO("Shutting down IPC server");
    
    server_running = false;
    
    // Close server socket (will interrupt accept())
    if (server_socket >= 0) {
        close(server_socket);
        server_socket = -1;
    }
    
    // Close all client connections
    pthread_mutex_lock(&clients_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].active) {
            close(clients[i].socket_fd);
            clients[i].active = false;
        }
    }
    pthread_mutex_unlock(&clients_mutex);
    
    // Remove socket file
    unlink(SOCKET_PATH);
    
    LOG_INFO("IPC server stopped");
}

