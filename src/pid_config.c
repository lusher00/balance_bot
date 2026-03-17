/**
 * @file pid_config.c
 * @brief PID configuration file I/O
 * 
 * Loads and saves PID parameters to/from a text file.
 * Compatible with the original rc_balance pidconfig.txt format.
 * 
 * File format:
 * Line 1: holdPosition (0 or 1) - currently unused
 * Line 2: balance_angle (float) - balance angle offset
 * Line 3: D1_balance Kp Ki Kd
 * Line 4: D2_drive Kp Ki Kd
 * Line 5: D3_steering Kp Ki Kd
 * 
 * Example pidconfig.txt:
 * 0
 * 0.02
 * 40.0 0.0 5.0
 * 20.0 0.5 2.0
 * 15.0 0.0 1.5
 */

#include "balance_bot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define DEFAULT_CONFIG_FILE "pidconfig.txt"

/**
 * @brief Load PID configuration from file
 * 
 * @param filename Path to configuration file
 * @param config Pointer to structure to populate
 * @return 0 on success, -1 on error
 */
int pid_config_load(const char* filename, pid_config_file_t* config) {
    FILE* file;
    int hold_position;
    int fields_read;
    
    if (!filename) filename = DEFAULT_CONFIG_FILE;
    
    LOG_INFO("Loading PID config from %s", filename);
    
    file = fopen(filename, "r");
    if (!file) {
        LOG_ERROR("Failed to open %s: %s", filename, strerror(errno));
        return -1;
    }
    
    // Line 1: holdPosition (legacy, ignored)
    fields_read = fscanf(file, "%d", &hold_position);
    if (fields_read != 1) {
        LOG_ERROR("Failed to read holdPosition");
        fclose(file);
        return -1;
    }
    
    // Line 2: balance_angle
    fields_read = fscanf(file, "%f", &config->balance_angle);
    if (fields_read != 1) {
        LOG_ERROR("Failed to read balance_angle");
        fclose(file);
        return -1;
    }
    
    // Line 3: D1_balance (Kp Ki Kd)
    fields_read = fscanf(file, "%f %f %f", 
                        &config->D1_balance.kp,
                        &config->D1_balance.ki,
                        &config->D1_balance.kd);
    if (fields_read != 3) {
        LOG_ERROR("Failed to read D1_balance PID gains");
        fclose(file);
        return -1;
    }
    
    // Line 4: D2_drive (Kp Ki Kd)
    fields_read = fscanf(file, "%f %f %f",
                        &config->D2_drive.kp,
                        &config->D2_drive.ki,
                        &config->D2_drive.kd);
    if (fields_read != 3) {
        LOG_ERROR("Failed to read D2_drive PID gains");
        fclose(file);
        return -1;
    }
    
    // Line 5: D3_steering (Kp Ki Kd)
    fields_read = fscanf(file, "%f %f %f",
                        &config->D3_steering.kp,
                        &config->D3_steering.ki,
                        &config->D3_steering.kd);
    if (fields_read != 3) {
        LOG_ERROR("Failed to read D3_steering PID gains");
        fclose(file);
        return -1;
    }
    
    fclose(file);
    
    LOG_INFO("PID config loaded successfully");
    LOG_INFO("  Balance angle: %.3f", config->balance_angle);
    LOG_INFO("  D1_balance: Kp=%.1f Ki=%.1f Kd=%.1f", 
             config->D1_balance.kp, config->D1_balance.ki, config->D1_balance.kd);
    LOG_INFO("  D2_drive: Kp=%.1f Ki=%.1f Kd=%.1f",
             config->D2_drive.kp, config->D2_drive.ki, config->D2_drive.kd);
    LOG_INFO("  D3_steering: Kp=%.1f Ki=%.1f Kd=%.1f",
             config->D3_steering.kp, config->D3_steering.ki, config->D3_steering.kd);
    
    return 0;
}

/**
 * @brief Save PID configuration to file
 * 
 * @param filename Path to configuration file
 * @param config Pointer to configuration to save
 * @return 0 on success, -1 on error
 */
int pid_config_save(const char* filename, const pid_config_file_t* config) {
    FILE* file;
    
    if (!filename) filename = DEFAULT_CONFIG_FILE;
    
    LOG_INFO("Saving PID config to %s", filename);
    
    file = fopen(filename, "w");
    if (!file) {
        LOG_ERROR("Failed to open %s for writing: %s", filename, strerror(errno));
        return -1;
    }
    
    // Write file in same format as load expects
    fprintf(file, "0\n");  // holdPosition (legacy)
    fprintf(file, "%.3f\n", config->balance_angle);
    fprintf(file, "%.2f %.2f %.2f\n", 
            config->D1_balance.kp, config->D1_balance.ki, config->D1_balance.kd);
    fprintf(file, "%.2f %.2f %.2f\n",
            config->D2_drive.kp, config->D2_drive.ki, config->D2_drive.kd);
    fprintf(file, "%.2f %.2f %.2f\n",
            config->D3_steering.kp, config->D3_steering.ki, config->D3_steering.kd);
    
    fclose(file);
    
    LOG_INFO("PID config saved successfully");
    return 0;
}

/**
 * @brief Apply configuration to PID controllers
 * 
 * Updates the global PID controller structures with loaded configuration.
 * 
 * @param config Pointer to configuration
 */
void pid_config_apply(const pid_config_file_t* config) {
    LOG_INFO("Applying PID configuration");
    
    // Update balance PID
    pid_set_gains(&balance_pid, 
                  config->D1_balance.kp,
                  config->D1_balance.ki,
                  config->D1_balance.kd);
    
    // Update steering PID
    pid_set_gains(&steering_pid,
                  config->D3_steering.kp,
                  config->D3_steering.ki,
                  config->D3_steering.kd);
    
    // D2_drive not implemented yet
    // TODO: Apply D2 gains when drive controller is added
    
    // Apply balance angle offset as runtime trim
    state.theta_offset = config->balance_angle;
    LOG_INFO("Balance trim (theta_offset): %.3f deg", state.theta_offset);
    
    LOG_INFO("PID configuration applied");
}

/**
 * @brief Get current PID configuration from controllers
 * 
 * Reads current gains from PID controllers and populates config structure.
 * Useful for saving current tuned values to file.
 * 
 * @param config Pointer to structure to populate
 */
void pid_config_get_current(pid_config_file_t* config) {
    config->balance_angle = state.theta_offset;
    
    config->D1_balance.kp = balance_pid.kp;
    config->D1_balance.ki = balance_pid.ki;
    config->D1_balance.kd = balance_pid.kd;
    
    config->D2_drive.kp = 0.0;  // Not implemented yet
    config->D2_drive.ki = 0.0;
    config->D2_drive.kd = 0.0;
    
    config->D3_steering.kp = steering_pid.kp;
    config->D3_steering.ki = steering_pid.ki;
    config->D3_steering.kd = steering_pid.kd;
}

/**
 * @brief Create default configuration file
 * 
 * Creates a default pidconfig.txt with reasonable starting values.
 * 
 * @param filename Path to file to create
 * @return 0 on success, -1 on error
 */
int pid_config_create_default(const char* filename) {
    pid_config_file_t default_config = {
        .balance_angle = 0.0,
        .D1_balance = { .kp = BALANCE_KP, .ki = BALANCE_KI, .kd = BALANCE_KD },
        .D2_drive = { .kp = 0.0, .ki = 0.0, .kd = 0.0 },
        .D3_steering = { .kp = STEERING_KP, .ki = STEERING_KI, .kd = STEERING_KD }
    };
    
    if (!filename) filename = DEFAULT_CONFIG_FILE;
    
    LOG_INFO("Creating default config at %s", filename);
    
    return pid_config_save(filename, &default_config);
}

/**
 * @brief Load configuration or create default
 * 
 * Attempts to load configuration file. If it doesn't exist,
 * creates a default one.
 * 
 * @param filename Path to configuration file
 * @param config Pointer to structure to populate
 * @return 0 on success, -1 on error
 */
int pid_config_load_or_default(const char* filename, pid_config_file_t* config) {
    if (!filename) filename = DEFAULT_CONFIG_FILE;
    
    // Try to load existing file
    if (pid_config_load(filename, config) == 0) {
        return 0;
    }
    
    // File doesn't exist or is invalid - create default
    LOG_WARN("Config file not found, creating default");
    
    if (pid_config_create_default(filename) < 0) {
        LOG_ERROR("Failed to create default config");
        return -1;
    }
    
    // Load the newly created default
    return pid_config_load(filename, config);
}

/**
 * @brief Print configuration to console
 * 
 * @param config Configuration to print
 */
void pid_config_print(const pid_config_file_t* config) {
    printf("\n=== PID Configuration ===\n");
    printf("Balance trim (theta_offset): %.3f deg\n", config->balance_angle);
    printf("\nD1 Balance Controller:\n");
    printf("  Kp = %.2f\n", config->D1_balance.kp);
    printf("  Ki = %.2f\n", config->D1_balance.ki);
    printf("  Kd = %.2f\n", config->D1_balance.kd);
    printf("\nD2 Drive Controller:\n");
    printf("  Kp = %.2f\n", config->D2_drive.kp);
    printf("  Ki = %.2f\n", config->D2_drive.ki);
    printf("  Kd = %.2f\n", config->D2_drive.kd);
    printf("\nD3 Steering Controller:\n");
    printf("  Kp = %.2f\n", config->D3_steering.kp);
    printf("  Ki = %.2f\n", config->D3_steering.ki);
    printf("  Kd = %.2f\n", config->D3_steering.kd);
    printf("=========================\n\n");
}
