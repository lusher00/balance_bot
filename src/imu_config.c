/**
 * @file imu_config.c
 * @brief IMU calibration and orientation configuration
 * 
 * Handles:
 * - IMU board mounting orientation transform
 * - Zero-point offset calibration
 * - Non-volatile storage in /etc/balance_bot_imu.conf
 */

#include "cat_follower.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define IMU_CONFIG_FILE "/etc/balance_bot_imu.conf"
#define IMU_CONFIG_BACKUP "/home/debian/balance_bot_imu.conf"

// Global IMU configuration
imu_config_t g_imu_config = {0};

/**
 * @brief Get default IMU configuration
 * 
 * IMU mounting:
 * - X-axis: DOWN (through axle), reads +9.81 m/s² at rest
 * - Y-axis: SIDEWAYS (left-right)
 * - Z-axis: FORWARD (direction of travel)
 * 
 * Robot angles:
 * - Pitch = rotation around Y (forward/back lean)
 * - Yaw = rotation around X (turning)
 * - Roll = rotation around Z (shouldn't happen)
 */
imu_config_t imu_config_get_default(void) {
    imu_config_t config = {0};
    
    // Orientation: IMU axes to robot axes
    // Robot pitch comes from IMU Y-axis rotation
    // Robot yaw comes from IMU X-axis rotation
    config.orientation.pitch_axis = IMU_AXIS_Y;
    config.orientation.yaw_axis = IMU_AXIS_X;
    config.orientation.roll_axis = IMU_AXIS_Z;
    
    // Sign corrections (determined by right-hand rule)
    config.orientation.pitch_sign = 1.0f;
    config.orientation.yaw_sign = 1.0f;
    config.orientation.roll_sign = 1.0f;
    
    // Zero offsets (calibrated when robot is balanced)
    config.offsets.pitch_offset = 0.0f;
    config.offsets.yaw_offset = 0.0f;
    config.offsets.roll_offset = 0.0f;
    
    // Gyro offsets (drift compensation)
    config.offsets.gyro_x_offset = 0.0f;
    config.offsets.gyro_y_offset = 0.0f;
    config.offsets.gyro_z_offset = 0.0f;
    
    config.calibrated = false;
    
    return config;
}

/**
 * @brief Load IMU configuration from file
 * 
 * @param config Pointer to config structure to fill
 * @return 0 on success, -1 on error
 */
int imu_config_load(imu_config_t* config) {
    FILE* fp = fopen(IMU_CONFIG_FILE, "r");
    if (!fp) {
        // Try backup location
        fp = fopen(IMU_CONFIG_BACKUP, "r");
        if (!fp) {
            LOG_WARN("IMU config file not found, using defaults");
            *config = imu_config_get_default();
            return -1;
        }
    }
    
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n') {
            continue;
        }
        
        char key[64];
        float value;
        
        if (sscanf(line, "%63s %f", key, &value) == 2) {
            if (strcmp(key, "pitch_offset") == 0) {
                config->offsets.pitch_offset = value;
            }
            else if (strcmp(key, "yaw_offset") == 0) {
                config->offsets.yaw_offset = value;
            }
            else if (strcmp(key, "roll_offset") == 0) {
                config->offsets.roll_offset = value;
            }
            else if (strcmp(key, "gyro_x_offset") == 0) {
                config->offsets.gyro_x_offset = value;
            }
            else if (strcmp(key, "gyro_y_offset") == 0) {
                config->offsets.gyro_y_offset = value;
            }
            else if (strcmp(key, "gyro_z_offset") == 0) {
                config->offsets.gyro_z_offset = value;
            }
            else if (strcmp(key, "pitch_sign") == 0) {
                config->orientation.pitch_sign = value;
            }
            else if (strcmp(key, "yaw_sign") == 0) {
                config->orientation.yaw_sign = value;
            }
            else if (strcmp(key, "roll_sign") == 0) {
                config->orientation.roll_sign = value;
            }
        }
    }
    
    fclose(fp);
    
    // Set orientation axes (fixed for this robot)
    config->orientation.pitch_axis = IMU_AXIS_Y;
    config->orientation.yaw_axis = IMU_AXIS_X;
    config->orientation.roll_axis = IMU_AXIS_Z;
    
    config->calibrated = true;
    
    LOG_INFO("IMU config loaded from %s", IMU_CONFIG_FILE);
    return 0;
}

/**
 * @brief Save IMU configuration to file
 * 
 * @param config Pointer to config structure
 * @return 0 on success, -1 on error
 */
int imu_config_save(const imu_config_t* config) {
    FILE* fp = fopen(IMU_CONFIG_FILE, "w");
    if (!fp) {
        // Try backup location
        fp = fopen(IMU_CONFIG_BACKUP, "w");
        if (!fp) {
            LOG_ERROR("Failed to save IMU config");
            return -1;
        }
    }
    
    fprintf(fp, "# Balance Bot IMU Configuration\n");
    fprintf(fp, "# Generated automatically - edit with caution\n\n");
    
    fprintf(fp, "# Angle offsets (radians)\n");
    fprintf(fp, "pitch_offset %.6f\n", config->offsets.pitch_offset);
    fprintf(fp, "yaw_offset %.6f\n", config->offsets.yaw_offset);
    fprintf(fp, "roll_offset %.6f\n", config->offsets.roll_offset);
    
    fprintf(fp, "\n# Gyro drift offsets (rad/s)\n");
    fprintf(fp, "gyro_x_offset %.6f\n", config->offsets.gyro_x_offset);
    fprintf(fp, "gyro_y_offset %.6f\n", config->offsets.gyro_y_offset);
    fprintf(fp, "gyro_z_offset %.6f\n", config->offsets.gyro_z_offset);
    
    fprintf(fp, "\n# Axis sign corrections\n");
    fprintf(fp, "pitch_sign %.0f\n", config->orientation.pitch_sign);
    fprintf(fp, "yaw_sign %.0f\n", config->orientation.yaw_sign);
    fprintf(fp, "roll_sign %.0f\n", config->orientation.roll_sign);
    
    fclose(fp);
    
    LOG_INFO("IMU config saved");
    return 0;
}

/**
 * @brief Apply IMU transform to raw DMP data
 * 
 * Transforms IMU coordinate system to robot coordinate system
 * and applies calibration offsets.
 * 
 * @param raw Raw IMU data from DMP
 * @param transformed Output transformed data
 * @param config IMU configuration
 */
void imu_config_apply_transform(const rc_mpu_data_t* raw, 
                                imu_transform_t* transformed,
                                const imu_config_t* config) {
    // Extract raw DMP Tait-Bryan angles
    float raw_angles[3] = {
        raw->dmp_TaitBryan[TB_PITCH_X],  // IMU X rotation
        raw->dmp_TaitBryan[TB_ROLL_Y],   // IMU Y rotation
        raw->dmp_TaitBryan[TB_YAW_Z]     // IMU Z rotation
    };
    
    // Map to robot axes
    // Robot pitch = IMU Y-axis rotation
    // Robot yaw = IMU X-axis rotation  
    // Robot roll = IMU Z-axis rotation
    transformed->pitch = raw_angles[config->orientation.pitch_axis] * 
                        config->orientation.pitch_sign - 
                        config->offsets.pitch_offset;
    
    transformed->yaw = raw_angles[config->orientation.yaw_axis] * 
                      config->orientation.yaw_sign - 
                      config->offsets.yaw_offset;
    
    transformed->roll = raw_angles[config->orientation.roll_axis] * 
                       config->orientation.roll_sign - 
                       config->offsets.roll_offset;
    
    // Angular rates (gyro data)
    transformed->pitch_dot = raw->gyro[config->orientation.pitch_axis] * DEG_TO_RAD * 
                             config->orientation.pitch_sign - 
                             config->offsets.gyro_y_offset;
    
    transformed->yaw_dot = raw->gyro[config->orientation.yaw_axis] * DEG_TO_RAD * 
                          config->orientation.yaw_sign - 
                          config->offsets.gyro_x_offset;
    
    transformed->roll_dot = raw->gyro[config->orientation.roll_axis] * DEG_TO_RAD * 
                           config->orientation.roll_sign - 
                           config->offsets.gyro_z_offset;
    
    // Raw accelerometer (for reference)
    transformed->accel_x = raw->accel[0];
    transformed->accel_y = raw->accel[1];
    transformed->accel_z = raw->accel[2];
}

/**
 * @brief Calibrate IMU offsets
 * 
 * Robot must be balanced and stationary.
 * Sets current angles as zero reference.
 * 
 * @param raw Current raw IMU data
 * @param config Configuration to update
 */
void imu_config_calibrate(const rc_mpu_data_t* raw, imu_config_t* config) {
    // Current angles become the offsets
    config->offsets.pitch_offset = raw->dmp_TaitBryan[config->orientation.pitch_axis] * 
                                   config->orientation.pitch_sign;
    
    config->offsets.yaw_offset = raw->dmp_TaitBryan[config->orientation.yaw_axis] * 
                                config->orientation.yaw_sign;
    
    config->offsets.roll_offset = raw->dmp_TaitBryan[config->orientation.roll_axis] * 
                                 config->orientation.roll_sign;
    
    // Gyro drift (average over time, not instant)
    config->offsets.gyro_x_offset = raw->gyro[0] * DEG_TO_RAD;
    config->offsets.gyro_y_offset = raw->gyro[1] * DEG_TO_RAD;
    config->offsets.gyro_z_offset = raw->gyro[2] * DEG_TO_RAD;
    
    config->calibrated = true;
    
    LOG_INFO("IMU calibrated: pitch_offset=%.3f, yaw_offset=%.3f", 
            config->offsets.pitch_offset, config->offsets.yaw_offset);
    
    // Save immediately
    imu_config_save(config);
}

/**
 * @brief Print IMU configuration
 */
void imu_config_print(const imu_config_t* config) {
    printf("\n=== IMU Configuration ===\n");
    printf("Orientation mapping:\n");
    printf("  Robot pitch ← IMU %c-axis rotation (sign: %.0f)\n", 
           'X' + config->orientation.pitch_axis, config->orientation.pitch_sign);
    printf("  Robot yaw   ← IMU %c-axis rotation (sign: %.0f)\n", 
           'X' + config->orientation.yaw_axis, config->orientation.yaw_sign);
    printf("  Robot roll  ← IMU %c-axis rotation (sign: %.0f)\n", 
           'X' + config->orientation.roll_axis, config->orientation.roll_sign);
    
    printf("\nAngle offsets:\n");
    printf("  Pitch: %.4f rad (%.2f°)\n", 
           config->offsets.pitch_offset, config->offsets.pitch_offset * RAD_TO_DEG);
    printf("  Yaw:   %.4f rad (%.2f°)\n", 
           config->offsets.yaw_offset, config->offsets.yaw_offset * RAD_TO_DEG);
    printf("  Roll:  %.4f rad (%.2f°)\n", 
           config->offsets.roll_offset, config->offsets.roll_offset * RAD_TO_DEG);
    
    printf("\nGyro offsets:\n");
    printf("  X: %.4f rad/s\n", config->offsets.gyro_x_offset);
    printf("  Y: %.4f rad/s\n", config->offsets.gyro_y_offset);
    printf("  Z: %.4f rad/s\n", config->offsets.gyro_z_offset);
    
    printf("\nCalibrated: %s\n", config->calibrated ? "YES" : "NO");
    printf("=========================\n\n");
}
