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
int pid_config_load(const char *filename, pid_config_file_t *config)
{
    FILE *file;
    int hold_position;
    int fields_read;

    if (!filename)
        filename = DEFAULT_CONFIG_FILE;

    LOG_INFO("Loading PID config from %s", filename);

    file = fopen(filename, "r");
    if (!file)
    {
        LOG_ERROR("Failed to open %s: %s", filename, strerror(errno));
        return -1;
    }

    // Line 1: holdPosition (legacy, ignored)
    fields_read = fscanf(file, "%d", &hold_position);
    if (fields_read != 1)
    {
        LOG_ERROR("Failed to read holdPosition");
        fclose(file);
        return -1;
    }

    // Line 2: balance_angle
    fields_read = fscanf(file, "%f", &config->balance_angle);
    if (fields_read != 1)
    {
        LOG_ERROR("Failed to read balance_angle");
        fclose(file);
        return -1;
    }

    // Line 3: D1_balance (Kp Ki Kd)
    fields_read = fscanf(file, "%f %f %f",
                         &config->D1_balance.kp,
                         &config->D1_balance.ki,
                         &config->D1_balance.kd);
    if (fields_read != 3)
    {
        LOG_ERROR("Failed to read D1_balance PID gains");
        fclose(file);
        return -1;
    }

    // Line 4: D2_drive (Kp Ki Kd)
    fields_read = fscanf(file, "%f %f %f",
                         &config->D2_drive.kp,
                         &config->D2_drive.ki,
                         &config->D2_drive.kd);
    if (fields_read != 3)
    {
        LOG_ERROR("Failed to read D2_drive PID gains");
        fclose(file);
        return -1;
    }

    // Line 5: D3_steering (Kp Ki Kd)
    fields_read = fscanf(file, "%f %f %f",
                         &config->D3_steering.kp,
                         &config->D3_steering.ki,
                         &config->D3_steering.kd);
    if (fields_read != 3)
    {
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
int pid_config_save(const char *filename, const pid_config_file_t *config)
{
    FILE *file;

    if (!filename)
        filename = DEFAULT_CONFIG_FILE;

    LOG_INFO("Saving PID config to %s", filename);

    file = fopen(filename, "w");
    if (!file)
    {
        LOG_ERROR("Failed to open %s for writing: %s", filename, strerror(errno));
        return -1;
    }

    // Write file in same format as load expects
    fprintf(file, "0\n"); // holdPosition (legacy)
    fprintf(file, "%.3f\n", config->balance_angle);
    fprintf(file, "%.3f %.3f %.3f\n",
            config->D1_balance.kp, config->D1_balance.ki, config->D1_balance.kd);
    fprintf(file, "%.3f %.3f %.3f\n",
            config->D2_drive.kp, config->D2_drive.ki, config->D2_drive.kd);
    fprintf(file, "%.3f %.3f %.3f\n",
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
void pid_config_apply(const pid_config_file_t *config)
{
    LOG_INFO("Applying PID configuration");

    // Update balance PID
    pid_set_gains(&balance_pid,
                  config->D1_balance.kp,
                  config->D1_balance.ki,
                  config->D1_balance.kd);

    // Update drive PID
    pid_set_gains(&drive_pid,
                  config->D2_drive.kp,
                  config->D2_drive.ki,
                  config->D2_drive.kd);

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
void pid_config_get_current(pid_config_file_t *config)
{
    config->balance_angle = state.theta_offset;

    config->D1_balance.kp = balance_pid.kp;
    config->D1_balance.ki = balance_pid.ki;
    config->D1_balance.kd = balance_pid.kd;

    config->D2_drive.kp = drive_pid.kp;
    config->D2_drive.ki = drive_pid.ki;
    config->D2_drive.kd = drive_pid.kd;

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
int pid_config_create_default(const char *filename)
{
    pid_config_file_t default_config = {
        .balance_angle = 0.0,
        .D1_balance = {.kp = BALANCE_KP, .ki = BALANCE_KI, .kd = BALANCE_KD},
        .D2_drive = {.kp = DRIVE_KP, .ki = DRIVE_KI, .kd = DRIVE_KD},
        .D3_steering = {.kp = STEERING_KP, .ki = STEERING_KI, .kd = STEERING_KD}};

    if (!filename)
        filename = DEFAULT_CONFIG_FILE;

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
int pid_config_load_or_default(const char *filename, pid_config_file_t *config)
{
    if (!filename)
        filename = DEFAULT_CONFIG_FILE;

    // Try to load existing file
    if (pid_config_load(filename, config) == 0)
    {
        return 0;
    }

    // File doesn't exist or is invalid - create default
    LOG_WARN("Config file not found, creating default");

    if (pid_config_create_default(filename) < 0)
    {
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
void pid_config_print(const pid_config_file_t *config)
{
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

// ============================================================================
// POSITION CONTROLLER CONFIG
// ============================================================================

#define POS_CONFIG_SECTION "# pos_config"

void pos_config_apply(const pos_config_t *cfg)
{
    g_pos_config = *cfg;
    LOG_INFO("pos_config applied: zones=%d/%d/%d scales=%.0f/%.0f/%.0f/%.0f "
             "vel=%.0f/%.0f/%.0f stopped=%d maxcorr=%.1f rate=%.1f bts=%d",
             cfg->zone_a, cfg->zone_b, cfg->zone_c,
             cfg->scale_a, cfg->scale_b, cfg->scale_c, cfg->scale_d,
             cfg->vel_scale_stop, cfg->vel_scale_move, cfg->vel_scale_turning,
             cfg->stopped_vel, cfg->max_correction,
             cfg->max_angle_rate, cfg->back_to_spot);
}

void pos_config_get_current(pos_config_t *cfg)
{
    *cfg = g_pos_config;
}

/**
 * Append (or overwrite the section starting at POS_CONFIG_SECTION) in the
 * pidconfig.txt file.  Uses a simple append — the section is always added at
 * the end, and load uses the last occurrence.
 *
 * Section format (one key=value per line, terminated by blank line or EOF):
 *   # pos_config
 *   zone_a=8000
 *   zone_b=4000
 *   ...
 */
int pos_config_save(const char *filename, const pos_config_t *cfg)
{
    if (!filename)
        filename = DEFAULT_CONFIG_FILE;

    FILE *f = fopen(filename, "a");
    if (!f)
    {
        LOG_ERROR("pos_config_save: cannot open %s: %s", filename, strerror(errno));
        return -1;
    }
    fprintf(f, "\n%s\n", POS_CONFIG_SECTION);
    fprintf(f, "zone_a=%.1f\n", (float)cfg->zone_a);
    fprintf(f, "zone_b=%.1f\n", (float)cfg->zone_b);
    fprintf(f, "zone_c=%.1f\n", (float)cfg->zone_c);
    fprintf(f, "scale_a=%.3f\n", cfg->scale_a);
    fprintf(f, "scale_b=%.3f\n", cfg->scale_b);
    fprintf(f, "scale_c=%.3f\n", cfg->scale_c);
    fprintf(f, "scale_d=%.3f\n", cfg->scale_d);
    fprintf(f, "vel_scale_stop=%.3f\n", cfg->vel_scale_stop);
    fprintf(f, "vel_scale_move=%.3f\n", cfg->vel_scale_move);
    fprintf(f, "vel_scale_turning=%.3f\n", cfg->vel_scale_turning);
    fprintf(f, "stopped_vel=%.1f\n", (float)cfg->stopped_vel);
    fprintf(f, "max_correction=%.3f\n", cfg->max_correction);
    fprintf(f, "max_angle_rate=%.3f\n", cfg->max_angle_rate);
    fprintf(f, "back_to_spot=%.1f\n", (float)cfg->back_to_spot);
    fclose(f);
    LOG_INFO("pos_config saved to %s", filename);
    return 0;
}

int pos_config_load_or_default(const char *filename, pos_config_t *cfg)
{
    // Fill defaults first — if the section is absent we still have sane values
    cfg->zone_a = POS_ZONE_A_DEFAULT;
    cfg->zone_b = POS_ZONE_B_DEFAULT;
    cfg->zone_c = POS_ZONE_C_DEFAULT;
    cfg->scale_a = POS_SCALE_A_DEFAULT;
    cfg->scale_b = POS_SCALE_B_DEFAULT;
    cfg->scale_c = POS_SCALE_C_DEFAULT;
    cfg->scale_d = POS_SCALE_D_DEFAULT;
    cfg->vel_scale_stop = POS_VEL_SCALE_STOP_DEFAULT;
    cfg->vel_scale_move = POS_VEL_SCALE_MOVE_DEFAULT;
    cfg->vel_scale_turning = POS_VEL_SCALE_TURNING_DEFAULT;
    cfg->stopped_vel = POS_STOPPED_VEL_DEFAULT;
    cfg->max_correction = POS_MAX_CORRECTION_DEFAULT;
    cfg->max_angle_rate = POS_MAX_ANGLE_RATE_DEFAULT;
    cfg->back_to_spot = POS_BACK_TO_SPOT_DEFAULT;

    if (!filename)
        filename = DEFAULT_CONFIG_FILE;
    FILE *f = fopen(filename, "r");
    if (!f)
        return 0; // no file yet — defaults are fine

    char line[128];
    int in_section = 0;
    while (fgets(line, sizeof(line), f))
    {
        if (strncmp(line, POS_CONFIG_SECTION, strlen(POS_CONFIG_SECTION)) == 0)
        {
            in_section = 1;
            continue;
        }
        if (!in_section)
            continue;
        if (line[0] == '#' || line[0] == '\n')
        {
            in_section = 0;
            continue;
        }
        // Parse key=value
        char key[64];
        float fval;
        int ival;
        if (sscanf(line, "%63[^=]=%f", key, &fval) == 2)
        {
            if (!strcmp(key, "zone_a"))
                cfg->zone_a = (int32_t)fval;
            else if (!strcmp(key, "zone_b"))
                cfg->zone_b = (int32_t)fval;
            else if (!strcmp(key, "zone_c"))
                cfg->zone_c = (int32_t)fval;
            else if (!strcmp(key, "scale_a"))
                cfg->scale_a = fval;
            else if (!strcmp(key, "scale_b"))
                cfg->scale_b = fval;
            else if (!strcmp(key, "scale_c"))
                cfg->scale_c = fval;
            else if (!strcmp(key, "scale_d"))
                cfg->scale_d = fval;
            else if (!strcmp(key, "vel_scale_stop"))
                cfg->vel_scale_stop = fval;
            else if (!strcmp(key, "vel_scale_move"))
                cfg->vel_scale_move = fval;
            else if (!strcmp(key, "vel_scale_turning"))
                cfg->vel_scale_turning = fval;
            else if (!strcmp(key, "stopped_vel"))
                cfg->stopped_vel = (int32_t)fval;
            else if (!strcmp(key, "max_correction"))
                cfg->max_correction = fval;
            else if (!strcmp(key, "max_angle_rate"))
                cfg->max_angle_rate = fval;
            else if (!strcmp(key, "back_to_spot"))
                cfg->back_to_spot = (int)fval;
        }
        (void)ival;
    }
    fclose(f);
    LOG_INFO("pos_config loaded from %s", filename);
    return 0;
}

// ============================================================================
// MOTOR CONFIG  (runtime RoboClaw drive mode + velocity tuning)
// ============================================================================

#define MOTOR_CONFIG_SECTION "# motor_config"

void motor_config_apply(const motor_config_t *cfg)
{
    g_motor_config = *cfg;
    LOG_INFO("motor_config applied: mode=%d qpps_max=%d accel_qpps=%d pol_l=%.1f pol_r=%.1f enc_pol_l=%.1f enc_pol_r=%.1f",
             cfg->mode, cfg->qpps_max, cfg->accel_qpps, cfg->pol_l, cfg->pol_r, cfg->enc_pol_l, cfg->enc_pol_r);
}

void motor_config_get_current(motor_config_t *cfg)
{
    *cfg = g_motor_config;
}

int motor_config_save(const char *filename, const motor_config_t *cfg)
{
    if (!filename)
        filename = DEFAULT_CONFIG_FILE;

    FILE *f = fopen(filename, "a");
    if (!f)
    {
        LOG_ERROR("motor_config_save: cannot open %s: %s", filename, strerror(errno));
        return -1;
    }
    fprintf(f, "\n%s\n", MOTOR_CONFIG_SECTION);
    fprintf(f, "mode=%d\n", cfg->mode);
    fprintf(f, "qpps_max=%d\n", cfg->qpps_max);
    fprintf(f, "accel_qpps=%d\n", cfg->accel_qpps);
    fprintf(f, "pol_l=%.1f\n", cfg->pol_l);
    fprintf(f, "pol_r=%.1f\n", cfg->pol_r);
    fprintf(f, "enc_pol_l=%.1f\n", cfg->enc_pol_l);
    fprintf(f, "enc_pol_r=%.1f\n", cfg->enc_pol_r);
    fclose(f);
    LOG_INFO("motor_config saved to %s", filename);
    return 0;
}

int motor_config_load_or_default(const char *filename, motor_config_t *cfg)
{
    /* Fill defaults first */
    cfg->mode = MOTOR_HAL_MODE_DEFAULT;
    cfg->qpps_max = MOTOR_QPPS_MAX_DEFAULT;
    cfg->accel_qpps = MOTOR_ACCEL_QPPS_DEFAULT;
    cfg->pol_l = 1.0f;
    cfg->pol_r = 1.0f;
    cfg->enc_pol_l = 1.0f;
    cfg->enc_pol_r = 1.0f;

    if (!filename)
        filename = DEFAULT_CONFIG_FILE;
    FILE *f = fopen(filename, "r");
    if (!f)
        return 0; /* no file yet — defaults are fine */

    char line[128];
    int in_section = 0;
    while (fgets(line, sizeof(line), f))
    {
        if (strncmp(line, MOTOR_CONFIG_SECTION, strlen(MOTOR_CONFIG_SECTION)) == 0)
        {
            in_section = 1;
            continue;
        }
        if (!in_section)
            continue;
        if (line[0] == '#' || line[0] == '\n')
        {
            in_section = 0;
            continue;
        }

        char key[64];
        float fval;
        if (sscanf(line, "%63[^=]=%f", key, &fval) == 2)
        {
            if (!strcmp(key, "mode"))
                cfg->mode = (int)fval;
            else if (!strcmp(key, "qpps_max"))
                cfg->qpps_max = (int)fval;
            else if (!strcmp(key, "accel_qpps"))
                cfg->accel_qpps = (int)fval;
            else if (!strcmp(key, "pol_l"))
                cfg->pol_l = fval;
            else if (!strcmp(key, "pol_r"))
                cfg->pol_r = fval;
            else if (!strcmp(key, "enc_pol_l"))
                cfg->enc_pol_l = fval;
            else if (!strcmp(key, "enc_pol_r"))
                cfg->enc_pol_r = fval;
        }
    }
    fclose(f);
    LOG_INFO("motor_config loaded from %s", filename);
    return 0;
}