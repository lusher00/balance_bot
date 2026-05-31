/**
 * @file pid_config.c
 * @brief PID configuration file I/O
 *
 * All sections use key=value format.  Three sections in pidconfig.txt:
 *
 *   # pid_config
 *   d1_kp=0.070
 *   d1_ki=0.020
 *   d1_kd=0.005
 *   d3_kp=0.010
 *   d3_ki=0.005
 *   d3_kd=0.000
 *
 *   # pos_config
 *   zone_a=8000
 *   ...
 *
 *   # motor_config
 *   mode=0
 *   ...
 */

#include "balance_bot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define DEFAULT_CONFIG_FILE "pidconfig.txt"
#define PID_CONFIG_SECTION   "# pid_config"
#define POS_CONFIG_SECTION   "# pos_config"
#define MOTOR_CONFIG_SECTION "# motor_config"

// ============================================================================
// PID CONFIG
// ============================================================================

int pid_config_load(const char *filename, pid_config_file_t *config)
{
    if (!filename) filename = DEFAULT_CONFIG_FILE;

    LOG_INFO("Loading PID config from %s", filename);

    FILE *f = fopen(filename, "r");
    if (!f)
    {
        LOG_ERROR("Failed to open %s: %s", filename, strerror(errno));
        return -1;
    }

    char line[128];
    int in_section = 0;
    while (fgets(line, sizeof(line), f))
    {
        if (strncmp(line, PID_CONFIG_SECTION, strlen(PID_CONFIG_SECTION)) == 0)
        {
            in_section = 1;
            continue;
        }
        if (!in_section) continue;
        if (line[0] == '#' || line[0] == '\n') { in_section = 0; continue; }

        char key[64];
        float fval;
        if (sscanf(line, "%63[^=]=%f", key, &fval) == 2)
        {
            if      (!strcmp(key, "d1_kp")) config->D1_balance.kp = fval;
            else if (!strcmp(key, "d1_ki")) config->D1_balance.ki = fval;
            else if (!strcmp(key, "d1_kd")) config->D1_balance.kd = fval;
            else if (!strcmp(key, "d3_kp")) config->D3_steering.kp = fval;
            else if (!strcmp(key, "d3_ki")) config->D3_steering.ki = fval;
            else if (!strcmp(key, "d3_kd")) config->D3_steering.kd = fval;
        }
    }
    fclose(f);

    LOG_INFO("PID config loaded successfully");
    LOG_INFO("  D1_balance: Kp=%.3f Ki=%.3f Kd=%.3f",
             config->D1_balance.kp, config->D1_balance.ki, config->D1_balance.kd);
    LOG_INFO("  D3_steering: Kp=%.3f Ki=%.3f Kd=%.3f",
             config->D3_steering.kp, config->D3_steering.ki, config->D3_steering.kd);
    return 0;
}

int pid_config_save(const char *filename, const pid_config_file_t *config)
{
    if (!filename) filename = DEFAULT_CONFIG_FILE;

    LOG_INFO("Saving PID config to %s", filename);

    FILE *f = fopen(filename, "a");
    if (!f)
    {
        LOG_ERROR("Failed to open %s for writing: %s", filename, strerror(errno));
        return -1;
    }
    fprintf(f, "\n%s\n", PID_CONFIG_SECTION);
    fprintf(f, "d1_kp=%.6f\n", config->D1_balance.kp);
    fprintf(f, "d1_ki=%.6f\n", config->D1_balance.ki);
    fprintf(f, "d1_kd=%.6f\n", config->D1_balance.kd);
    fprintf(f, "d3_kp=%.6f\n", config->D3_steering.kp);
    fprintf(f, "d3_ki=%.6f\n", config->D3_steering.ki);
    fprintf(f, "d3_kd=%.6f\n", config->D3_steering.kd);
    fclose(f);

    LOG_INFO("PID config saved successfully");
    return 0;
}

void pid_config_apply(const pid_config_file_t *config)
{
    LOG_INFO("Applying PID configuration");

    pid_set_gains(&balance_pid,
                  config->D1_balance.kp,
                  config->D1_balance.ki,
                  config->D1_balance.kd);

    pid_set_gains(&steering_pid,
                  config->D3_steering.kp,
                  config->D3_steering.ki,
                  config->D3_steering.kd);

    LOG_INFO("PID configuration applied");
}

void pid_config_get_current(pid_config_file_t *config)
{
    config->D1_balance.kp = balance_pid.kp;
    config->D1_balance.ki = balance_pid.ki;
    config->D1_balance.kd = balance_pid.kd;
    config->D3_steering.kp = steering_pid.kp;
    config->D3_steering.ki = steering_pid.ki;
    config->D3_steering.kd = steering_pid.kd;
}

int pid_config_create_default(const char *filename)
{
    pid_config_file_t default_config = {
        .D1_balance  = { .kp = BALANCE_KP,  .ki = BALANCE_KI,  .kd = BALANCE_KD  },
        .D3_steering = { .kp = STEERING_KP, .ki = STEERING_KI, .kd = STEERING_KD },
    };
    if (!filename) filename = DEFAULT_CONFIG_FILE;
    LOG_INFO("Creating default config at %s", filename);
    return pid_config_save(filename, &default_config);
}

int pid_config_load_or_default(const char *filename, pid_config_file_t *config)
{
    // Set defaults first
    config->D1_balance.kp  = BALANCE_KP;
    config->D1_balance.ki  = BALANCE_KI;
    config->D1_balance.kd  = BALANCE_KD;
    config->D3_steering.kp = STEERING_KP;
    config->D3_steering.ki = STEERING_KI;
    config->D3_steering.kd = STEERING_KD;

    if (!filename) filename = DEFAULT_CONFIG_FILE;

    if (pid_config_load(filename, config) == 0)
        return 0;

    LOG_WARN("Config file not found, creating default");
    if (pid_config_create_default(filename) < 0)
    {
        LOG_ERROR("Failed to create default config");
        return -1;
    }
    return pid_config_load(filename, config);
}

void pid_config_print(const pid_config_file_t *config)
{
    printf("\n=== PID Configuration ===\n");
    printf("\nD1 Balance Controller:\n");
    printf("  Kp = %.2f\n", config->D1_balance.kp);
    printf("  Ki = %.2f\n", config->D1_balance.ki);
    printf("  Kd = %.2f\n", config->D1_balance.kd);
    printf("\nD2 Drive Controller:\n");
    printf("\nD3 Steering Controller:\n");
    printf("  Kp = %.2f\n", config->D3_steering.kp);
    printf("  Ki = %.2f\n", config->D3_steering.ki);
    printf("  Kd = %.2f\n", config->D3_steering.kd);
    printf("=========================\n\n");
}
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
    LOG_INFO("motor_config applied: mode=%d qpps_max=%d accel_qpps=%d pol_l=%.1f pol_r=%.1f "
             "enc_pol_l=%.1f enc_pol_r=%.1f claw_kp=%.4f claw_ki=%.4f claw_kd=%.4f",
             cfg->mode, cfg->qpps_max, cfg->accel_qpps, cfg->pol_l, cfg->pol_r,
             cfg->enc_pol_l, cfg->enc_pol_r, cfg->claw_kp, cfg->claw_ki, cfg->claw_kd);
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
    fprintf(f, "claw_kp=%.6f\n", cfg->claw_kp);
    fprintf(f, "claw_ki=%.6f\n", cfg->claw_ki);
    fprintf(f, "claw_kd=%.6f\n", cfg->claw_kd);
    fprintf(f, "baud=%d\n",      cfg->baud);
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
    cfg->pol_l = MOTOR_ENC_POL_L;
    cfg->pol_r = MOTOR_ENC_POL_R;
    cfg->enc_pol_l = MOTOR_MOT_POL_L;
    cfg->enc_pol_r = MOTOR_MOT_POL_R;
    cfg->claw_kp = MOTOR_CLAW_KP_DEFAULT;
    cfg->claw_ki = MOTOR_CLAW_KI_DEFAULT;
    cfg->claw_kd = MOTOR_CLAW_KD_DEFAULT;
    cfg->baud    = MOTOR_BAUD_DEFAULT;

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
            else if (!strcmp(key, "claw_kp"))
                cfg->claw_kp = fval;
            else if (!strcmp(key, "claw_ki"))
                cfg->claw_ki = fval;
            else if (!strcmp(key, "claw_kd"))
                cfg->claw_kd = fval;
            else if (!strcmp(key, "baud"))
                cfg->baud = (int)fval;
        }
    }
    fclose(f);
    LOG_INFO("motor_config loaded from %s", filename);
    return 0;
}
// ============================================================================
// SYSTEM CONFIG
// ============================================================================

#define SYSTEM_CONFIG_SECTION "# system_config"

system_config_t g_system_config = {
    .use_batt_adc = SYSTEM_USE_BATT_ADC_DEFAULT,
    .batt_r1      = SYSTEM_BATT_R1_DEFAULT,
    .batt_r2      = SYSTEM_BATT_R2_DEFAULT,
    .batt_trim    = SYSTEM_BATT_TRIM_DEFAULT,
};

void system_config_apply(const system_config_t *cfg)
{
    g_system_config = *cfg;
    LOG_INFO("system_config applied: use_batt_adc=%d batt_r1=%.0f batt_r2=%.0f batt_trim=%.4f",
             cfg->use_batt_adc, cfg->batt_r1, cfg->batt_r2, cfg->batt_trim);
}

int system_config_save(const char *filename, const system_config_t *cfg)
{
    if (!filename) filename = DEFAULT_CONFIG_FILE;

    FILE *f = fopen(filename, "a");
    if (!f)
    {
        LOG_ERROR("system_config_save: cannot open %s: %s", filename, strerror(errno));
        return -1;
    }
    fprintf(f, "\n%s\n", SYSTEM_CONFIG_SECTION);
    fprintf(f, "use_batt_adc=%d\n",  cfg->use_batt_adc);
    fprintf(f, "batt_r1=%.1f\n",     cfg->batt_r1);
    fprintf(f, "batt_r2=%.1f\n",     cfg->batt_r2);
    fprintf(f, "batt_trim=%.6f\n",   cfg->batt_trim);
    fclose(f);
    LOG_INFO("system_config saved to %s", filename);
    return 0;
}

int system_config_load_or_default(const char *filename, system_config_t *cfg)
{
    cfg->use_batt_adc = SYSTEM_USE_BATT_ADC_DEFAULT;
    cfg->batt_r1      = SYSTEM_BATT_R1_DEFAULT;
    cfg->batt_r2      = SYSTEM_BATT_R2_DEFAULT;
    cfg->batt_trim    = SYSTEM_BATT_TRIM_DEFAULT;

    if (!filename) filename = DEFAULT_CONFIG_FILE;
    FILE *f = fopen(filename, "r");
    if (!f) return 0;

    char line[128];
    int in_section = 0;
    while (fgets(line, sizeof(line), f))
    {
        if (strncmp(line, SYSTEM_CONFIG_SECTION, strlen(SYSTEM_CONFIG_SECTION)) == 0)
        {
            in_section = 1;
            continue;
        }
        if (!in_section) continue;
        if (line[0] == '#' || line[0] == '\n') { in_section = 0; continue; }

        char key[64];
        float fval;
        if (sscanf(line, "%63[^=]=%f", key, &fval) == 2)
        {
            if      (!strcmp(key, "use_batt_adc")) cfg->use_batt_adc = (int)fval;
            else if (!strcmp(key, "batt_r1"))      cfg->batt_r1 = fval;
            else if (!strcmp(key, "batt_r2"))      cfg->batt_r2 = fval;
            else if (!strcmp(key, "batt_trim"))    cfg->batt_trim = fval;
        }
    }
    fclose(f);
    LOG_INFO("system_config loaded from %s", filename);
    return 0;
}