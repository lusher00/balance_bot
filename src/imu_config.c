#include "balance_bot.h"
#include <robotcontrol.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define IMU_OFFSET_FILE        "/etc/balance_bot_imu.conf"
#define IMU_OFFSET_FILE_BACKUP "/home/debian/balance_bot_imu.conf"

imu_offsets_t g_imu_offsets = { .pitch_offset = 0.0f, .yaw_offset = 0.0f, .pitch_dot_offset = 0.0f };

int imu_offsets_load(imu_offsets_t *offsets)
{
    FILE *f = fopen(IMU_OFFSET_FILE, "r");
    if (!f) f = fopen(IMU_OFFSET_FILE_BACKUP, "r");
    if (!f) {
        LOG_WARN("No IMU offset file — run zero_imu when upright.");
        offsets->pitch_offset = 0.0f;
        offsets->yaw_offset   = 0.0f;
        return -1;
    }
    char key[64]; float val;
    while (fscanf(f, "%63s %f", key, &val) == 2) {
        if (strcmp(key, "pitch_offset")      == 0) offsets->pitch_offset      = val;
        if (strcmp(key, "yaw_offset")        == 0) offsets->yaw_offset        = val;
        if (strcmp(key, "pitch_dot_offset")  == 0) offsets->pitch_dot_offset  = val;
    }
    fclose(f);
    LOG_INFO("IMU offsets loaded: pitch=%.2f deg  yaw=%.2f deg",
             offsets->pitch_offset, offsets->yaw_offset);
    return 0;
}

int imu_offsets_save(const imu_offsets_t *offsets)
{
    FILE *f = fopen(IMU_OFFSET_FILE, "w");
    if (!f) f = fopen(IMU_OFFSET_FILE_BACKUP, "w");
    if (!f) { LOG_ERROR("Failed to save IMU offsets"); return -1; }
    fprintf(f, "pitch_offset     %.4f\n", offsets->pitch_offset);
    fprintf(f, "yaw_offset       %.4f\n", offsets->yaw_offset);
    fprintf(f, "pitch_dot_offset %.4f\n", offsets->pitch_dot_offset);
    fclose(f);
    LOG_INFO("IMU offsets saved: pitch=%.2f deg  yaw=%.2f deg",
             offsets->pitch_offset, offsets->yaw_offset);
    return 0;
}

void imu_offsets_calibrate(const rc_mpu_data_t *raw, imu_offsets_t *offsets)
{
    float qw = (float)raw->dmp_quat[0];
    float qx = (float)raw->dmp_quat[1];
    float qy = (float)raw->dmp_quat[2];
    float qz = (float)raw->dmp_quat[3];

    float gx = -(2.0f * (qx*qz - qw*qy));
    float gz =   qw*qw - qx*qx - qy*qy + qz*qz;

    offsets->pitch_offset     = atan2f(gz, gx) * RAD_TO_DEG;
    offsets->yaw_offset       = raw->dmp_TaitBryan[TB_YAW_Z] * RAD_TO_DEG;
    offsets->pitch_dot_offset = raw->gyro[1] * RAD_TO_DEG;
    LOG_INFO("IMU calibrated: pitch=%.2f deg  gyro_bias=%.3f deg/s",
             offsets->pitch_offset, offsets->pitch_dot_offset);
}

void imu_apply_transform(const rc_mpu_data_t *raw, imu_transform_t *out,
                          const imu_offsets_t *offsets)
{
    float qw = (float)raw->dmp_quat[0];
    float qx = (float)raw->dmp_quat[1];
    float qy = (float)raw->dmp_quat[2];
    float qz = (float)raw->dmp_quat[3];

    float gx = -(2.0f * (qx*qz - qw*qy));
    float gz =   qw*qw - qx*qx - qy*qy + qz*qz;

    float pitch_deg = atan2f(gz, gx) * RAD_TO_DEG;

    out->pitch     = pitch_deg - offsets->pitch_offset;
    out->pitch_dot = raw->gyro[1] * RAD_TO_DEG - offsets->pitch_dot_offset;

    out->yaw       = raw->dmp_TaitBryan[TB_YAW_Z] * RAD_TO_DEG - offsets->yaw_offset;
    out->yaw_dot   = raw->gyro[2] * RAD_TO_DEG;

    out->roll      = raw->dmp_TaitBryan[TB_PITCH_X] * RAD_TO_DEG;
    out->roll_dot  = raw->gyro[0] * RAD_TO_DEG;

    out->accel_x   = raw->accel[0];
    out->accel_y   = raw->accel[1];
    out->accel_z   = raw->accel[2];
}
