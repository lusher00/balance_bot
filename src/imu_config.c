// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#include "balance_bot.h"
#include "rc_compat.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define IMU_OFFSET_FILE "/etc/balance_bot_imu.conf"
#define IMU_OFFSET_FILE_BACKUP "/home/debian/balance_bot_imu.conf"

// pitch_axis: 0=X  1=Y(default, Y-up -Z forward)  2=Z
imu_offsets_t g_imu_offsets = {.pitch_offset = 0.0f, .yaw_offset = 0.0f, .pitch_axis = 1};

int imu_offsets_load(imu_offsets_t *offsets)
{
    FILE *f = fopen(IMU_OFFSET_FILE, "r");
    if (!f)
        f = fopen(IMU_OFFSET_FILE_BACKUP, "r");
    if (!f)
    {
        LOG_WARN("No IMU offset file — run zero_imu when upright.");
        offsets->pitch_offset = 0.0f;
        offsets->yaw_offset = 0.0f;
        offsets->pitch_axis = 1;
        return -1;
    }
    char key[64];
    float val;
    while (fscanf(f, "%63s %f", key, &val) == 2)
    {
        if (strcmp(key, "pitch_offset") == 0)
            offsets->pitch_offset = val;
        if (strcmp(key, "yaw_offset") == 0)
            offsets->yaw_offset = val;
        if (strcmp(key, "pitch_axis") == 0)
            offsets->pitch_axis = (int)val;
    }
    if (offsets->pitch_axis < 0 || offsets->pitch_axis > 2)
        offsets->pitch_axis = 1;
    fclose(f);
    LOG_INFO("IMU offsets loaded: pitch=%.2f deg  yaw=%.2f deg  pitch_axis=%d",
             offsets->pitch_offset, offsets->yaw_offset, offsets->pitch_axis);
    return 0;
}

int imu_offsets_save(const imu_offsets_t *offsets)
{
    FILE *f = fopen(IMU_OFFSET_FILE, "w");
    if (!f)
        f = fopen(IMU_OFFSET_FILE_BACKUP, "w");
    if (!f)
    {
        LOG_ERROR("Failed to save IMU offsets");
        return -1;
    }
    fprintf(f, "pitch_offset %.4f\n", offsets->pitch_offset);
    fprintf(f, "yaw_offset   %.4f\n", offsets->yaw_offset);
    fprintf(f, "pitch_axis   %d\n", offsets->pitch_axis);
    fclose(f);
    LOG_INFO("IMU offsets saved: pitch=%.2f deg  yaw=%.2f deg  pitch_axis=%d",
             offsets->pitch_offset, offsets->yaw_offset, offsets->pitch_axis);
    return 0;
}

/* ── Gravity vector helpers ──────────────────────────────────────────────────
 *
 * Orientation: Y-up, -Z forward, X right.
 * DMP quat [w,x,y,z] rotates body→world. World gravity = [0,0,-1].
 * Gravity in body frame (pointing down):
 *   gx =  2*(qx*qz - qw*qy)
 *   gy =  2*(qy*qz + qw*qx)
 *   gz =  qw²-qx²-qy²+qz²
 *
 * At upright (q=[1,0,0,0]): gx=0, gy=0, gz=1  → gravity along +Z body.
 * Pitch = rotation around X (lean forward/back).
 * Forward lean (+X rotation): gy increases.
 * atan2(gy, gz): 0° upright, positive = forward lean. ✓
 * Singularity at 90° sideways — safe.
 */

void imu_offsets_calibrate(const rc_mpu_data_t *raw, imu_offsets_t *offsets)
{
    float qw = (float)raw->dmp_quat[0];
    float qx = (float)raw->dmp_quat[1];
    float qy = (float)raw->dmp_quat[2];
    float qz = (float)raw->dmp_quat[3];

    float gy = 2.0f * (qy * qz + qw * qx);
    float gz = qw * qw - qx * qx - qy * qy + qz * qz;

    offsets->pitch_offset = atan2f(gy, gz) * RAD_TO_DEG;
    offsets->yaw_offset = raw->dmp_TaitBryan[TB_YAW_Z] * RAD_TO_DEG;
    LOG_INFO("IMU calibrated: pitch_offset=%.2f deg", offsets->pitch_offset);
}

void imu_apply_transform(const rc_mpu_data_t *raw, imu_transform_t *out,
                         const imu_offsets_t *offsets)
{
    float qw = (float)raw->dmp_quat[0];
    float qx = (float)raw->dmp_quat[1];
    float qy = (float)raw->dmp_quat[2];
    float qz = (float)raw->dmp_quat[3];

    // Gravity vector components and gyro axis — selected by offsets->pitch_axis.
    // 0=X  1=Y (default: BBB Y-up, -Z forward, pitch around X)  2=Z
    float ga, gb;
    int gyro_idx;
    if (offsets->pitch_axis == 0)
    {
        // Pitch around X
        ga = 2.0f * (qx * qy - qw * qz);
        gb = qw * qw + qx * qx - qy * qy - qz * qz;
        gyro_idx = 0;
    }
    else if (offsets->pitch_axis == 2)
    {
        // Pitch around Z
        ga = 2.0f * (qx * qz - qw * qy);
        gb = qw * qw - qx * qx - qy * qy + qz * qz;
        gyro_idx = 2;
    }
    else
    {
        // Pitch around X axis (Y-up, -Z forward) — default
        // atan2(gy, gz): 0° upright, positive = forward lean
        ga = 2.0f * (qy * qz + qw * qx);
        gb = qw * qw - qx * qx - qy * qy + qz * qz;
        gyro_idx = 0;
    }

    float pitch_deg = atan2f(ga, gb) * RAD_TO_DEG;

    // pitch_offset is the raw angle at upright. Subtract to get deviation.
    // Positive = forward lean, negative = backward lean.
    out->pitch = -(pitch_deg - offsets->pitch_offset);
    out->pitch_dot = -raw->gyro[gyro_idx] * RAD_TO_DEG;

    out->yaw = raw->dmp_TaitBryan[TB_YAW_Z] * RAD_TO_DEG - offsets->yaw_offset;
    out->yaw_dot = raw->gyro[2] * RAD_TO_DEG;

    out->roll = raw->dmp_TaitBryan[TB_PITCH_X] * RAD_TO_DEG;
    out->roll_dot = raw->gyro[0] * RAD_TO_DEG;

    out->accel_x = raw->accel[0];
    out->accel_y = raw->accel[1];
    out->accel_z = raw->accel[2];
}