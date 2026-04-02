/**
 * @file mpu_dmp.c
 * @brief MPU-9250 DMP driver over Linux i2c-dev.
 *
 * Replaces librobotcontrol's rc_mpu_initialize_dmp() / rc_mpu_set_dmp_callback().
 *
 * Communicates directly with the MPU-9250 on the BeagleBone Blue
 * (I2C bus 2, address 0x68) using the Linux i2c-dev interface.
 *
 * The DMP firmware is the same 2-KB image used by librobotcontrol and
 * the original MotionApps library.  This file:
 *   1. Wakes the MPU and sets the clock source.
 *   2. Loads the DMP firmware via I2C bank writes.
 *   3. Configures the sample rate, FIFO, and enables the DMP.
 *   4. In _mpu_dmp_read() reads one FIFO packet and extracts the
 *      quaternion + raw gyro/accel into rc_mpu_data_t.
 *
 * Output format is identical to librobotcontrol so imu_config.c
 * and the rest of the codebase work unchanged.
 */

#include "rc_compat.h"
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

/* ── MPU-9250 register map ──────────────────────────────────────── */
#define MPU_RA_XG_OFFS_TC       0x00
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_ACCEL_CONFIG2    0x1D
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

/* DMP firmware packet size (quaternion + gyro + accel) */
#define DMP_PACKET_SIZE  28

/* Gyro full-scale ±2000 dps; scale factor: 1 / (16.4 * 32768) rad/s per LSB */
#define GYRO_SCALE  (1.0 / (16.4 * 32768.0) * (3.14159265358979323846 / 180.0))
/* Accel full-scale ±2g; scale factor: 9.81 / (16384 * 65.536) */
#define ACCEL_SCALE (9.81 / 16384.0)

/* ── DMP firmware image ─────────────────────────────────────────── */
/* This is the standard MotionApps v6.12 DMP binary used by
 * librobotcontrol, identical to the one in the Invensense SDK.
 * Included as a C array to avoid a runtime file dependency.         */
#include "dmp_firmware.h"   /* uint8_t dmp_firmware[]; size_t dmp_firmware_size; */

static int  g_i2c_fd = -1;
static int  g_addr   = MPU9250_I2C_ADDR;

/* ── I2C helpers ────────────────────────────────────────────────── */

static int i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (write(g_i2c_fd, buf, 2) != 2) return -1;
    return 0;
}

static int i2c_read_reg(uint8_t reg, uint8_t *val)
{
    if (write(g_i2c_fd, &reg, 1) != 1) return -1;
    if (read(g_i2c_fd, val, 1)   != 1) return -1;
    return 0;
}

static int i2c_read_bytes(uint8_t reg, uint8_t *buf, int len)
{
    if (write(g_i2c_fd, &reg, 1) != 1) return -1;
    int n = read(g_i2c_fd, buf, len);
    return (n == len) ? 0 : -1;
}

/* ── DMP bank write ─────────────────────────────────────────────── */

static int dmp_write_mem(uint16_t mem_addr, uint16_t length, const uint8_t *data)
{
    /* Select bank */
    if (i2c_write_reg(MPU_RA_BANK_SEL, (uint8_t)(mem_addr >> 8)) < 0) return -1;
    if (i2c_write_reg(MPU_RA_MEM_START_ADDR, (uint8_t)(mem_addr & 0xFF)) < 0) return -1;

    /* Write in 32-byte chunks */
    uint16_t written = 0;
    while (written < length) {
        uint8_t chunk = (length - written > 32) ? 32 : (uint8_t)(length - written);
        uint8_t buf[33];
        buf[0] = MPU_RA_MEM_R_W;
        memcpy(&buf[1], data + written, chunk);
        if (write(g_i2c_fd, buf, 1 + chunk) != 1 + chunk) return -1;
        written += chunk;
    }
    return 0;
}

/* ── Open / init ────────────────────────────────────────────────── */

int _mpu_dmp_open(int i2c_bus, int i2c_addr, int sample_rate_hz)
{
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", i2c_bus);
    g_i2c_fd = open(path, O_RDWR);
    if (g_i2c_fd < 0) {
        fprintf(stderr, "mpu_dmp: cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }
    g_addr = i2c_addr;
    if (ioctl(g_i2c_fd, I2C_SLAVE, g_addr) < 0) {
        fprintf(stderr, "mpu_dmp: I2C_SLAVE ioctl failed: %s\n", strerror(errno));
        close(g_i2c_fd); g_i2c_fd = -1;
        return -1;
    }

    /* WHO_AM_I sanity check (MPU-9250 = 0x71) */
    uint8_t who = 0;
    i2c_read_reg(MPU_RA_WHO_AM_I, &who);
    if (who != 0x71 && who != 0x73) /* 0x73 = MPU-9255 */
        fprintf(stderr, "mpu_dmp: WHO_AM_I=0x%02X (expected 0x71/0x73 for MPU-9250/9255)\n", who);

    /* Reset */
    i2c_write_reg(MPU_RA_PWR_MGMT_1, 0x80); /* device reset */
    usleep(100000);
    i2c_write_reg(MPU_RA_PWR_MGMT_1, 0x01); /* wake, use PLL gyro X clock */
    usleep(10000);
    i2c_write_reg(MPU_RA_PWR_MGMT_2, 0x00); /* enable all sensors */

    /* Gyro ±2000 dps, Accel ±2g, DLPF 42 Hz */
    i2c_write_reg(MPU_RA_GYRO_CONFIG,   0x18);
    i2c_write_reg(MPU_RA_ACCEL_CONFIG,  0x00);
    i2c_write_reg(MPU_RA_CONFIG,        0x03); /* DLPF 42 Hz */

    /* Sample rate: 1000/(1+div) = sample_rate_hz */
    uint8_t smplrt_div = (uint8_t)((1000 / sample_rate_hz) - 1);
    i2c_write_reg(MPU_RA_SMPLRT_DIV, smplrt_div);

    /* Load DMP firmware */
    i2c_write_reg(MPU_RA_USER_CTRL, 0x00); /* disable DMP + FIFO during load */
    if (dmp_write_mem(0x0000, (uint16_t)dmp_firmware_size, dmp_firmware) < 0) {
        fprintf(stderr, "mpu_dmp: DMP firmware load failed\n");
        close(g_i2c_fd); g_i2c_fd = -1;
        return -1;
    }

    /* Set DMP program start address (0x0400) */
    i2c_write_reg(MPU_RA_DMP_CFG_1, 0x04);
    i2c_write_reg(MPU_RA_DMP_CFG_2, 0x00);

    /* Enable DMP and FIFO */
    i2c_write_reg(MPU_RA_USER_CTRL, 0xC0); /* DMP_EN | FIFO_EN */
    i2c_write_reg(MPU_RA_FIFO_EN,   0x00); /* DMP controls FIFO content */
    i2c_write_reg(MPU_RA_INT_ENABLE, 0x02); /* DMP interrupt */

    usleep(50000);
    return 0;
}

/* ── Read one DMP FIFO packet ───────────────────────────────────── */

int _mpu_dmp_read(rc_mpu_data_t *out)
{
    if (g_i2c_fd < 0) return -1;

    /* Wait for DMP interrupt / FIFO data */
    for (int i = 0; i < 100; i++) {
        uint8_t status;
        i2c_read_reg(MPU_RA_INT_STATUS, &status);
        if (status & 0x02) break; /* DMP_INT bit */
        usleep(500);
    }

    /* Read FIFO count */
    uint8_t cnt[2];
    i2c_read_bytes(MPU_RA_FIFO_COUNTH, cnt, 2);
    uint16_t fifo_count = ((uint16_t)cnt[0] << 8) | cnt[1];

    if (fifo_count < DMP_PACKET_SIZE) return -1;

    /* Drain extra packets if FIFO has accumulated */
    while (fifo_count > DMP_PACKET_SIZE) {
        uint8_t discard[DMP_PACKET_SIZE];
        i2c_read_bytes(MPU_RA_FIFO_R_W, discard, DMP_PACKET_SIZE);
        fifo_count -= DMP_PACKET_SIZE;
    }

    /* Read one packet */
    uint8_t pkt[DMP_PACKET_SIZE];
    if (i2c_read_bytes(MPU_RA_FIFO_R_W, pkt, DMP_PACKET_SIZE) < 0) return -1;

    /* ── Quaternion (bytes 0–7: qw qx qy qz as 16-bit fixed-point) ── */
    int16_t qw_raw = (int16_t)((pkt[0]  << 8) | pkt[1]);
    int16_t qx_raw = (int16_t)((pkt[4]  << 8) | pkt[5]);
    int16_t qy_raw = (int16_t)((pkt[8]  << 8) | pkt[9]);
    int16_t qz_raw = (int16_t)((pkt[12] << 8) | pkt[13]);

    /* Normalise to unit quaternion */
    double qw = qw_raw / 16384.0;
    double qx = qx_raw / 16384.0;
    double qy = qy_raw / 16384.0;
    double qz = qz_raw / 16384.0;
    double norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 1e-6) { qw /= norm; qx /= norm; qy /= norm; qz /= norm; }

    out->dmp_quat[0] = qw;
    out->dmp_quat[1] = qx;
    out->dmp_quat[2] = qy;
    out->dmp_quat[3] = qz;

    /* ── Tait-Bryan angles from quaternion ─── */
    /* pitch (X rotation) = atan2(2(qy*qz + qw*qx), qw^2 - qx^2 - qy^2 + qz^2) */
    out->dmp_TaitBryan[TB_PITCH_X] = atan2(2.0*(qy*qz + qw*qx),
                                            qw*qw - qx*qx - qy*qy + qz*qz);
    /* roll (Y rotation) = asin(-2(qx*qz - qw*qy)) */
    double sinr = -2.0*(qx*qz - qw*qy);
    if (sinr >  1.0) sinr =  1.0;
    if (sinr < -1.0) sinr = -1.0;
    out->dmp_TaitBryan[1] = asin(sinr);
    /* yaw (Z rotation) = atan2(2(qx*qy + qw*qz), qw^2 + qx^2 - qy^2 - qz^2) */
    out->dmp_TaitBryan[TB_YAW_Z] = atan2(2.0*(qx*qy + qw*qz),
                                          qw*qw + qx*qx - qy*qy - qz*qz);

    /* ── Gyro (bytes 16–21) ── */
    int16_t gx_raw = (int16_t)((pkt[16] << 8) | pkt[17]);
    int16_t gy_raw = (int16_t)((pkt[18] << 8) | pkt[19]);
    int16_t gz_raw = (int16_t)((pkt[20] << 8) | pkt[21]);
    out->gyro[0] = gx_raw * GYRO_SCALE;
    out->gyro[1] = gy_raw * GYRO_SCALE;
    out->gyro[2] = gz_raw * GYRO_SCALE;

    /* ── Accel (bytes 22–27) ── */
    int16_t ax_raw = (int16_t)((pkt[22] << 8) | pkt[23]);
    int16_t ay_raw = (int16_t)((pkt[24] << 8) | pkt[25]);
    int16_t az_raw = (int16_t)((pkt[26] << 8) | pkt[27]);
    out->accel[0] = ax_raw * ACCEL_SCALE;
    out->accel[1] = ay_raw * ACCEL_SCALE;
    out->accel[2] = az_raw * ACCEL_SCALE;

    return 0;
}

/* ── Close ──────────────────────────────────────────────────────── */

void _mpu_dmp_close(void)
{
    if (g_i2c_fd >= 0) {
        /* Put MPU to sleep */
        i2c_write_reg(MPU_RA_USER_CTRL, 0x00);
        i2c_write_reg(MPU_RA_PWR_MGMT_1, 0x40); /* sleep bit */
        close(g_i2c_fd);
        g_i2c_fd = -1;
    }
}
