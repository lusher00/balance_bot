/**
 * @file mpu_dmp.c
 * @brief MPU-9250 DMP driver — ported from librobotcontrol to raw i2c-dev.
 *
 * Replaces librobotcontrol rc_mpu_initialize_dmp() / rc_mpu_set_dmp_callback().
 * No magnetometer, no calibration files, no librobotcontrol dependency.
 *
 * Implements the full Invensense MotionApps DMP init sequence:
 *   1. Reset MPU
 *   2. Configure gyro/accel FSR and DLPF
 *   3. Load DMP firmware with bank-boundary-aware writes + readback verify
 *   4. Set orientation matrix
 *   5. Enable 6x LP quaternion + tap features
 *   6. Set FIFO rate
 *   7. Enable DMP and interrupts
 *   8. Spin background thread polling FIFO, call user callback
 */

#include "rc_compat.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "dmp_firmware.h"
#include "dmpKey.h"
#include "dmpmap.h"

/* ── MPU-9250 register map ──────────────────────────────────────────────── */
#define MPU_SMPLRT_DIV 0x19
#define MPU_CONFIG 0x1A
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_CONFIG2 0x1D
#define MPU_FIFO_EN 0x23
#define MPU_INT_PIN_CFG 0x37
#define MPU_INT_ENABLE 0x38
#define MPU_INT_STATUS 0x3A
#define MPU_USER_CTRL 0x6A
#define MPU_PWR_MGMT_1 0x6B
#define MPU_PWR_MGMT_2 0x6C
#define MPU_BANK_SEL 0x6D
#define MPU_MEM_START_ADDR 0x6E
#define MPU_MEM_R_W 0x6F
#define MPU_DMP_CFG_1 0x70
#define MPU_DMP_CFG_2 0x71
#define MPU_FIFO_COUNTH 0x72
#define MPU_FIFO_COUNTL 0x73
#define MPU_FIFO_R_W 0x74
#define MPU_WHO_AM_I 0x75

/* USER_CTRL bits */
#define BIT_FIFO_EN 0x40
#define BIT_DMP_EN 0x80
#define BIT_FIFO_RST 0x04
#define BIT_DMP_RST 0x08
#define BIT_DMP_INT_EN 0x02
#define BIT_FIFO_SIZE_1024 0x40

/* INT_PIN_CFG bits */
#define LATCH_INT_EN 0x20
#define INT_ANYRD_CLEAR 0x10
#define ACTL_ACTIVE_LOW 0x80
#define BYPASS_EN 0x02

/* PWR_MGMT_1 bits */
#define H_RESET 0x80

/* INT_SRC */
#define INT_SRC_TAP 0x01

/* DMP constants */
#define DMP_BANK_SIZE 256
#define DMP_LOAD_CHUNK 16
#define DMP_SAMPLE_RATE 200 /* internal DMP rate, always 200 Hz */
#define DMP_START_ADDR 0x0400
#define DMP_INT_CONTINUOUS 0
#define DMP_INT_GESTURE 1

/* GYRO_SF for 2000 dps */
#define GYRO_SF (46850825LL)

/* Feature flags */
#define DMP_FEATURE_TAP 0x001
#define DMP_FEATURE_6X_LP_QUAT 0x010
#define DMP_FEATURE_GYRO_CAL 0x020
#define DMP_FEATURE_SEND_RAW_ACCEL 0x040
#define DMP_FEATURE_SEND_RAW_GYRO 0x080
#define DMP_FEATURE_SEND_CAL_GYRO 0x100
#define DMP_FEATURE_SEND_ANY_GYRO (DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_SEND_CAL_GYRO)

/* FIFO packet lengths */
#define FIFO_LEN_QUAT_TAP 20            /* 16 quat + 4 tap */
#define FIFO_LEN_QUAT_ACCEL_GYRO_TAP 32 /* 16 quat + 6 accel + 6 gyro + 4 tap */
#define MAX_FIFO_BUFFER (FIFO_LEN_QUAT_ACCEL_GYRO_TAP * 5)

/* Quaternion sanity check */
#define QUAT_ERROR_THRESH (1L << 16)
#define QUAT_MAG_SQ_NORMALIZED (1L << 28)
#define QUAT_MAG_SQ_MIN (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)

/* TAP axes */
#define TAP_X 0x01
#define TAP_Y 0x02
#define TAP_Z 0x04
#define TAP_XYZ 0x07

/* DMP memory addresses (from Invensense MotionApps / librobotcontrol common.h) */
#define D_0_22          (22+512)    /* 534  = 0x0216 */
#define D_0_104         104         /* 104  = 0x0068 */
#define CFG_6           2753        /* 2753 = 0x0AC1 */
#define CFG_15          2727        /* 2727 = 0x0AA7 */
#define CFG_20          2224        /* 2224 = 0x08B0 */
#define CFG_27          2742        /* 2742 = 0x0AB6 */
#define CFG_LP_QUAT     2712        /* 2712 = 0x0A98 */
#define CFG_8           2718        /* 2718 = 0x0A9E — 6x LP quat enable */
#define FCFG_1          1062        /* 1062 = 0x0426 */
#define FCFG_2          1066        /* 1066 = 0x042A */
#define FCFG_3          1088        /* 1088 = 0x0440 */
#define FCFG_7          1073        /* 1073 = 0x0431 */
#define CFG_FIFO_ON_EVENT 2690      /* 2690 = 0x0A82 */
#define CFG_GYRO_RAW_DATA 2722      /* 2722 = 0x0AA2 */
#define CFG_ANDROID_ORIENT_INT 1853 /* 1853 = 0x073D */
#define CFG_MOTION_BIAS 1208        /* 1208 = 0x04B8 */
#define D_1_36          (256 + 36)
#define D_1_40          (256 + 40)
#define D_1_44          (256 + 44)
#define D_1_72          (256 + 72)
#define D_1_79          (256 + 79)
#define D_1_88          (256 + 88)
#define D_1_90          (256 + 90)
#define D_1_92          (256 + 92)
#define D_1_218         (256 + 218)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define RAD_TO_DEG 57.295779513

/* ── Module state ───────────────────────────────────────────────────────── */
static int g_i2c_fd = -1;
static int g_i2c_addr = 0x68;
static int g_dmp_en = 0;
static int g_packet_len = 0;
static volatile int g_running = 0;
static pthread_t g_thread;

typedef void (*_mpu_cb_t)(void);
static _mpu_cb_t g_callback = NULL;
static rc_mpu_data_t *g_data = NULL;

void _mpu_set_data_ptr(void *data)
{
    g_data = (rc_mpu_data_t *)data;
}

/* ── Raw i2c helpers ────────────────────────────────────────────────────── */

static int i2c_write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    if (write(g_i2c_fd, buf, 2) != 2)
        return -1;
    return 0;
}

static int i2c_read_byte(uint8_t reg, uint8_t *val)
{
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data idata;
    msgs[0].addr  = g_i2c_addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;
    msgs[1].addr  = g_i2c_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = 1;
    msgs[1].buf   = val;
    idata.msgs  = msgs;
    idata.nmsgs = 2;
    return ioctl(g_i2c_fd, I2C_RDWR, &idata) < 0 ? -1 : 0;
}

static int i2c_write_bytes(uint8_t reg, uint16_t len, const uint8_t *data)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    if (write(g_i2c_fd, buf, len + 1) != (ssize_t)(len + 1))
        return -1;
    return 0;
}

static int i2c_read_bytes(uint8_t reg, uint16_t len, uint8_t *data)
{
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data idata;
    msgs[0].addr  = g_i2c_addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;
    msgs[1].addr  = g_i2c_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = len;
    msgs[1].buf   = data;
    idata.msgs  = msgs;
    idata.nmsgs = 2;
    return ioctl(g_i2c_fd, I2C_RDWR, &idata) < 0 ? -1 : (int)len;
}

static int i2c_read_word(uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    if (i2c_read_bytes(reg, 2, buf) < 0)
        return -1;
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

/* ── DMP memory write/read (bank-boundary aware) ────────────────────────── */

static int dmp_write_mem(uint16_t mem_addr, uint16_t length, const uint8_t *data)
{
    if (!data)
        return -1;
    uint8_t bank = mem_addr >> 8;
    uint8_t offset = mem_addr & 0xFF;
    /* Refuse to cross bank boundary in a single call — same as librobotcontrol */
    if (offset + length > DMP_BANK_SIZE)
    {
        fprintf(stderr, "dmp_write_mem: exceeds bank boundary\n");
        return -1;
    }
    uint8_t tmp[2] = {bank, offset};
    if (i2c_write_bytes(MPU_BANK_SEL, 2, tmp))
        return -1;
    if (i2c_write_bytes(MPU_MEM_R_W, length, data))
        return -1;
    return 0;
}

static int dmp_read_mem(uint16_t mem_addr, uint16_t length, uint8_t *data)
{
    if (!data)
        return -1;
    uint8_t bank = mem_addr >> 8;
    uint8_t offset = mem_addr & 0xFF;
    if (offset + length > DMP_BANK_SIZE)
    {
        fprintf(stderr, "dmp_read_mem: exceeds bank boundary\n");
        return -1;
    }
    uint8_t tmp[2] = {bank, offset};
    if (i2c_write_bytes(MPU_BANK_SEL, 2, tmp))
        return -1;
    if (i2c_read_bytes(MPU_MEM_R_W, length, data) != length)
        return -1;
    return 0;
}

/* ── Reset FIFO ─────────────────────────────────────────────────────────── */

static int mpu_reset_fifo(void)
{
    uint8_t data;

    /* Disable interrupts, FIFO, DMP */
    if (i2c_write_byte(MPU_INT_ENABLE, 0x00)) return -1;
    if (i2c_write_byte(MPU_FIFO_EN, 0x00))    return -1;
    if (i2c_write_byte(MPU_USER_CTRL, 0x00))  return -1;

    /* Reset FIFO and DMP */
    data = BIT_FIFO_RST | BIT_DMP_RST;
    if (i2c_write_byte(MPU_USER_CTRL, data)) return -1;
    usleep(50000);

    /* Re-enable DMP + FIFO */
    data = BIT_DMP_EN | BIT_FIFO_EN;
    if (i2c_write_byte(MPU_USER_CTRL, data)) return -1;

    /* Re-enable DMP interrupt */
    data = BIT_DMP_INT_EN;
    if (i2c_write_byte(MPU_INT_ENABLE, data)) return -1;
    data = 0;
    if (i2c_write_byte(MPU_FIFO_EN, data)) return -1;

    return 0;
}

/* ── Load DMP firmware ──────────────────────────────────────────────────── */

static int dmp_load_firmware(void)
{
    uint16_t ii;
    uint16_t this_write;
    uint8_t cur[DMP_LOAD_CHUNK];
    uint8_t tmp[2];

    for (ii = 0; ii < (uint16_t)dmp_firmware_size; ii += this_write)
    {
        this_write = min(DMP_LOAD_CHUNK, (uint16_t)dmp_firmware_size - ii);
        if (dmp_write_mem(ii, this_write, dmp_firmware + ii))
        {
            fprintf(stderr, "mpu_dmp: firmware write failed at 0x%04X\n", ii);
            return -1;
        }
        if (dmp_read_mem(ii, this_write, cur))
        {
            fprintf(stderr, "mpu_dmp: firmware readback failed at 0x%04X\n", ii);
            return -1;
        }
        if (memcmp(dmp_firmware + ii, cur, this_write))
        {
            fprintf(stderr, "mpu_dmp: firmware verify failed at 0x%04X\n", ii);
            return -2;
        }
    }

    /* Set DMP program start address */
    tmp[0] = DMP_START_ADDR >> 8;
    tmp[1] = DMP_START_ADDR & 0xFF;
    if (i2c_write_bytes(MPU_DMP_CFG_1, 2, tmp))
    {
        fprintf(stderr, "mpu_dmp: failed to write program start address\n");
        return -1;
    }
    return 0;
}

/* ── Set orientation (identity = 0x88) ─────────────────────────────────── */

static int dmp_set_orientation(unsigned short orient)
{
    const uint8_t gyro_axes[3] = {DINA4C, DINACD, DINA6C};
    const uint8_t accel_axes[3] = {DINA0C, DINAC9, DINA2C};
    const uint8_t gyro_sign[3] = {DINA36, DINA56, DINA76};
    const uint8_t accel_sign[3] = {DINA26, DINA46, DINA66};
    uint8_t gyro_regs[3], accel_regs[3];

    gyro_regs[0] = gyro_axes[orient & 3];
    gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
    gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
    accel_regs[0] = accel_axes[orient & 3];
    accel_regs[1] = accel_axes[(orient >> 3) & 3];
    accel_regs[2] = accel_axes[(orient >> 6) & 3];

    if (dmp_write_mem(FCFG_1, 3, gyro_regs))
        return -1;
    if (dmp_write_mem(FCFG_2, 3, accel_regs))
        return -1;

    memcpy(gyro_regs, gyro_sign, 3);
    memcpy(accel_regs, accel_sign, 3);
    if (orient & 4)
    {
        gyro_regs[0] |= 1;
        accel_regs[0] |= 1;
    }
    if (orient & 0x20)
    {
        gyro_regs[1] |= 1;
        accel_regs[1] |= 1;
    }
    if (orient & 0x100)
    {
        gyro_regs[2] |= 1;
        accel_regs[2] |= 1;
    }

    if (dmp_write_mem(FCFG_3, 3, gyro_regs))
        return -1;
    if (dmp_write_mem(FCFG_7, 3, accel_regs))
        return -1;
    return 0;
}

/* ── Set FIFO rate ──────────────────────────────────────────────────────── */

static int dmp_set_fifo_rate(unsigned short rate)
{
    const uint8_t regs_end[12] = {
        DINAFE, DINAF2, DINAAB,
        0xc4, DINAAA, DINAF1,
        DINADF, DINADF, 0xBB, 0xAF, DINADF, DINADF};
    unsigned short div;
    uint8_t tmp[8];

    if (rate > DMP_SAMPLE_RATE)
        return -1;
    div = DMP_SAMPLE_RATE / rate - 1;
    tmp[0] = (div >> 8) & 0xFF;
    tmp[1] = div & 0xFF;
    if (dmp_write_mem(D_0_22, 2, tmp))
        return -1;
    if (dmp_write_mem(CFG_6, 12, (uint8_t *)regs_end))
        return -1;
    return 0;
}

/* ── Disable 3-axis LP quaternion (must be off when using 6x) ───────────── */

static int dmp_enable_lp_quat(int enable)
{
    uint8_t regs[4];
    if (enable) {
        regs[0] = DINBC0;
        regs[1] = DINBC2;
        regs[2] = DINBC4;
        regs[3] = DINBC6;
    } else {
        memset(regs, 0x8B, 4);
    }
    return dmp_write_mem(CFG_LP_QUAT, 4, regs);
}

/* ── Gyro calibration enable ────────────────────────────────────────────── */

static int dmp_enable_gyro_cal(int enable)
{
    if (enable) {
        uint8_t regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
        return dmp_write_mem(CFG_MOTION_BIAS, 9, regs);
    } else {
        uint8_t regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
        return dmp_write_mem(CFG_MOTION_BIAS, 9, regs);
    }
}

/* ── Enable 6x LP quaternion ────────────────────────────────────────────── */

static int dmp_enable_6x_lp_quat(int enable)
{
    uint8_t regs[4];
    if (enable)
    {
        regs[0] = DINA20;
        regs[1] = DINA28;
        regs[2] = DINA30;
        regs[3] = DINA38;
    }
    else
    {
        regs[0] = regs[1] = regs[2] = regs[3] = 0xA3;
    }
    return dmp_write_mem(CFG_8, 4, regs);
}

/* ── Enable features ────────────────────────────────────────────────────── */

static int dmp_enable_feature(unsigned short mask)
{
    uint8_t tmp[10];

    /* Set integration scale factor */
    tmp[0] = (uint8_t)((GYRO_SF >> 24) & 0xFF);
    tmp[1] = (uint8_t)((GYRO_SF >> 16) & 0xFF);
    tmp[2] = (uint8_t)((GYRO_SF >> 8) & 0xFF);
    tmp[3] = (uint8_t)(GYRO_SF & 0xFF);
    if (dmp_write_mem(D_0_104, 4, tmp))
        return -1;

    /* Sensor data to FIFO */
    tmp[0] = 0xA3;
    if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
    {
        tmp[1] = 0xC0;
        tmp[2] = 0xC8;
        tmp[3] = 0xC2;
    }
    else
    {
        tmp[1] = 0xA3;
        tmp[2] = 0xA3;
        tmp[3] = 0xA3;
    }
    if (mask & DMP_FEATURE_SEND_ANY_GYRO)
    {
        tmp[4] = 0xC4;
        tmp[5] = 0xCC;
        tmp[6] = 0xC6;
    }
    else
    {
        tmp[4] = 0xA3;
        tmp[5] = 0xA3;
        tmp[6] = 0xA3;
    }
    tmp[7] = tmp[8] = tmp[9] = 0xA3;
    if (dmp_write_mem(CFG_15, 10, tmp))
        return -1;

    /* Gesture (tap) data to FIFO */
    tmp[0] = (mask & DMP_FEATURE_TAP) ? DINA20 : 0xD8;
    if (dmp_write_mem(CFG_27, 1, tmp))
        return -1;

    /* Gyro calibration */
    if (mask & DMP_FEATURE_SEND_ANY_GYRO)
    {
        if (mask & DMP_FEATURE_SEND_CAL_GYRO)
        {
            tmp[0] = 0xB2;
            tmp[1] = 0x8B;
            tmp[2] = 0xB6;
            tmp[3] = 0x9B;
        }
        else
        {
            tmp[0] = DINAC0;
            tmp[1] = DINA80;
            tmp[2] = DINAC2;
            tmp[3] = DINA90;
        }
        if (dmp_write_mem(CFG_GYRO_RAW_DATA, 4, tmp))
            return -1;
    }

    /* Tap feature */
    if (mask & DMP_FEATURE_TAP)
    {
        tmp[0] = 0xF8;
        if (dmp_write_mem(CFG_20, 1, tmp))
            return -1;

        /* tap threshold 250 mg/ms on all axes */
        {
            /* ACCEL_FSR_8G: scale = 4096 */
            double scaled = 250.0 / DMP_SAMPLE_RATE;
            unsigned short dmp_thresh = (unsigned short)(scaled * 4096);
            unsigned short dmp_thresh2 = (unsigned short)(scaled * 3072);
            uint8_t t[4];
            t[0] = dmp_thresh >> 8;
            t[1] = dmp_thresh & 0xFF;
            t[2] = dmp_thresh2 >> 8;
            t[3] = dmp_thresh2 & 0xFF;
            if (dmp_write_mem(DMP_TAP_THX, 2, t))
                return -1;
            if (dmp_write_mem(D_1_36, 2, t + 2))
                return -1;
            if (dmp_write_mem(DMP_TAP_THY, 2, t))
                return -1;
            if (dmp_write_mem(D_1_40, 2, t + 2))
                return -1;
            if (dmp_write_mem(DMP_TAP_THZ, 2, t))
                return -1;
            if (dmp_write_mem(D_1_44, 2, t + 2))
                return -1;
        }
        /* tap axes = XYZ */
        tmp[0] = 0x3F;
        if (dmp_write_mem(D_1_72, 1, tmp))
            return -1;
        /* min taps = 1 */
        tmp[0] = 0;
        if (dmp_write_mem(D_1_79, 1, tmp))
            return -1;
        /* tap time 100ms */
        {
            unsigned short t = 100 / (1000 / DMP_SAMPLE_RATE);
            uint8_t tb[2] = {t >> 8, t & 0xFF};
            if (dmp_write_mem(DMP_TAPW_MIN, 2, tb))
                return -1;
        }
        /* tap time multi 600ms */
        {
            unsigned short t = 600 / (1000 / DMP_SAMPLE_RATE);
            uint8_t tb[2] = {t >> 8, t & 0xFF};
            if (dmp_write_mem(D_1_218, 2, tb))
                return -1;
        }
        /* shake reject thresh 300 dps */
        {
            long thresh_scaled = GYRO_SF / 1000 * 300;
            uint8_t t[4];
            t[0] = (thresh_scaled >> 24) & 0xFF;
            t[1] = (thresh_scaled >> 16) & 0xFF;
            t[2] = (thresh_scaled >> 8) & 0xFF;
            t[3] = thresh_scaled & 0xFF;
            if (dmp_write_mem(D_1_92, 4, t))
                return -1;
        }
        /* shake reject time 80ms */
        {
            unsigned short t = 80 / (1000 / DMP_SAMPLE_RATE);
            uint8_t tb[2] = {t >> 8, t & 0xFF};
            if (dmp_write_mem(D_1_90, 2, tb))
                return -1;
        }
        /* shake reject timeout 100ms */
        {
            unsigned short t = 100 / (1000 / DMP_SAMPLE_RATE);
            uint8_t tb[2] = {t >> 8, t & 0xFF};
            if (dmp_write_mem(D_1_88, 2, tb))
                return -1;
        }
    }
    else
    {
        tmp[0] = 0xD8;
        if (dmp_write_mem(CFG_20, 1, tmp))
            return -1;
    }

    /* Android orient — disabled */
    tmp[0] = 0xD8;
    if (dmp_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp))
        return -1;

    /* Gyro calibration — disabled (matches librobotcontrol default) */
    if (dmp_enable_gyro_cal(0))
        return -1;

    /* Disable 3-axis LP quat — must be off when using 6x LP quat */
    if (dmp_enable_lp_quat(0))
        return -1;

    /* Enable 6x LP quaternion */
    if (dmp_enable_6x_lp_quat(mask & DMP_FEATURE_6X_LP_QUAT))
        return -1;

    /* Reset FIFO now that features are configured */
    mpu_reset_fifo();

    /* Compute packet length */
    g_packet_len = 0;
    if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
        g_packet_len += 6;
    if (mask & DMP_FEATURE_SEND_ANY_GYRO)
        g_packet_len += 6;
    if (mask & DMP_FEATURE_6X_LP_QUAT)
        g_packet_len += 16;
    if (mask & DMP_FEATURE_TAP)
        g_packet_len += 4;

    return 0;
}

/* ── Set DMP interrupt mode ─────────────────────────────────────────────── */

static int dmp_set_interrupt_mode(int mode)
{
    const uint8_t regs_cont[11] = {
        0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9};
    const uint8_t regs_gest[11] = {
        0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda};
    if (mode == DMP_INT_CONTINUOUS)
        return dmp_write_mem(CFG_FIFO_ON_EVENT, 11, regs_cont);
    return dmp_write_mem(CFG_FIFO_ON_EVENT, 11, regs_gest);
}

/* ── Enable DMP state ───────────────────────────────────────────────────── */

static int mpu_set_dmp_state(int enable)
{
    uint8_t tmp;
    if (enable)
    {
        /* Disable data-ready interrupt */
        i2c_write_byte(MPU_INT_ENABLE, 0x00);

        /* Set USER_CTRL: FIFO_EN (dmp_en is already 1 at this point) */
        tmp = BIT_FIFO_EN;
        i2c_write_byte(MPU_USER_CTRL, tmp);
        usleep(3000);

        /* INT_PIN_CFG = 0xB2: active low, latch, FSYNC_INT_LEVEL, bypass on */
        i2c_write_byte(MPU_INT_PIN_CFG, 0xB2);

        /* Remove FIFO elements */
        i2c_write_byte(MPU_FIFO_EN, 0x00);

        /* Enable DMP interrupt */
        i2c_write_byte(MPU_INT_ENABLE, BIT_DMP_INT_EN);

        /* Reset FIFO */
        mpu_reset_fifo();
    }
    else
    {
        i2c_write_byte(MPU_INT_ENABLE, 0x00);
        i2c_write_byte(MPU_FIFO_EN, 0x00);
        mpu_reset_fifo();
    }
    return 0;
}

/* ── Quaternion → Tait-Bryan ────────────────────────────────────────────── */

static void quat_to_tb(double *q, double *tb)
{
    /* pitch (X) */
    tb[0] = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]),
                  q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    /* roll (Y) */
    double sinr = -2.0 * (q[1] * q[3] - q[0] * q[2]);
    if (sinr > 1.0)
        sinr = 1.0;
    if (sinr < -1.0)
        sinr = -1.0;
    tb[1] = asin(sinr);
    /* yaw (Z) */
    tb[2] = atan2(2.0 * (q[2] * q[3] + q[0] * q[1]),
                  q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
}

/* ── Read FIFO ──────────────────────────────────────────────────────────── */

static int read_dmp_fifo(rc_mpu_data_t *data, uint16_t fifo_count)
{
    static int first_run = 1;
    uint8_t raw[MAX_FIFO_BUFFER];
    int i = 0;

    if (g_packet_len != FIFO_LEN_QUAT_TAP &&
        g_packet_len != FIFO_LEN_QUAT_ACCEL_GYRO_TAP)
    {
        fprintf(stderr, "mpu_dmp: bad packet_len %d\n", g_packet_len);
        return -1;
    }

    if (fifo_count == 0)
    {
        return -1;
    }
    else if (fifo_count % (uint16_t)g_packet_len == 0)
    {
        /* Drain all but the last packet — keeps us current without resetting */
        i = fifo_count - g_packet_len;
    }
    else
    {
        /* Non-aligned count — true overflow or corruption, reset */
        if (!first_run)
            fprintf(stderr, "mpu_dmp: unexpected FIFO count %d (packet_len=%d), resetting\n",
                    fifo_count, g_packet_len);
        mpu_reset_fifo();
        return -1;
    }

    memset(raw, 0, sizeof(raw));
    if (i2c_read_bytes(MPU_FIFO_R_W, fifo_count, raw) != (int)fifo_count)
    {
        fprintf(stderr, "mpu_dmp: FIFO read failed\n");
        return -1;
    }

    /* Quaternion — 32-bit big-endian, [w x y z] */
    int32_t quat[4];
    quat[0] = ((int32_t)raw[i + 0] << 24) | ((int32_t)raw[i + 1] << 16) | ((int32_t)raw[i + 2] << 8) | raw[i + 3];
    quat[1] = ((int32_t)raw[i + 4] << 24) | ((int32_t)raw[i + 5] << 16) | ((int32_t)raw[i + 6] << 8) | raw[i + 7];
    quat[2] = ((int32_t)raw[i + 8] << 24) | ((int32_t)raw[i + 9] << 16) | ((int32_t)raw[i + 10] << 8) | raw[i + 11];
    quat[3] = ((int32_t)raw[i + 12] << 24) | ((int32_t)raw[i + 13] << 16) | ((int32_t)raw[i + 14] << 8) | raw[i + 15];
    i += 16;

    /* Quaternion sanity check */
    int32_t q14[4];
    q14[0] = quat[0] >> 16;
    q14[1] = quat[1] >> 16;
    q14[2] = quat[2] >> 16;
    q14[3] = quat[3] >> 16;
    int32_t mag_sq = q14[0] * q14[0] + q14[1] * q14[1] + q14[2] * q14[2] + q14[3] * q14[3];
    if (mag_sq < QUAT_MAG_SQ_MIN || mag_sq > QUAT_MAG_SQ_MAX)
    {
        if (!first_run)
            fprintf(stderr, "mpu_dmp: quaternion out of bounds\n");
        mpu_reset_fifo();
        return -1;
    }

    /* Normalize to double */
    double q[4], sum = 0.0, qlen;
    for (int j = 0; j < 4; j++)
    {
        q[j] = (double)quat[j];
        sum += q[j] * q[j];
    }
    qlen = sqrt(sum);
    for (int j = 0; j < 4; j++)
        q[j] /= qlen;
    for (int j = 0; j < 4; j++)
        data->dmp_quat[j] = q[j];

    /* Tait-Bryan angles */
    quat_to_tb(data->dmp_quat, data->dmp_TaitBryan);

    /* Accel + gyro if present */
    if (g_packet_len == FIFO_LEN_QUAT_ACCEL_GYRO_TAP)
    {
        /* Accel: ACCEL_FSR_8G → 4096 LSB/g → m/s² */
        int16_t ax = (int16_t)(((uint16_t)raw[i + 0] << 8) | raw[i + 1]);
        int16_t ay = (int16_t)(((uint16_t)raw[i + 2] << 8) | raw[i + 3]);
        int16_t az = (int16_t)(((uint16_t)raw[i + 4] << 8) | raw[i + 5]);
        i += 6;
        data->accel[0] = ax * (9.81 / 4096.0);
        data->accel[1] = ay * (9.81 / 4096.0);
        data->accel[2] = az * (9.81 / 4096.0);

        /* Gyro: GYRO_FSR_2000DPS → 16.4 LSB/dps → rad/s */
        int16_t gx = (int16_t)(((uint16_t)raw[i + 0] << 8) | raw[i + 1]);
        int16_t gy = (int16_t)(((uint16_t)raw[i + 2] << 8) | raw[i + 3]);
        int16_t gz = (int16_t)(((uint16_t)raw[i + 4] << 8) | raw[i + 5]);
        i += 6;
        data->gyro[0] = gx / (16.4 * RAD_TO_DEG);
        data->gyro[1] = gy / (16.4 * RAD_TO_DEG);
        data->gyro[2] = gz / (16.4 * RAD_TO_DEG);
    }

    /* Tap byte */
    uint8_t tap = 0x3F & raw[i + 3];
    if (raw[i + 1] & INT_SRC_TAP)
    {
        data->last_tap_direction = tap >> 3;
        data->last_tap_count = (tap % 8) + 1;
        data->tap_detected = 1;
    }
    else
    {
        data->tap_detected = 0;
    }

    if (first_run)
        first_run = 0;
    return 0;
}

/* ── Background thread ──────────────────────────────────────────────────── */

/* Poll FIFO. Check for overflow via INT_STATUS since a full FIFO reports count=0. */
static void *mpu_thread(void *arg)
{
    (void)arg;
    usleep(200000); /* let DMP settle */
    mpu_reset_fifo(); /* drain anything accumulated during settle */
    usleep(20000);   /* brief pause after reset before polling */

    while (g_running)
    {
        /* Check INT_STATUS first — bit 4 = FIFO overflow, bit 1 = DMP interrupt */
        uint8_t int_status = 0;
        i2c_read_byte(MPU_INT_STATUS, &int_status);

        if (int_status & 0x10) /* FIFO overflow */
        {
            mpu_reset_fifo();
            usleep(20000);
            continue;
        }

        if (!(int_status & 0x02)) /* DMP interrupt not set — no data yet */
        {
            usleep(2000);
            continue;
        }

        uint16_t fifo_count = 0;
        if (i2c_read_word(MPU_FIFO_COUNTH, &fifo_count) < 0)
        {
            usleep(1000);
            continue;
        }
        if (fifo_count >= (uint16_t)g_packet_len)
        {
            if (read_dmp_fifo(g_data, fifo_count) == 0 && g_callback)
                g_callback();
            else
                usleep(2000);
        }
        else
        {
            usleep(2000);
        }
    }
    return NULL;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

int _mpu_dmp_open(int i2c_bus, int i2c_addr, int sample_rate_hz)
{
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", i2c_bus);
    g_i2c_fd = open(path, O_RDWR);
    if (g_i2c_fd < 0)
    {
        fprintf(stderr, "mpu_dmp: cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }
    g_i2c_addr = i2c_addr;
    if (ioctl(g_i2c_fd, I2C_SLAVE, g_i2c_addr) < 0)
    {
        fprintf(stderr, "mpu_dmp: I2C_SLAVE ioctl failed: %s\n", strerror(errno));
        close(g_i2c_fd);
        g_i2c_fd = -1;
        return -1;
    }

    /* WHO_AM_I check */
    uint8_t who = 0;
    i2c_read_byte(MPU_WHO_AM_I, &who);
    if (who != 0x71 && who != 0x73)
        fprintf(stderr, "mpu_dmp: WHO_AM_I=0x%02X (expected 0x71/0x73)\n", who);

    /* Reset — matches librobotcontrol: H_RESET then 10ms, chip wakes to default PWR_MGMT_1=0x01 */
    i2c_write_byte(MPU_PWR_MGMT_1, H_RESET);
    usleep(10000);
    i2c_write_byte(MPU_PWR_MGMT_2, 0x00); /* enable all sensors */

    /* ACCEL_CONFIG2: first write matches librobotcontrol early init (BIT_FIFO_SIZE_1024|0x8=0x48),
     * then overwrite with DLPF_184 setting (BIT_FIFO_SIZE_1024|ACCEL_FCHOICE_1KHZ|1 = 0x41) */
    i2c_write_byte(MPU_ACCEL_CONFIG2, 0x48);

    /* Gyro FSR 2000 dps, Accel FSR 8g, DLPF 184 Hz */
    i2c_write_byte(MPU_GYRO_CONFIG, 0x18);  /* 2000 dps */
    i2c_write_byte(MPU_ACCEL_CONFIG, 0x10); /* 8g */
    i2c_write_byte(MPU_CONFIG, 0x01);       /* DLPF 184 Hz */
    i2c_write_byte(MPU_ACCEL_CONFIG2, 0x41); /* DLPF 184 Hz + FIFO size 1024 */

    /* Sample rate = 200 Hz internal (DMP requires 200) */
    i2c_write_byte(MPU_SMPLRT_DIV, (1000 / 200) - 1);

    /* Set dmp_en=1 now — matches librobotcontrol which sets this before firmware load.
     * This affects USER_CTRL value inside mpu_set_bypass / mpu_reset_fifo. */
    g_dmp_en = 1;

    /* bypass ON before firmware load — USER_CTRL=0x00, INT_PIN_CFG=0xB2 (matches librobotcontrol) */
    i2c_write_byte(MPU_USER_CTRL, 0x00);
    usleep(3000);
    i2c_write_byte(MPU_INT_PIN_CFG, 0xB2);
    i2c_write_byte(MPU_USER_CTRL, 0x00);
    usleep(10000);

    /* Load firmware */
    fprintf(stderr, "mpu_dmp: loading firmware (%zu bytes)...\n", dmp_firmware_size);
    if (dmp_load_firmware() < 0)
    {
        fprintf(stderr, "mpu_dmp: firmware load failed\n");
        close(g_i2c_fd);
        g_i2c_fd = -1;
        return -1;
    }
    fprintf(stderr, "mpu_dmp: firmware loaded OK\n");

    /* Set orientation — X_DOWN for BBB Blue mounted with +X down, +Z forward */
    if (dmp_set_orientation(266) < 0)
    {
        fprintf(stderr, "mpu_dmp: set_orientation failed\n");
        return -1;
    }

    /* Enable features: 6x LP quat + tap + accel + gyro */
    unsigned short feat = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                          DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO;
    if (dmp_enable_feature(feat) < 0)
    {
        fprintf(stderr, "mpu_dmp: enable_feature failed\n");
        return -1;
    }

    /* Set FIFO rate to requested sample rate */
    if (dmp_set_fifo_rate((unsigned short)sample_rate_hz) < 0)
    {
        fprintf(stderr, "mpu_dmp: set_fifo_rate failed\n");
        return -1;
    }

    /* Turn DMP on — matches librobotcontrol order */
    if (mpu_set_dmp_state(1) < 0)
    {
        fprintf(stderr, "mpu_dmp: set_dmp_state failed\n");
        return -1;
    }

    /* Set interrupt mode to continuous (after dmp state, matching librobotcontrol) */
    if (dmp_set_interrupt_mode(DMP_INT_CONTINUOUS) < 0)
    {
        fprintf(stderr, "mpu_dmp: set_interrupt_mode failed\n");
        return -1;
    }

    fprintf(stderr, "mpu_dmp: init complete, packet_len=%d\n", g_packet_len);

    /* Start background thread */
    g_running = 1;
    if (pthread_create(&g_thread, NULL, mpu_thread, NULL) != 0)
    {
        fprintf(stderr, "mpu_dmp: failed to start thread\n");
        return -1;
    }

    return 0;
}

int _mpu_dmp_read(rc_mpu_data_t *out)
{
    /* Called from rc_compat.h wrapper — not used in threaded mode */
    uint16_t fifo_count = 0;
    if (i2c_read_word(MPU_FIFO_COUNTH, &fifo_count) < 0)
        return -1;
    return read_dmp_fifo(out, fifo_count);
}

void _mpu_dmp_close(void)
{
    g_running = 0;
    if (g_thread)
        pthread_join(g_thread, NULL);
    if (g_i2c_fd >= 0)
    {
        i2c_write_byte(MPU_USER_CTRL, 0x00);
        i2c_write_byte(MPU_PWR_MGMT_1, 0x40); /* sleep */
        close(g_i2c_fd);
        g_i2c_fd = -1;
    }
}

void _mpu_set_callback(void (*cb)(void))
{
    g_callback = cb;
}