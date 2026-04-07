/**
 * @file rc_compat.h
 * @brief Drop-in replacements for the librobotcontrol APIs used by balance_bot.
 *
 * Removes the dependency on librobotcontrol entirely.  Only the subset
 * of rc_* symbols actually used in this project are provided here:
 *
 *   Timing:    rc_nanos_since_boot(), rc_usleep()
 *   State:     rc_get_state(), rc_set_state(), RUNNING, EXITING
 *   Signals:   rc_enable_signal_handler()
 *   PID file:  rc_make_pid_file(), rc_remove_pid_file(), rc_kill_existing_process()
 *   LEDs:      rc_led_set(), RC_LED_GREEN, RC_LED_RED
 *   Buttons:   rc_button_init(), rc_button_set_callbacks(),
 *              RC_BTN_PIN_PAUSE, RC_BTN_PIN_MODE,
 *              RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US
 *   ADC:       rc_adc_init()  (stub — battery read handled elsewhere)
 *   IMU:       rc_mpu_config_t, rc_mpu_data_t, rc_mpu_default_config(),
 *              rc_mpu_initialize_dmp(), rc_mpu_set_dmp_callback(),
 *              rc_mpu_power_off(), TB_YAW_Z, TB_PITCH_X
 *
 * The IMU section delegates entirely to src/mpu_dmp.c which runs its own
 * background thread and calls the user callback directly.
 */

#ifndef RC_COMPAT_H
#define RC_COMPAT_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ═══════════════════════════════════════════════════════════════════════════
     * TIMING
     * ═══════════════════════════════════════════════════════════════════════════ */

    static inline uint64_t rc_nanos_since_boot(void)
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    }

    static inline void rc_usleep(unsigned int us)
    {
        usleep(us);
    }

    /* ═══════════════════════════════════════════════════════════════════════════
     * PROCESS STATE
     * ═══════════════════════════════════════════════════════════════════════════ */

    typedef enum
    {
        UNINITIALIZED = 0,
        RUNNING,
        PAUSED,
        EXITING
    } rc_state_t;

    extern volatile rc_state_t g_rc_state;

    static inline rc_state_t rc_get_state(void) { return g_rc_state; }
    static inline void rc_set_state(rc_state_t s) { g_rc_state = s; }

    /* ═══════════════════════════════════════════════════════════════════════════
     * SIGNAL HANDLER
     * ═══════════════════════════════════════════════════════════════════════════ */

    static inline void _rc_signal_handler(int sig)
    {
        (void)sig;
        rc_set_state(EXITING);
    }

    static inline int rc_enable_signal_handler(void)
    {
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = _rc_signal_handler;
        sigemptyset(&sa.sa_mask);
        if (sigaction(SIGINT, &sa, NULL) < 0)
            return -1;
        if (sigaction(SIGTERM, &sa, NULL) < 0)
            return -1;
        return 0;
    }

    /* ═══════════════════════════════════════════════════════════════════════════
     * PID FILE
     * ═══════════════════════════════════════════════════════════════════════════ */

#define RC_PID_FILE "/var/run/balance_bot.pid"

    static inline int rc_kill_existing_process(float timeout_s)
    {
        FILE *f = fopen(RC_PID_FILE, "r");
        if (!f)
            return 0;

        int old_pid = 0;
        fscanf(f, "%d", &old_pid);
        fclose(f);

        if (old_pid <= 0)
            return 0;

        if (kill(old_pid, 0) < 0)
        {
            remove(RC_PID_FILE);
            return 0;
        }

        kill(old_pid, SIGTERM);
        int waited_ms = 0;
        int limit_ms = (int)(timeout_s * 1000.0f);
        while (waited_ms < limit_ms)
        {
            usleep(50000);
            waited_ms += 50;
            if (kill(old_pid, 0) < 0)
            {
                remove(RC_PID_FILE);
                return 0;
            }
        }
        kill(old_pid, SIGKILL);
        usleep(200000);
        remove(RC_PID_FILE);
        if (kill(old_pid, 0) < 0)
            return 0; /* dead or zombie — either way, proceed */
        if (errno == ESRCH)
            return 0; /* zombie: exists in proc table but gone */
        return -3;
    }

    static inline int rc_make_pid_file(void)
    {
        FILE *f = fopen(RC_PID_FILE, "w");
        if (!f)
        {
            perror("rc_make_pid_file");
            return -1;
        }
        fprintf(f, "%d\n", (int)getpid());
        fclose(f);
        return 0;
    }

    static inline void rc_remove_pid_file(void)
    {
        remove(RC_PID_FILE);
    }

    /* ═══════════════════════════════════════════════════════════════════════════
     * LEDs  (BeagleBone Blue — sysfs)
     * ═══════════════════════════════════════════════════════════════════════════ */

#define RC_LED_GREEN 0
#define RC_LED_RED 1

    static inline int rc_led_set(int led, int value)
    {
        static const char *paths[2] = {
            "/sys/class/leds/beaglebone:green:usr0/brightness",
            "/sys/class/leds/beaglebone:green:usr1/brightness"};
        if (led < 0 || led > 1)
            return -1;
        int fd = open(paths[led], O_WRONLY);
        if (fd < 0)
            return -1;
        const char *v = value ? "1\n" : "0\n";
        write(fd, v, 2);
        close(fd);
        return 0;
    }

    /* ═══════════════════════════════════════════════════════════════════════════
     * BUTTONS  (BeagleBone Blue — sysfs GPIO polling)
     * ═══════════════════════════════════════════════════════════════════════════ */

#define RC_BTN_PIN_PAUSE 69
#define RC_BTN_PIN_MODE 68
#define RC_BTN_POLARITY_NORM_HIGH 1
#define RC_BTN_DEBOUNCE_DEFAULT_US 2000

    typedef void (*rc_btn_callback_t)(void);

    typedef struct
    {
        int pin;
        int polarity;
        int debounce_us;
        rc_btn_callback_t press_cb;
        rc_btn_callback_t release_cb;
        pthread_t thread;
        int running;
        int fd_val;
    } _rc_btn_t;

    static _rc_btn_t _g_buttons[2];
    static int _g_btn_count = 0;

    static void *_rc_btn_thread(void *arg)
    {
        _rc_btn_t *b = (_rc_btn_t *)arg;
        int prev_state = 1;

        while (b->running)
        {
            char buf[4] = {0};
            lseek(b->fd_val, 0, SEEK_SET);
            if (read(b->fd_val, buf, 3) < 1)
            {
                usleep(5000);
                continue;
            }

            int raw = atoi(buf);
            int pressed = (b->polarity == RC_BTN_POLARITY_NORM_HIGH) ? (raw == 0) : (raw == 1);

            if (pressed && !prev_state)
            {
                usleep(b->debounce_us);
                lseek(b->fd_val, 0, SEEK_SET);
                read(b->fd_val, buf, 3);
                if ((b->polarity == RC_BTN_POLARITY_NORM_HIGH) ? (atoi(buf) == 0) : (atoi(buf) == 1))
                {
                    if (b->press_cb)
                        b->press_cb();
                    prev_state = 1;
                }
            }
            else if (!pressed && prev_state)
            {
                usleep(b->debounce_us);
                lseek(b->fd_val, 0, SEEK_SET);
                read(b->fd_val, buf, 3);
                if ((b->polarity == RC_BTN_POLARITY_NORM_HIGH) ? (atoi(buf) == 1) : (atoi(buf) == 0))
                {
                    if (b->release_cb)
                        b->release_cb();
                    prev_state = 0;
                }
            }
            usleep(10000);
        }
        return NULL;
    }

    static inline int rc_button_init(int pin, int polarity, int debounce_us)
    {
        if (_g_btn_count >= 2)
            return -1;

        char path[64];
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
        if (access(path, F_OK) != 0)
        {
            int efd = open("/sys/class/gpio/export", O_WRONLY);
            if (efd >= 0)
            {
                char num[8];
                snprintf(num, sizeof(num), "%d", pin);
                write(efd, num, strlen(num));
                close(efd);
                usleep(50000);
            }
        }

        char dirpath[64];
        snprintf(dirpath, sizeof(dirpath), "/sys/class/gpio/gpio%d/direction", pin);
        int dfd = open(dirpath, O_WRONLY);
        if (dfd >= 0)
        {
            write(dfd, "in", 2);
            close(dfd);
        }

        int vfd = open(path, O_RDONLY);
        if (vfd < 0)
        {
            fprintf(stderr, "rc_button_init: cannot open %s: %s\n", path, strerror(errno));
            return -1;
        }

        _rc_btn_t *b = &_g_buttons[_g_btn_count++];
        b->pin = pin;
        b->polarity = polarity;
        b->debounce_us = debounce_us;
        b->press_cb = NULL;
        b->release_cb = NULL;
        b->running = 1;
        b->fd_val = vfd;

        pthread_create(&b->thread, NULL, _rc_btn_thread, b);
        return 0;
    }

    static inline void rc_button_set_callbacks(int pin,
                                               rc_btn_callback_t press_cb,
                                               rc_btn_callback_t release_cb)
    {
        for (int i = 0; i < _g_btn_count; i++)
        {
            if (_g_buttons[i].pin == pin)
            {
                _g_buttons[i].press_cb = press_cb;
                _g_buttons[i].release_cb = release_cb;
                return;
            }
        }
    }

    static inline void _rc_buttons_cleanup(void)
    {
        for (int i = 0; i < _g_btn_count; i++)
        {
            _g_buttons[i].running = 0;
            pthread_join(_g_buttons[i].thread, NULL);
            if (_g_buttons[i].fd_val >= 0)
                close(_g_buttons[i].fd_val);
        }
        _g_btn_count = 0;
    }

    /* ═══════════════════════════════════════════════════════════════════════════
     * ADC  (stub)
     * ═══════════════════════════════════════════════════════════════════════════ */

    static inline int rc_adc_init(void) { return 0; }

    /* ═══════════════════════════════════════════════════════════════════════════
     * IMU  —  MPU-9250 DMP
     *
     * mpu_dmp.c owns a background thread that reads the DMP FIFO and calls
     * the user callback directly.  This wrapper just wires up the API.
     * ═══════════════════════════════════════════════════════════════════════════ */

#define TB_PITCH_X 0
#define TB_YAW_Z 2

#define MPU9250_I2C_BUS 2
#define MPU9250_I2C_ADDR 0x68

    typedef struct
    {
        double dmp_quat[4];      /* [W, X, Y, Z] normalised quaternion */
        double dmp_TaitBryan[3]; /* [pitch_x, roll_y, yaw_z] radians */
        double gyro[3];          /* [x, y, z] rad/s */
        double accel[3];         /* [x, y, z] m/s^2 */
        int tap_detected;
        int last_tap_direction;
        int last_tap_count;
    } rc_mpu_data_t;

    typedef struct
    {
        int dmp_sample_rate;
        int dmp_fetch_accel_gyro;
        int i2c_bus;
        int i2c_addr;
    } rc_mpu_config_t;

    static inline rc_mpu_config_t rc_mpu_default_config(void)
    {
        rc_mpu_config_t cfg;
        cfg.dmp_sample_rate = 100;
        cfg.dmp_fetch_accel_gyro = 1;
        cfg.i2c_bus = MPU9250_I2C_BUS;
        cfg.i2c_addr = MPU9250_I2C_ADDR;
        return cfg;
    }

    /* Forward declarations — implemented in src/mpu_dmp.c */
    int _mpu_dmp_open(int i2c_bus, int i2c_addr, int sample_rate_hz);
    void _mpu_dmp_close(void);
    void _mpu_set_callback(void (*cb)(void));
    void _mpu_set_data_ptr(void *data);

    static inline int rc_mpu_initialize_dmp(rc_mpu_data_t *data, rc_mpu_config_t cfg)
    {
        _mpu_set_data_ptr(data);
        if (_mpu_dmp_open(cfg.i2c_bus, cfg.i2c_addr, cfg.dmp_sample_rate) < 0)
            return -1;
        return 0;
    }

    static inline void rc_mpu_set_dmp_callback(void (*cb)(void))
    {
        _mpu_set_callback(cb);
    }

    static inline void rc_mpu_power_off(void)
    {
        _mpu_dmp_close();
    }

    /* ═══════════════════════════════════════════════════════════════════════════
     * rc_saturate_float
     * ═══════════════════════════════════════════════════════════════════════════ */

#ifndef rc_saturate_float
#define rc_saturate_float(val, mn, mx) \
    do                                 \
    {                                  \
        if (*(val) < (mn))             \
            *(val) = (mn);             \
        else if (*(val) > (mx))        \
            *(val) = (mx);             \
    } while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* RC_COMPAT_H */