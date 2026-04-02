/**
 * @file uart_input.c
 * @brief Generic packet-based UART input reader.
 *
 * Reads newline-terminated ASCII packets from any UART and exposes the
 * latest decoded packet via uart_input_get().
 *
 * Wire format:
 *   PKT,<x>,<y>,<conf>\n
 *
 * Fields:
 *   x    – lateral / steering normalised float  (-1 to +1)
 *   y    – forward / drive normalised float     (-1 to +1)
 *   conf – source confidence                    ( 0 to  1)
 */

#include "balance_bot.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define LINE_BUF_LEN  128

static input_packet_t  g_latest      = {0};
static pthread_mutex_t g_mutex       = PTHREAD_MUTEX_INITIALIZER;
static pthread_t       g_thread;
static volatile int    g_running     = 0;
static int             g_fd          = -1;
static int             g_timeout_ms  = 500;

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:     return B115200;
    }
}

static void *reader_thread(void *arg)
{
    (void)arg;
    char line[LINE_BUF_LEN];
    int  pos = 0;

    while (g_running) {
        uint8_t byte;
        int n = (int)read(g_fd, &byte, 1);

        if (n != 1) {
            usleep(1000);
            continue;
        }

        if (byte == '\n') {
            line[pos] = '\0';
            if (pos > 0 && line[pos - 1] == '\r')
                line[--pos] = '\0';

            if (pos > 0) {
                float x, y, conf;
                if (strncmp(line, "PKT,", 4) == 0 &&
                    sscanf(line + 4, "%f,%f,%f", &x, &y, &conf) == 3) {
                    pthread_mutex_lock(&g_mutex);
                    g_latest.x            = x;
                    g_latest.y            = y;
                    g_latest.confidence   = conf;
                    g_latest.timestamp_ns = rc_nanos_since_boot();
                    g_latest.valid        = 1;
                    pthread_mutex_unlock(&g_mutex);
                }
            }
            pos = 0;
        } else if (pos < LINE_BUF_LEN - 1) {
            line[pos++] = (char)byte;
        } else {
            pos = 0;
        }
    }

    return NULL;
}

int uart_input_init(const char *device, int baud, int timeout_ms)
{
    if (!device) device = "/dev/ttyO1";
    g_timeout_ms = timeout_ms;

    LOG_INFO("uart_input: opening %s at %d baud, timeout %d ms",
             device, baud, timeout_ms);

    g_fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (g_fd < 0) {
        LOG_ERROR("uart_input: open(%s) failed: %s", device, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(g_fd, &tty);
    speed_t spd = baud_to_speed(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;   /* 100 ms read timeout */
    if (tcsetattr(g_fd, TCSANOW, &tty) < 0) {
        LOG_ERROR("uart_input: tcsetattr failed: %s", strerror(errno));
        close(g_fd); g_fd = -1;
        return -1;
    }

    /* switch to blocking for the reader thread */
    int flags = fcntl(g_fd, F_GETFL, 0);
    fcntl(g_fd, F_SETFL, flags & ~O_NONBLOCK);

    memset(&g_latest, 0, sizeof(g_latest));
    g_running = 1;

    if (pthread_create(&g_thread, NULL, reader_thread, NULL) != 0) {
        LOG_ERROR("uart_input: failed to create reader thread");
        close(g_fd); g_fd = -1; g_running = 0;
        return -1;
    }

    LOG_INFO("uart_input: ready on %s", device);
    return 0;
}

int uart_input_get(input_packet_t *pkt)
{
    pthread_mutex_lock(&g_mutex);
    uint64_t age_ms = (rc_nanos_since_boot() - g_latest.timestamp_ns) / 1000000ULL;
    if (age_ms > (uint64_t)g_timeout_ms)
        g_latest.valid = 0;
    *pkt = g_latest;
    pthread_mutex_unlock(&g_mutex);
    return pkt->valid;
}

void uart_input_cleanup(void)
{
    LOG_INFO("uart_input: shutting down");
    g_running = 0;
    pthread_join(g_thread, NULL);
    if (g_fd >= 0) { close(g_fd); g_fd = -1; }
    LOG_INFO("uart_input: done");
}
