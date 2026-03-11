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
 *
 * The prefix ("PKT") is intentionally short to be easy to change from
 * whichever coprocessor (RPi, ESP32, etc.) is sending the data.
 * Lines that don't match the format are silently ignored.
 */

#include "balance_bot.h"

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#define UART_INPUT_BUS      1           /* default rc_uart bus if none specified */
#define LINE_BUF_LEN        128

static input_packet_t  g_latest      = {0};
static pthread_mutex_t g_mutex       = PTHREAD_MUTEX_INITIALIZER;
static pthread_t       g_thread;
static volatile int    g_running     = 0;
static int             g_uart_bus    = UART_INPUT_BUS;
static int             g_timeout_ms  = 500;

static void *reader_thread(void *arg)
{
    (void)arg;
    char line[LINE_BUF_LEN];
    int  pos = 0;

    while (g_running) {
        uint8_t byte;
        int n = rc_uart_read_bytes(g_uart_bus, &byte, 1);

        if (n != 1) {
            rc_usleep(1000);    /* 1 ms — avoid busy-spin on empty UART */
            continue;
        }

        if (byte == '\n') {
            line[pos] = '\0';

            /* strip optional trailing \r */
            if (pos > 0 && line[pos - 1] == '\r')
                line[--pos] = '\0';

            if (pos > 0) {
                float x, y, conf;
                /* Accept "PKT,x,y,conf" — prefix is exactly 4 chars */
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
            /* line too long — discard and start fresh */
            pos = 0;
        }
    }

    return NULL;
}

int uart_input_init(const char *device, int baud, int timeout_ms)
{
    /* rc_uart_init takes a bus index, not a path.
     * BeagleBone UARTs are numbered 1-5; the caller passes the path
     * e.g. "/dev/ttyO1" and we strip the index from the last char. */
    if (device) {
        int len = (int)strlen(device);
        if (len > 0) {
            char last = device[len - 1];
            if (last >= '0' && last <= '9')
                g_uart_bus = last - '0';
        }
    }

    g_timeout_ms = timeout_ms;

    LOG_INFO("uart_input: opening UART%d (%s) at %d baud, timeout %d ms",
             g_uart_bus, device ? device : "default", baud, timeout_ms);

    if (rc_uart_init(g_uart_bus, baud, 0.1, 0, 1, 0) < 0) {
        LOG_ERROR("uart_input: failed to open UART%d", g_uart_bus);
        return -1;
    }

    memset(&g_latest, 0, sizeof(g_latest));
    g_running = 1;

    if (pthread_create(&g_thread, NULL, reader_thread, NULL) != 0) {
        LOG_ERROR("uart_input: failed to create reader thread");
        rc_uart_close(g_uart_bus);
        g_running = 0;
        return -1;
    }

    LOG_INFO("uart_input: ready on UART%d", g_uart_bus);
    return 0;
}

int uart_input_get(input_packet_t *pkt)
{
    pthread_mutex_lock(&g_mutex);

    /* mark stale if we haven't received a packet recently */
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
    rc_uart_close(g_uart_bus);
    LOG_INFO("uart_input: done");
}
