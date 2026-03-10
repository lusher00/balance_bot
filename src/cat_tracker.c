/**
 * @file cat_tracker.c
 * @brief Cat position tracker via UART from RPi5
 */

#include "cat_follower.h"
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

static cat_position_t latest_cat = {0};
static pthread_mutex_t cat_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t uart_thread;
static volatile bool running = false;

static void* uart_read_thread(void* arg __attribute__((unused))) {
    char buffer[128];
    size_t pos = 0;
    
    while (running) {
        uint8_t byte;
        int n = rc_uart_read_bytes(CAT_UART_BUS, &byte, 1);
        
        if (n == 1) {
            if (byte == '\n') {
                buffer[pos] = '\0';
                
                // Parse: CAT,x,y,confidence
                if (strncmp(buffer, "CAT,", 4) == 0) {
                    float x, y, conf;
                    if (sscanf(buffer + 4, "%f,%f,%f", &x, &y, &conf) == 3) {
                        pthread_mutex_lock(&cat_mutex);
                        latest_cat.x = x;
                        latest_cat.y = y;
                        latest_cat.confidence = conf;
                        latest_cat.detected = 1;
                        latest_cat.timestamp_ns = rc_nanos_since_boot();
                        pthread_mutex_unlock(&cat_mutex);
                    }
                }
                pos = 0;
            } else if (pos < sizeof(buffer) - 1) {
                buffer[pos++] = byte;
            }
        } else {
            rc_usleep(1000);  // 1ms
        }
    }
    
    return NULL;
}

int cat_tracker_init(void) {
    LOG_INFO("Initializing cat tracker on UART%d", CAT_UART_BUS);
    
    if (rc_uart_init(CAT_UART_BUS, CAT_UART_BAUD, 0.1, 0, 1, 0) < 0) {
        LOG_ERROR("Failed to initialize UART%d", CAT_UART_BUS);
        return -1;
    }
    
    memset(&latest_cat, 0, sizeof(latest_cat));
    running = true;
    
    if (pthread_create(&uart_thread, NULL, uart_read_thread, NULL) != 0) {
        LOG_ERROR("Failed to create UART thread");
        rc_uart_close(CAT_UART_BUS);
        return -1;
    }
    
    LOG_INFO("Cat tracker initialized");
    return 0;
}

int cat_tracker_get_position(cat_position_t* pos) {
    pthread_mutex_lock(&cat_mutex);
    
    // Check if data is stale
    uint64_t age_ms = (rc_nanos_since_boot() - latest_cat.timestamp_ns) / 1000000;
    if (age_ms > CAT_TIMEOUT_MS) {
        latest_cat.detected = 0;
    }
    
    *pos = latest_cat;
    pthread_mutex_unlock(&cat_mutex);
    
    return pos->detected;
}

void cat_tracker_cleanup(void) {
    LOG_INFO("Cleaning up cat tracker");
    
    running = false;
    pthread_join(uart_thread, NULL);
    rc_uart_close(CAT_UART_BUS);
    
    LOG_INFO("Cat tracker cleaned up");
}
