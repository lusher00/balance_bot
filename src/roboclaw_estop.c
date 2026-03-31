/**
 * roboclaw_estop.c — hardware e-stop control for RoboClaw 2x7A
 * GPIO1_25 → S4 (active low, non-latching configured via roboclaw_util.py)
 * WriteNVM required to clear latched state after e-stop.
 */

#include <robotcontrol.h>
#include <termios.h>      // tcflush, TCIOFLUSH
#include <sys/select.h>   // select
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#define ESTOP_CHIP 1
#define ESTOP_PIN 25

/* ── CRC16 ── */
static uint16_t crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

/* ── Assert e-stop (GPIO low) ── */
void roboclaw_estop_assert(void)
{
    rc_gpio_set_value(ESTOP_CHIP, ESTOP_PIN, 0);
}

/* ── Init GPIO as output, deassert ── */
int roboclaw_estop_init(void)
{
    if (rc_gpio_init(ESTOP_CHIP, ESTOP_PIN, GPIOHANDLE_REQUEST_OUTPUT) < 0)
        return -1;
    rc_gpio_set_value(ESTOP_CHIP, ESTOP_PIN, 1);
    return 0;
}

/**
 * roboclaw_estop_clear() — clear e-stop and reset RoboClaw
 * Call this on re-arm. Blocks ~3.5s while unit resets.
 * fd = open serial fd for ttyO1 (your existing roboclaw serial fd)
 * addr = RoboClaw address (0x80)
 */
int roboclaw_estop_clear(int fd, uint8_t addr)
{
    /* 1. Drive GPIO high */
    rc_gpio_set_value(ESTOP_CHIP, ESTOP_PIN, 1);
    usleep(100000); /* 100ms — let RoboClaw see the line */

    /* 2. WriteNVM with unlock key — triggers reset */
    uint8_t buf[7];
    buf[0] = addr;
    buf[1] = 94;
    buf[2] = 0xE2;
    buf[3] = 0x2E;
    buf[4] = 0xAB;
    buf[5] = 0x7A;
    uint16_t crc = crc16(buf, 6);
    buf[6] = 0; /* placeholder — we write separately */

    uint8_t pkt[8];
    memcpy(pkt, buf, 6);
    pkt[6] = (crc >> 8) & 0xFF;
    pkt[7] = crc & 0xFF;
    write(fd, pkt, 8);

    /* 3. Wait for ACK (optional — unit resets immediately after) */
    uint8_t ack = 0;
    struct timeval tv = {1, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    if (select(fd + 1, &fds, NULL, NULL, &tv) > 0)
        read(fd, &ack, 1);

    /* 4. Wait for unit to come back up */
    sleep(3);

    /* 5. Flush and send stop */
    tcflush(fd, TCIOFLUSH);
    uint8_t stop[10];
    stop[0] = addr;
    stop[1] = 37;           /* MIXEDSPEED */
    memset(stop + 2, 0, 8); /* speed=0 both motors */
    /* rebuild properly */
    uint8_t spkt[12];
    spkt[0] = addr;
    spkt[1] = 37;
    memset(spkt + 2, 0, 8);
    uint16_t sc = crc16(spkt, 10);
    spkt[10] = (sc >> 8) & 0xFF;
    spkt[11] = sc & 0xFF;
    write(fd, spkt, 12);

    return (ack == 0xFF) ? 0 : -1;
}