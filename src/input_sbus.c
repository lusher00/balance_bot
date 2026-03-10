/**
 * @file input_sbus.c
 * @brief FrSky R-XSR SBUS input for balance_bot
 *
 * Reads SBUS from the R-XSR receiver over a BeagleBone UART.
 * Plugs into the existing input architecture alongside input_xbox.c —
 * launch with:  balance_bot -i sbus
 *
 * Hardware wiring:
 *   R-XSR SBUS pin  →  BeagleBone UART1 RX (P9.26) via signal inverter
 *   R-XSR 5V/GND    →  BeagleBone 5V / GND
 *
 *   The R-XSR outputs inverted SBUS (115200 baud, 8E2, active-low).
 *   Use a single NPN transistor (e.g. 2N3904) with a 10k pull-up to
 *   re-invert back to UART-normal before the BeagleBone RX pin.
 *
 * TX channel mapping (AETR / OpenTX standard order):
 *
 *   CH1   Ail  Right stick X  →  Yaw / turn          (-1.0 to +1.0)
 *   CH2   Ele  Right stick Y  →  Forward / back       (-1.0 to +1.0, fwd = +)
 *   CH3   Thr  Left  stick Y  →  Throttle (unused)    raw only
 *   CH4   Rud  Left  stick X  →  (unused)             raw only
 *   CH5   SA   3-pos switch   →  Arm / Disarm         (2=armed, 0/1=disarmed)
 *   CH6   SB   3-pos switch   →  Kill switch          (2=run, 0/1=kill)
 *   CH7   S1   pot/slider     →  Aux analogue 1       (-1.0 to +1.0)
 *   CH8   S2   pot/slider     →  Aux analogue 2       (-1.0 to +1.0)
 *   CH9   SC   3-pos switch   →  Aux switch C         (0/1/2)
 *   CH10  SD   3-pos switch   →  Speed mode           (slow/normal/sport)
 *   CH11  SE   3-pos switch   →  Aux switch E         (0/1/2)
 *   CH12  SF   2-pos switch   →  Aux switch F         (bool)
 *   CH13–CH16  (stubs)        →  Reserved for future use
 *
 * Adding a new channel:
 *   1. Add a field to the sbus state struct (or use sbus_get_channel_raw())
 *   2. Update sbus_decode_channels() to populate it
 *   3. Add a getter function at the bottom of this file
 *   4. Declare the getter in cat_follower.h
 */

#include "cat_follower.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

// Custom baud rate support (115200 baud for SBUS).
// We define our own struct and ioctl numbers rather than pulling in
// linux/termios.h which conflicts with the glibc termios.h already included
// via robotcontrol.h. The values match the kernel ABI exactly.
#define SBUS_TCGETS2  _IOR('T', 0x2A, struct sbus_termios2)
#define SBUS_TCSETS2  _IOW('T', 0x2B, struct sbus_termios2)
#define SBUS_BOTHER   0010000
#define SBUS_CBAUD    0010017

struct sbus_termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t     c_line;
    cc_t     c_cc[19];
    speed_t  c_ispeed;
    speed_t  c_ospeed;
};

// ── SBUS protocol constants ───────────────────────────────────────────────
#define SBUS_FRAME_LEN      25
#define SBUS_HEADER         0x0F
#define SBUS_FOOTER         0x00
#define SBUS_FOOTER_MASK    0x0F    // Only lower nibble is footer
#define SBUS_NUM_CHANNELS   16

#define SBUS_MIN_RAW        172
#define SBUS_MAX_RAW        1811
#define SBUS_MID_RAW        992

// Switch thresholds (raw SBUS values)
#define SBUS_SW_LOW         400     // 2-pos or 3-pos: low position
#define SBUS_SW_MID         1000    // 3-pos: middle position
#define SBUS_SW_HIGH        1600    // 2-pos or 3-pos: high position

// ── Hardware config ───────────────────────────────────────────────────────
#define SBUS_UART_DEFAULT   "/dev/ttyO4"   // BeagleBone UART1
#define SBUS_BAUD           B38400         // Actual baud = 115200; closest
                                            // termios constant — see note below
/*
 * NOTE on baud rate:
 * SBUS uses 115200 baud which is non-standard. On BeagleBone you can set
 * it via a custom divisor after opening the port:
 *
 *   struct serial_struct ss;
 *   ioctl(fd, TIOCGSERIAL, &ss);
 *   ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
 *   ss.custom_divisor = ss.baud_base / 115200;
 *   ioctl(fd, TIOCSSERIAL, &ss);
 *
 * This is done automatically in sbus_open_uart() below.
 * B38400 is passed to cfsetspeed() as a placeholder; the custom divisor
 * overrides the actual baud.
 */

// ── Deadband ──────────────────────────────────────────────────────────────
#define SBUS_STICK_DEADBAND 0.03f   // ±3% around center, eliminates jitter

// ── Speed mode scale factors ──────────────────────────────────────────────
#define SPEED_SLOW_SCALE    0.4f
#define SPEED_NORMAL_SCALE  0.7f
#define SPEED_SPORT_SCALE   1.0f

// ── Internal state ────────────────────────────────────────────────────────
typedef enum {
    SPEED_SLOW   = 0,
    SPEED_NORMAL = 1,
    SPEED_SPORT  = 2
} speed_mode_t;

static struct {
    int         fd;
    bool        connected;

    // Raw 16-channel values (SBUS_MIN_RAW – SBUS_MAX_RAW)
    uint16_t    raw[SBUS_NUM_CHANNELS];

    // Decoded primary stick channels
    float       drive;          // CH2  Ele: forward/back  (-1 to +1)
    float       turn;           // CH1  Ail: yaw           (-1 to +1)

    // Decoded switch channels
    int         arm;            // CH5  SA:  3-pos (0=low 1=mid 2=high)
    int         kill;           // CH6  SB:  3-pos (0=low 1=mid 2=high) — 0=kill active
    speed_mode_t speed_mode;    // CH10 SD:  slow/normal/sport

    // Decoded aux analogue channels
    float       aux1;           // CH7  S1:  pot/slider    (-1 to +1)
    float       aux2;           // CH8  S2:  pot/slider    (-1 to +1)

    // Decoded aux switches
    int         sw_c;           // CH9  SC:  3-pos (0=low 1=mid 2=high)
    int         sw_e;           // CH11 SE:  3-pos (0=low 1=mid 2=high)
    bool        sw_f;           // CH12 SF:  2-pos switch

    // SBUS frame flags (byte 23)
    bool        failsafe;
    bool        frame_lost;

    // Sync buffer
    uint8_t     buf[SBUS_FRAME_LEN * 2];
    int         buf_pos;

    char        device[64];
} sbus = {
    .fd         = -1,
    .connected  = false,
    .drive      = 0.0f,
    .turn       = 0.0f,
    .arm        = 0,            // Default low (disarmed)
    .kill       = 0,            // Default low (kill active until first frame)
    .speed_mode = SPEED_NORMAL,
    .aux1       = 0.0f,
    .aux2       = 0.0f,
    .sw_c       = 0,            // Default low
    .sw_e       = 1,            // Default mid
    .sw_f       = false,
    .failsafe   = false,
    .frame_lost = false,
    .buf_pos    = 0,
    .device     = SBUS_UART_DEFAULT,
};

// ── Helpers ───────────────────────────────────────────────────────────────

/**
 * @brief Convert raw SBUS value to normalised float
 *
 * Maps SBUS_MIN_RAW → -1.0, SBUS_MID_RAW → 0.0, SBUS_MAX_RAW → +1.0
 */
static float sbus_raw_to_float(uint16_t raw) {
    float f = ((float)(raw - SBUS_MID_RAW)) /
              ((float)(SBUS_MAX_RAW - SBUS_MIN_RAW) / 2.0f);
    if (f >  1.0f) f =  1.0f;
    if (f < -1.0f) f = -1.0f;
    return f;
}

/**
 * @brief Apply symmetric deadband
 */
static float apply_deadband(float val, float db) {
    if (val >  db) return (val - db) / (1.0f - db);
    if (val < -db) return (val + db) / (1.0f - db);
    return 0.0f;
}

/**
 * @brief Decode 25-byte SBUS frame into raw[] channel array
 *
 * Channels are packed as 11-bit values, LSB first.
 */
static void sbus_decode_frame(const uint8_t *frame) {
    const uint8_t *b = frame + 1;   // Skip header byte

    sbus.raw[0]  = ((b[0]       | (uint16_t)b[1]  << 8)  & 0x07FF);
    sbus.raw[1]  = ((b[1]  >> 3 | (uint16_t)b[2]  << 5)  & 0x07FF);
    sbus.raw[2]  = ((b[2]  >> 6 | (uint16_t)b[3]  << 2
                              | (uint16_t)b[4]  << 10) & 0x07FF);
    sbus.raw[3]  = ((b[4]  >> 1 | (uint16_t)b[5]  << 7)  & 0x07FF);
    sbus.raw[4]  = ((b[5]  >> 4 | (uint16_t)b[6]  << 4)  & 0x07FF);
    sbus.raw[5]  = ((b[6]  >> 7 | (uint16_t)b[7]  << 1
                              | (uint16_t)b[8]  << 9)  & 0x07FF);
    sbus.raw[6]  = ((b[8]  >> 2 | (uint16_t)b[9]  << 6)  & 0x07FF);
    sbus.raw[7]  = ((b[9]  >> 5 | (uint16_t)b[10] << 3)  & 0x07FF);
    sbus.raw[8]  = ((b[11]      | (uint16_t)b[12] << 8)  & 0x07FF);
    sbus.raw[9]  = ((b[12] >> 3 | (uint16_t)b[13] << 5)  & 0x07FF);
    sbus.raw[10] = ((b[13] >> 6 | (uint16_t)b[14] << 2
                              | (uint16_t)b[15] << 10) & 0x07FF);
    sbus.raw[11] = ((b[15] >> 1 | (uint16_t)b[16] << 7)  & 0x07FF);
    sbus.raw[12] = ((b[16] >> 4 | (uint16_t)b[17] << 4)  & 0x07FF);
    sbus.raw[13] = ((b[17] >> 7 | (uint16_t)b[18] << 1
                              | (uint16_t)b[19] << 9)  & 0x07FF);
    sbus.raw[14] = ((b[19] >> 2 | (uint16_t)b[20] << 6)  & 0x07FF);
    sbus.raw[15] = ((b[20] >> 5 | (uint16_t)b[21] << 3)  & 0x07FF);

    // Flags byte (b[22] = frame[23])
    uint8_t flags  = frame[23];
    sbus.frame_lost = (flags & 0x20) != 0;
    sbus.failsafe   = (flags & 0x08) != 0;
}

/**
 * @brief Decode raw channels into semantic values
 *
 * This is the only function you need to touch when adding channels.
 * Each active channel is decoded here; stubs show where to add more.
 */
static void sbus_decode_channels(void) {
    // ── CH1  Ail  Right stick X  →  Yaw / turn ───────────────────────────
    sbus.turn  = apply_deadband(sbus_raw_to_float(sbus.raw[0]),
                                SBUS_STICK_DEADBAND);

    // ── CH2  Ele  Right stick Y  →  Drive forward/back ───────────────────
    // Invert: stick forward (high raw) → positive drive
    sbus.drive = apply_deadband(-sbus_raw_to_float(sbus.raw[1]),
                                SBUS_STICK_DEADBAND);

    // ── CH3  Thr  Left stick Y  →  (unused — raw available via getter) ───

    // ── CH4  Rud  Left stick X  →  (unused — raw available via getter) ───

    // ── CH5  SA  3-pos  →  Arm / Disarm ──────────────────────────────────
    // Convention: 2=high = armed, 0=low or 1=mid = disarmed
    if      (sbus.raw[4] < SBUS_SW_LOW)  sbus.arm = 0;
    else if (sbus.raw[4] > SBUS_SW_HIGH) sbus.arm = 2;
    else                                  sbus.arm = 1;

    // ── CH6  SB  3-pos  →  Kill switch ───────────────────────────────────
    // Convention: 0=low = kill active (safe default), 2=high = run enabled
    if      (sbus.raw[5] < SBUS_SW_LOW)  sbus.kill = 0;
    else if (sbus.raw[5] > SBUS_SW_HIGH) sbus.kill = 2;
    else                                  sbus.kill = 1;

    // ── CH7  S1  pot/slider  →  Aux analogue 1 ───────────────────────────
    sbus.aux1 = sbus_raw_to_float(sbus.raw[6]);

    // ── CH8  S2  pot/slider  →  Aux analogue 2 ───────────────────────────
    sbus.aux2 = sbus_raw_to_float(sbus.raw[7]);

    // ── CH9  SC  3-pos  →  Aux switch C ──────────────────────────────────
    if      (sbus.raw[8] < SBUS_SW_LOW)  sbus.sw_c = 0;
    else if (sbus.raw[8] > SBUS_SW_HIGH) sbus.sw_c = 2;
    else                                  sbus.sw_c = 1;

    // ── CH10  SD  3-pos  →  Speed mode ───────────────────────────────────
    if      (sbus.raw[9] < SBUS_SW_LOW)  sbus.speed_mode = SPEED_SLOW;
    else if (sbus.raw[9] > SBUS_SW_HIGH) sbus.speed_mode = SPEED_SPORT;
    else                                  sbus.speed_mode = SPEED_NORMAL;

    // ── CH11  SE  3-pos  →  Aux switch E (0=low 1=mid 2=high) ───────────
    if      (sbus.raw[10] < SBUS_SW_LOW)  sbus.sw_e = 0;
    else if (sbus.raw[10] > SBUS_SW_HIGH) sbus.sw_e = 2;
    else                                   sbus.sw_e = 1;

    // ── CH12  SF  2-pos  →  Aux switch F ─────────────────────────────────
    sbus.sw_f = (sbus.raw[11] > SBUS_SW_HIGH);

    // ── CH13–CH16  stubs  ─────────────────────────────────────────────────
    // Access via sbus_get_channel_raw(12..15) or sbus_get_channel_float(n)
}

// ── UART setup ────────────────────────────────────────────────────────────

static int sbus_open_uart(const char *device) {
    int fd = open(device, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        LOG_ERROR("SBUS: failed to open %s: %s", device, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        LOG_ERROR("SBUS: tcgetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    // 8E2 — even parity, 2 stop bits (SBUS spec)
    tty.c_cflag  = CS8 | CSTOPB | PARENB | CLOCAL | CREAD;
    tty.c_iflag  = 0;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;   // Non-blocking

    cfsetispeed(&tty, B38400);   // Placeholder; overridden below by termios2
    cfsetospeed(&tty, B38400);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        LOG_ERROR("SBUS: tcsetattr failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

	// Set exact 100000 baud (SBUS) using custom divisor — same method as test_sbus.c
    struct serial_struct ss;
    if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
        ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
        ss.custom_divisor = ss.baud_base / 100000;
        if (ioctl(fd, TIOCSSERIAL, &ss) < 0) {
            LOG_WARN("SBUS: TIOCSSERIAL failed — baud may be wrong: %s", strerror(errno));
        } else {
            LOG_INFO("SBUS: baud_base=%d divisor=%d actual=%d",
                     ss.baud_base, ss.custom_divisor,
                     ss.baud_base / ss.custom_divisor);
        }
    } else {
        LOG_WARN("SBUS: TIOCGSERIAL failed — baud may be wrong: %s", strerror(errno));
    }
    fcntl(fd, F_SETFL, 0);
    
    tcflush(fd, TCIFLUSH);
    LOG_INFO("SBUS: UART open on %s (115200 baud 8E2)", device);
    return fd;
}

// ── Public API ────────────────────────────────────────────────────────────

/**
 * @brief Initialize SBUS input
 *
 * @param device  UART device path, e.g. "/dev/ttyO4".
 *                Pass NULL to use SBUS_UART_DEFAULT.
 * @return 0 on success, -1 on error
 */
int sbus_init(const char *device) {
    if (!device) device = SBUS_UART_DEFAULT;
    strncpy(sbus.device, device, sizeof(sbus.device) - 1);

    sbus.fd = sbus_open_uart(sbus.device);
    if (sbus.fd < 0) return -1;

    sbus.connected = true;
    sbus.kill      = true;  // Kill active until first valid frame
    sbus.buf_pos   = 0;

    LOG_INFO("SBUS: initialized — kill ACTIVE until first valid frame");
    return 0;
}

/**
 * @brief Read and process pending SBUS bytes
 *
 * Call this from robot_run() at 100 Hz alongside xbox_update().
 * Syncs on the SBUS header/footer bytes so partial reads are handled
 * gracefully.
 *
 * @return  1  valid frame decoded this call
 *          0  no complete frame yet (normal, call again next cycle)
 *         -1  UART error / not connected
 */
int sbus_update(void) {
    if (!sbus.connected || sbus.fd < 0) return -1;

    uint8_t byte;
    int decoded = 0;

    while (read(sbus.fd, &byte, 1) == 1) {
        // Sync: wait for header at position 0
        if (sbus.buf_pos == 0 && byte != SBUS_HEADER) continue;

        sbus.buf[sbus.buf_pos++] = byte;

        if (sbus.buf_pos == SBUS_FRAME_LEN) {
            sbus.buf_pos = 0;

            // Validate footer (lower nibble of byte 24)
            if ((sbus.buf[24] & SBUS_FOOTER_MASK) != SBUS_FOOTER) {
                LOG_DEBUG("SBUS: bad footer 0x%02X — resyncing", sbus.buf[24]);
                continue;
            }

            sbus_decode_frame(sbus.buf);

            if (sbus.failsafe) {
                LOG_WARN("SBUS: FAILSAFE active — forcing kill");
                sbus.kill  = 0;
                sbus.arm   = 0;
                sbus.drive = 0.0f;
                sbus.turn  = 0.0f;
                sbus.aux1  = 0.0f;
                sbus.aux2  = 0.0f;
            } else if (sbus.frame_lost) {
                // Single lost frame — hold last values, don't kill yet
                LOG_DEBUG("SBUS: frame lost flag set");
            } else {
                sbus_decode_channels();
            }

            decoded = 1;
        }
    }

    return decoded;
}

/**
 * @brief Get forward/back drive command
 * @return -1.0 (full reverse) to +1.0 (full forward), scaled by speed mode
 */
float sbus_get_drive(void) {
    if (!sbus.connected || sbus.kill < 2 || sbus.arm < 2) return 0.0f;
    float scale = (sbus.speed_mode == SPEED_SLOW)   ? SPEED_SLOW_SCALE :
                  (sbus.speed_mode == SPEED_SPORT)  ? SPEED_SPORT_SCALE :
                                                       SPEED_NORMAL_SCALE;
    return sbus.drive * scale;
}

/**
 * @brief Get yaw/turn command
 * @return -1.0 (full left) to +1.0 (full right), scaled by speed mode
 */
float sbus_get_turn(void) {
    if (!sbus.connected || sbus.kill < 2 || sbus.arm < 2) return 0.0f;
    float scale = (sbus.speed_mode == SPEED_SLOW)   ? SPEED_SLOW_SCALE :
                  (sbus.speed_mode == SPEED_SPORT)  ? SPEED_SPORT_SCALE :
                                                       SPEED_NORMAL_SCALE;
    return sbus.turn * scale;
}

/**
 * @brief Get arm state from CH5 SA (3-pos switch)
 * @return 0=low(disarmed), 1=mid, 2=high(armed)
 */
int sbus_get_arm(void) {
    return sbus.connected ? sbus.arm : 0;
}

/**
 * @brief Get kill switch state from CH6 SB (3-pos switch)
 *
 * Kill is active (motors stop) unless switch is fully HIGH (pos 2).
 * Mid position also kills — must be intentionally moved to high to run.
 *
 * @return 0=low(kill), 1=mid(kill), 2=high(run enabled)
 */
int sbus_get_kill(void) {
    if (!sbus.connected || sbus.failsafe) return 0;
    return sbus.kill;
}

/**
 * @brief Get speed mode from CH5 (SD 3-pos switch)
 * @return 0=slow, 1=normal, 2=sport
 */
int sbus_get_speed_mode(void) {
    return (int)sbus.speed_mode;
}

/**
 * @brief Get failsafe state
 * @return true if receiver is in failsafe (TX link lost)
 */
bool sbus_get_failsafe(void) {
    return sbus.failsafe;
}

/**
 * @brief Get raw 11-bit value for any channel (0-indexed)
 *
 * Use this when adding new channel assignments without modifying
 * sbus_decode_channels(). Channel range is 172–1811.
 *
 * @param ch  Channel index 0–15
 * @return Raw SBUS value, or SBUS_MID_RAW if out of range
 */
uint16_t sbus_get_channel_raw(int ch) {
    if (ch < 0 || ch >= SBUS_NUM_CHANNELS) return SBUS_MID_RAW;
    return sbus.raw[ch];
}

/**
 * @brief Get normalised float value for any channel (0-indexed)
 *
 * Convenience wrapper around sbus_get_channel_raw().
 * Returns -1.0 to +1.0 with no deadband applied.
 *
 * @param ch  Channel index 0–15
 * @return Normalised value -1.0 to +1.0
 */
float sbus_get_channel_float(int ch) {
    return sbus_raw_to_float(sbus_get_channel_raw(ch));
}

/**
 * @brief Get aux analogue 1 from CH7 (S1 pot/slider)
 * @return -1.0 to +1.0, no deadband applied
 */
float sbus_get_aux1(void) {
    return sbus.connected ? sbus.aux1 : 0.0f;
}

/**
 * @brief Get aux analogue 2 from CH8 (S2 pot/slider)
 * @return -1.0 to +1.0, no deadband applied
 */
float sbus_get_aux2(void) {
    return sbus.connected ? sbus.aux2 : 0.0f;
}

/**
 * @brief Get aux switch C from CH9 (SC 3-pos)
 * @return 0=low, 1=mid, 2=high
 */
int sbus_get_sw_c(void) {
    return sbus.connected ? sbus.sw_c : 0;
}

/**
 * @brief Get aux switch E from CH11 (SE 3-pos)
 * @return 0=low, 1=mid, 2=high
 */
int sbus_get_sw_e(void) {
    return sbus.connected ? sbus.sw_e : 1;
}

/**
 * @brief Get aux switch F from CH12 (SF 2-pos)
 * @return true = high position
 */
bool sbus_get_sw_f(void) {
    return sbus.connected && sbus.sw_f;
}

/**
 * @brief Check if SBUS receiver is connected and sending valid frames
 */
bool sbus_is_connected(void) {
    return sbus.connected;
}

/**
 * @brief Cleanup SBUS input — close UART
 */
void sbus_cleanup(void) {
    if (sbus.fd >= 0) {
        close(sbus.fd);
        sbus.fd = -1;
    }
    sbus.connected = false;
    LOG_INFO("SBUS: cleanup complete");
}
