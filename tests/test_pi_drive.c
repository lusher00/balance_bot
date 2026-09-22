/* Host test for pi_drive.c: who drives this tick, and command expiry.
 *   make test-pi-drive      (builds and runs on the Mac or the Bone)
 */
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include "debug_config.h"
#include "balance_bot.h"

/* Logging config normally owned by the main program. */
debug_config_t g_debug_config;

#define DIS PI_DRIVE_DISARMED
#define GATE PI_DRIVE_GATE_CLOSED
#define KILL PI_DRIVE_RC_KILL
#define STICK PI_DRIVE_STICK
#define NOCMD PI_DRIVE_NO_COMMAND
#define APPLY PI_DRIVE_APPLYING

static int failures = 0;
static void expect(const char *what, pi_drive_state_t got, pi_drive_state_t want)
{
    int ok = got == want;
    printf("%s %-58s %s\n", ok ? "PASS" : "FAIL", what, pi_drive_state_name(got));
    failures += !ok;
}

int main(void)
{
    /*                              armed gate sbus kill drive turn fresh */
    expect("disarmed beats everything",
           pi_drive_decide(0, 1, 1, 2, 0.5f, 0, 1), DIS);
    expect("gate closed: Pi ignored",
           pi_drive_decide(1, 0, 0, 0, 0, 0, 1), GATE);
    expect("transmitter on, kill switch not at RUN: Pi ignored",
           pi_drive_decide(1, 1, 1, 0, 0, 0, 1), KILL);
    expect("kill mid position also blocks",
           pi_drive_decide(1, 1, 1, 1, 0, 0, 1), KILL);
    expect("stick off centre wins over a fresh Pi command",
           pi_drive_decide(1, 1, 1, 2, 0.2f, 0, 1), STICK);
    expect("turn stick alone also wins",
           pi_drive_decide(1, 1, 1, 2, 0, -0.1f, 1), STICK);
    expect("stick centred, fresh command: Pi drives",
           pi_drive_decide(1, 1, 1, 2, 0, 0, 1), APPLY);
    expect("transmitter off (kill reads 0), fresh command: Pi drives",
           pi_drive_decide(1, 1, 0, 0, 0, 0, 1), APPLY);
    expect("gate open but command expired: centred stick",
           pi_drive_decide(1, 1, 0, 0, 0, 0, 0), NOCMD);

    /* Expiry and clamping through the real store. */
    float x, y;
    pi_drive_set(2.0f, -0.4f, 100);
    assert(pi_drive_get(&x, &y) == 1);
    printf("%s clamps to -1..1                                          x=%.2f y=%.2f\n",
           (x == 1.0f && y == -0.4f) ? "PASS" : "FAIL", x, y);
    failures += !(x == 1.0f && y == -0.4f);
    struct timespec ts = {0, 160 * 1000000L};
    nanosleep(&ts, NULL);
    int fresh = pi_drive_get(&x, &y);
    printf("%s expires after ttl_ms and reads as zero                    fresh=%d x=%.2f\n",
           (!fresh && x == 0.0f && y == 0.0f) ? "PASS" : "FAIL", fresh, x);
    failures += !(!fresh && x == 0.0f && y == 0.0f);
    pi_drive_set(0.1f, 0.1f, 5);            /* ttl below the floor -> 50 ms */
    ts.tv_nsec = 20 * 1000000L;
    nanosleep(&ts, NULL);
    printf("%s ttl is floored at %d ms                                   fresh=%d\n",
           pi_drive_get(&x, &y) ? "PASS" : "FAIL", PI_DRIVE_TTL_MIN_MS, pi_drive_get(&x, &y));
    failures += !pi_drive_get(&x, &y);

    pi_drive_set_gate(1);
    assert(pi_drive_gate() == 1);
    pi_drive_set_gate(0);
    assert(pi_drive_gate() == 0);

    printf("\n%s\n", failures ? "FAILED" : "all pi_drive tests passed");
    return failures != 0;
}
