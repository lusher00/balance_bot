/**
 * @file display.c
 * @brief ncurses live-display thread for balance_bot
 *
 * Redraws the terminal at ~20 Hz without blocking robot_run().
 *
 * stdout/stderr are redirected through a pipe as soon as display_init()
 * is called.  A collector thread reads that pipe and:
 *   1. Writes every line to LOG_FILE  (/tmp/balance_bot.log)
 *   2. Appends it to a circular in-memory buffer shown in the Log panel
 *
 * All LOG_xxx macros and printf() calls therefore appear in both places
 * without ever touching the raw terminal.
 */

#include "display.h"
#include "balance_bot.h"
#include "debug_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <ncurses.h>
#include <robotcontrol.h>   /* rc_set_state, EXITING */

/* ── constants ──────────────────────────────────────────────────── */

#define DISP_SBUS_MID   992
#define DISP_SBUS_MAX  1811
#define BAR_W           30
#define MAX_BLOCKS       6
#define LOG_LINES       8       /* rows in the log panel               */
#define LOG_LINE_LEN  256       /* max chars per log line              */
#define LOG_FILE      "/tmp/balance_bot.log"

/* ── colour pairs ───────────────────────────────────────────────── */
enum {
    CP_TITLE = 1, CP_HDR, CP_VAL, CP_POS, CP_NEG,
    CP_WARN, CP_OK, CP_SW_ON, CP_SW_OFF, CP_DIM, CP_LOG,
};

/* ── log ring buffer ────────────────────────────────────────────── */
static char        g_log[LOG_LINES][LOG_LINE_LEN];
static int         g_log_head  = 0;   /* next write position (circular) */
static pthread_mutex_t g_log_mutex = PTHREAD_MUTEX_INITIALIZER;

static void log_append(const char *line)
{
    pthread_mutex_lock(&g_log_mutex);
    snprintf(g_log[g_log_head], LOG_LINE_LEN, "%s", line);
    g_log_head = (g_log_head + 1) % LOG_LINES;
    pthread_mutex_unlock(&g_log_mutex);
}

/* ── stdout/stderr redirect ─────────────────────────────────────── */
static int  g_pipe_fds[2]  = {-1, -1};
static int  g_stdout_saved = -1;
static int  g_stderr_saved = -1;
static FILE *g_logfile     = NULL;
static pthread_t g_collector_thread;
static volatile int g_collector_running = 0;

static void *collector_thread(void *arg)
{
    (void)arg;
    char buf[LOG_LINE_LEN];
    char line[LOG_LINE_LEN];
    int  line_pos = 0;

    while (g_collector_running) {
        ssize_t n = read(g_pipe_fds[0], buf, sizeof(buf) - 1);
        if (n <= 0) {
            usleep(10000);
            continue;
        }
        buf[n] = '\0';

        /* split on newlines, flush complete lines */
        for (ssize_t i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\n') {
                if (line_pos > 0 && line[line_pos - 1] == '\r')
                    line_pos--;
                line[line_pos] = '\0';
                if (line_pos > 0) {
                    log_append(line);
                    if (g_logfile) { fprintf(g_logfile, "%s\n", line); fflush(g_logfile); }
                }
                line_pos = 0;
            } else if (c == '\r') {
                line[line_pos] = '\0';
                if (line_pos > 0) {
                    log_append(line);
                    if (g_logfile) { fprintf(g_logfile, "%s\n", line); fflush(g_logfile); }
                    line_pos = 0;
                }
            } else {
                if (line_pos < LOG_LINE_LEN - 1)
                    line[line_pos++] = c;
                /* force-flush if line buffer is full */
                if (line_pos == LOG_LINE_LEN - 1) {
                    line[line_pos] = '\0';
                    log_append(line);
                    if (g_logfile) { fprintf(g_logfile, "%s\n", line); fflush(g_logfile); }
                    line_pos = 0;
                }
            }
        }

        /* flush any partial line that has been sitting in the buffer —
           catches messages that never end with \n (e.g. from server.js) */
        if (line_pos > 0) {
            /* peek ahead: if nothing more arrives within 5ms, flush it */
            usleep(5000);
            /* check if pipe is still empty — if read returns 0 we flush */
            char peek;
            ssize_t p = read(g_pipe_fds[0], &peek, 1);
            if (p <= 0) {
                line[line_pos] = '\0';
                log_append(line);
                if (g_logfile) { fprintf(g_logfile, "%s\n", line); fflush(g_logfile); }
                line_pos = 0;
            } else {
                /* got more data — put it back by processing it now */
                if (peek != '\n' && peek != '\r' && line_pos < LOG_LINE_LEN - 1)
                    line[line_pos++] = peek;
            }
        }
    }
    return NULL;
}

static void redirect_output(void)
{
    if (pipe(g_pipe_fds) != 0) return;

    /* make read end non-blocking so collector doesn't stall on empty pipe */
    fcntl(g_pipe_fds[0], F_SETFL, O_NONBLOCK);

    g_logfile = fopen(LOG_FILE, "w");

    /* save originals */
    g_stdout_saved = dup(STDOUT_FILENO);
    g_stderr_saved = dup(STDERR_FILENO);

    /* point stdout + stderr at write end of pipe */
    dup2(g_pipe_fds[1], STDOUT_FILENO);
    dup2(g_pipe_fds[1], STDERR_FILENO);

    /* start collector */
    g_collector_running = 1;
    pthread_create(&g_collector_thread, NULL, collector_thread, NULL);
}

static void restore_output(void)
{
    g_collector_running = 0;

    if (g_stdout_saved >= 0) { dup2(g_stdout_saved, STDOUT_FILENO); close(g_stdout_saved); g_stdout_saved = -1; }
    if (g_stderr_saved >= 0) { dup2(g_stderr_saved, STDERR_FILENO); close(g_stderr_saved); g_stderr_saved = -1; }
    if (g_pipe_fds[1] >= 0)  { close(g_pipe_fds[1]); g_pipe_fds[1] = -1; }

    pthread_join(g_collector_thread, NULL);

    if (g_pipe_fds[0] >= 0)  { close(g_pipe_fds[0]); g_pipe_fds[0] = -1; }
    if (g_logfile)            { fclose(g_logfile);     g_logfile = NULL;   }
}

/* ── layout ─────────────────────────────────────────────────────── */
enum { BLK_SBUS=0, BLK_PID, BLK_ENC, BLK_IMU, BLK_MOT, BLK_SYS };

static int block_row[MAX_BLOCKS];
static int g_log_row = 0;   /* start row of log panel */

/* ── thread state ───────────────────────────────────────────────── */
static pthread_t    g_thread;
static volatile int g_running = 0;
static volatile int g_dirty   = 0;
static int          g_enabled = 0;

/* ── drawing helpers ────────────────────────────────────────────── */

static void init_colours(void)
{
    start_color();
    use_default_colors();
    init_pair(CP_TITLE,  COLOR_CYAN,   -1);
    init_pair(CP_HDR,    COLOR_YELLOW, -1);
    init_pair(CP_VAL,    COLOR_WHITE,  -1);
    init_pair(CP_POS,    COLOR_GREEN,  -1);
    init_pair(CP_NEG,    COLOR_BLUE,   -1);
    init_pair(CP_WARN,   COLOR_RED,    -1);
    init_pair(CP_OK,     COLOR_GREEN,  -1);
    init_pair(CP_SW_ON,  COLOR_BLACK,  COLOR_GREEN);
    init_pair(CP_SW_OFF, COLOR_BLACK,  COLOR_WHITE);
    init_pair(CP_DIM,    COLOR_WHITE,  -1);
    init_pair(CP_LOG,    COLOR_CYAN,   -1);
}

static void draw_bar(int row, int col, int raw)
{
    int half = BAR_W / 2;
    float norm = (float)(raw - DISP_SBUS_MID) / (float)(DISP_SBUS_MAX - DISP_SBUS_MID);
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;
    int filled = (int)(norm * half);

    for (int i = 0; i < BAR_W; i++) {
        int cpos = i - half;
        char ch; int cp;
        if (i == half)                                       { ch='|'; cp=CP_DIM; }
        else if (filled > 0 && cpos > 0 && cpos <= filled)  { ch='#'; cp=CP_POS; }
        else if (filled < 0 && cpos < 0 && cpos >= filled)  { ch='#'; cp=CP_NEG; }
        else                                                  { ch='-'; cp=CP_DIM; }
        attron(COLOR_PAIR(cp));
        mvaddch(row, col + i, ch);
        attroff(COLOR_PAIR(cp));
    }
    mvprintw(row, col + BAR_W + 1, "%4d", raw);
}

static void draw_3pos(int row, int col, int raw, const char *hi_label)
{
    int pos = (raw < 400) ? 0 : (raw > 1600) ? 2 : 1;
    const char *lbl[3] = { "LOW ", "MID ", hi_label };
    int x = col;
    for (int i = 0; i < 3; i++) {
        attron(COLOR_PAIR(i == pos ? CP_SW_ON : CP_SW_OFF));
        mvprintw(row, x, "[%s]", lbl[i]);
        attroff(COLOR_PAIR(CP_SW_ON));
        attroff(COLOR_PAIR(CP_SW_OFF));
        x += 7;
    }
    mvprintw(row, col + 22, "%4d", raw);
}

static void draw_2pos(int row, int col, int raw)
{
    int on = (raw > 1600);
    attron(COLOR_PAIR(on ? CP_SW_OFF : CP_SW_ON));
    mvprintw(row, col,     "[OFF]");
    attroff(COLOR_PAIR(CP_SW_ON)); attroff(COLOR_PAIR(CP_SW_OFF));
    attron(COLOR_PAIR(on ? CP_SW_ON : CP_SW_OFF));
    mvprintw(row, col + 6, "[ ON]");
    attroff(COLOR_PAIR(CP_SW_ON)); attroff(COLOR_PAIR(CP_SW_OFF));
    mvprintw(row, col + 13, "%4d", raw);
}

static void hdr(int row, const char *title)
{
    attron(COLOR_PAIR(CP_HDR) | A_BOLD);
    mvhline(row, 0, ACS_HLINE, 70);
    mvprintw(row, 2, " %s ", title);
    attroff(COLOR_PAIR(CP_HDR) | A_BOLD);
}

/* ── block renderers ────────────────────────────────────────────── */

static void draw_sbus(int r)
{
    hdr(r, "SBUS TX"); r++;
    mvprintw(r,   1, "CH1  Ail   Turn "); draw_bar(r,   17, sbus_get_channel_raw(0));
    mvprintw(r+1, 1, "CH2  Ele  Drive "); draw_bar(r+1, 17, sbus_get_channel_raw(1));
    mvprintw(r+2, 1, "CH3  Thr        "); draw_bar(r+2, 17, sbus_get_channel_raw(2));
    mvprintw(r+3, 1, "CH4  Rud        "); draw_bar(r+3, 17, sbus_get_channel_raw(3));
    mvprintw(r+4, 1, "CH7  S1   Aux1  "); draw_bar(r+4, 17, sbus_get_channel_raw(6));
    mvprintw(r+5, 1, "CH8  S2   Aux2  "); draw_bar(r+5, 17, sbus_get_channel_raw(7));
    mvprintw(r+6,  1, "CH5  SA   Arm   "); draw_3pos(r+6,  17, sbus_get_channel_raw(4),  "ARM ");
    mvprintw(r+7,  1, "CH6  SB   Kill  "); draw_3pos(r+7,  17, sbus_get_channel_raw(5),  "RUN ");
    mvprintw(r+8,  1, "CH9  SC   AuxC  "); draw_3pos(r+8,  17, sbus_get_channel_raw(8),  "HIGH");
    mvprintw(r+9,  1, "CH10 SD   Speed "); draw_3pos(r+9,  17, sbus_get_channel_raw(9),  "SPRT");
    mvprintw(r+10, 1, "CH11 SE   AuxE  "); draw_3pos(r+10, 17, sbus_get_channel_raw(10), "HIGH");
    mvprintw(r+11, 1, "CH12 SF   AuxF  "); draw_2pos(r+11, 17, sbus_get_channel_raw(11));
    int fr = r + 12;
    mvprintw(fr, 1, "Flags: ");
    attron(COLOR_PAIR(sbus_get_failsafe() ? CP_WARN : CP_OK));
    printw("FAILSAFE:%-3s  ", sbus_get_failsafe() ? "YES" : "no");
    attroff(COLOR_PAIR(CP_WARN)); attroff(COLOR_PAIR(CP_OK));
    attron(COLOR_PAIR(!sbus_is_connected() ? CP_WARN : CP_OK));
    printw("CONNECTED:%-3s", sbus_is_connected() ? "yes" : "NO");
    attroff(COLOR_PAIR(CP_WARN)); attroff(COLOR_PAIR(CP_OK));
}

static void draw_pid(int r)
{
    hdr(r, "PID"); r++;
    const char *names[3]      = { "D1 Balance", "D2 Drive  ", "D3 Steer  " };
    pid_telemetry_t *pids[3]  = { &g_telemetry_data.D1_balance,
                                   &g_telemetry_data.D2_drive,
                                   &g_telemetry_data.D3_steering };
    attron(A_DIM);
    mvprintw(r, 1, "%-10s  %7s  %7s  %7s  %7s  %7s  %7s  %7s",
             "Controller", "setp", "meas", "err", "P", "I", "D", "out");
    attroff(A_DIM); r++;
    for (int i = 0; i < 3; i++) {
        pid_telemetry_t *p = pids[i];
        mvprintw(r+i, 1, "%-10s  %+7.3f  %+7.3f  %+7.3f  %+7.3f  %+7.3f  %+7.3f  %+7.3f",
                 names[i], p->setpoint, p->measurement, p->error,
                 p->p_term, p->i_term, p->d_term, p->output);
    }
}

static void draw_encoders(int r)
{
    hdr(r, "Encoders"); r++;
    encoder_telemetry_t *e = &g_telemetry_data.encoders;
    attron(A_DIM);
    mvprintw(r, 1, "%-6s  %8s  %10s  %10s", "Side", "ticks", "pos (rad)", "vel (rad/s)");
    attroff(A_DIM); r++;
    mvprintw(r,   1, "Left    %8d  %+10.4f  %+10.4f", e->left_ticks,  e->left_rad,  e->left_vel);
    mvprintw(r+1, 1, "Right   %8d  %+10.4f  %+10.4f", e->right_ticks, e->right_rad, e->right_vel);
}

static void draw_imu(int r)
{
    hdr(r, "IMU"); r++;
    imu_telemetry_t *m = &g_telemetry_data.imu;
    attron(A_DIM);
    mvprintw(r, 1, "%-8s  %9s  %9s", "Axis", "angle(rad)", "rate(r/s)");
    attroff(A_DIM); r++;
    mvprintw(r,   1, "Pitch     %+9.4f  %+9.4f", m->theta, m->theta_dot);
    mvprintw(r+1, 1, "Roll      %+9.4f  %+9.4f", m->phi,   m->phi_dot);
    mvprintw(r+2, 1, "Yaw       %+9.4f  %+9.4f", m->psi,   m->psi_dot);
}

static void draw_motors(int r)
{
    hdr(r, "Motors"); r++;
    float l  = g_telemetry_data.motors.left_duty;
    float ri = g_telemetry_data.motors.right_duty;
    mvprintw(r,   1, "Left  "); draw_bar(r,   7, (int)((l  + 1.0f) * DISP_SBUS_MID));
    mvprintw(r,   BAR_W + 10, "%+6.3f", l);
    mvprintw(r+1, 1, "Right "); draw_bar(r+1, 7, (int)((ri + 1.0f) * DISP_SBUS_MID));
    mvprintw(r+1, BAR_W + 10, "%+6.3f", ri);
}

static void draw_system(int r)
{
    hdr(r, "System"); r++;
    system_telemetry_t *s = &g_telemetry_data.system;
    attron(COLOR_PAIR(s->armed ? CP_OK : CP_WARN));
    mvprintw(r, 1, "Armed: %-5s", s->armed ? "YES" : "no");
    attroff(COLOR_PAIR(CP_OK)); attroff(COLOR_PAIR(CP_WARN));
    const char *modes[] = { "BALANCE", "EXT_INPUT", "MANUAL" };
    mvprintw(r, 18, "Mode: %-12s", s->mode < 3 ? modes[s->mode] : "?");
    attron(COLOR_PAIR(s->battery_voltage < 10.5f ? CP_WARN : CP_OK));
    mvprintw(r, 40, "Batt: %.2fV", s->battery_voltage);
    attroff(COLOR_PAIR(CP_WARN)); attroff(COLOR_PAIR(CP_OK));
    mvprintw(r, 54, "theta: %+.2f deg", g_telemetry_data.imu.theta);
}

static void draw_log(int r)
{
    hdr(r, "Log  (full log: " LOG_FILE ")"); r++;
    int cols = getmaxx(stdscr) - 2;
    if (cols < 1) cols = 78;

    pthread_mutex_lock(&g_log_mutex);
    for (int i = 0; i < LOG_LINES; i++) {
        int idx = (g_log_head + i) % LOG_LINES;

        /* copy and strip non-printable chars */
        char clean[LOG_LINE_LEN];
        int  out = 0;
        for (int j = 0; g_log[idx][j] && out < cols; j++) {
            unsigned char c = (unsigned char)g_log[idx][j];
            clean[out++] = (c >= 32 && c < 127) ? c : ' ';
        }
        clean[out] = '\0';

        attron(COLOR_PAIR(CP_LOG));
        mvprintw(r + i, 1, "%-*s", cols, clean);
        attroff(COLOR_PAIR(CP_LOG));
    }
    pthread_mutex_unlock(&g_log_mutex);
}

/* ── layout calculation ─────────────────────────────────────────── */

static int block_lines[MAX_BLOCKS];

static void calc_layout(void)
{
    display_config_t *D = &g_debug_config.display;

    block_lines[BLK_SBUS] = 15;
    block_lines[BLK_PID]  = 6;
    block_lines[BLK_ENC]  = 5;
    block_lines[BLK_IMU]  = 6;
    block_lines[BLK_MOT]  = 4;
    block_lines[BLK_SYS]  = 3;

    int enabled[MAX_BLOCKS] = {
        D->sbus_tx, D->pid, D->encoders, D->imu, D->motors, D->system
    };

    int row = 1;
    for (int i = 0; i < MAX_BLOCKS; i++) {
        block_row[i] = enabled[i] ? row : -1;
        if (enabled[i]) row += block_lines[i];
    }

    g_log_row = row;   /* log panel always at the bottom */
}

/* ── main redraw ────────────────────────────────────────────────── */

static void redraw(void)
{
    display_config_t *D = &g_debug_config.display;

    attron(COLOR_PAIR(CP_TITLE) | A_BOLD);
    mvprintw(0, 0, " balance_bot monitor   q=quit   log: " LOG_FILE " ");
    attroff(COLOR_PAIR(CP_TITLE) | A_BOLD);

    if (D->sbus_tx  && block_row[BLK_SBUS] >= 0) draw_sbus    (block_row[BLK_SBUS]);
    if (D->pid      && block_row[BLK_PID]  >= 0) draw_pid     (block_row[BLK_PID]);
    if (D->encoders && block_row[BLK_ENC]  >= 0) draw_encoders(block_row[BLK_ENC]);
    if (D->imu      && block_row[BLK_IMU]  >= 0) draw_imu     (block_row[BLK_IMU]);
    if (D->motors   && block_row[BLK_MOT]  >= 0) draw_motors  (block_row[BLK_MOT]);
    if (D->system   && block_row[BLK_SYS]  >= 0) draw_system  (block_row[BLK_SYS]);

    draw_log(g_log_row);

    refresh();
}

/* ── display thread ─────────────────────────────────────────────── */

static void *display_thread(void *arg)
{
    (void)arg;

    SCREEN *scr = newterm(getenv("TERM"), fopen("/dev/tty", "r+"), fopen("/dev/tty", "r+"));
    if (!scr) {
        fprintf(stderr, "[display] newterm() failed — TERM=%s\n",
                getenv("TERM") ? getenv("TERM") : "(null)");
        fflush(stderr);
        return NULL;
    }
    set_term(scr);
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    if (has_colors()) init_colours();

    calc_layout();

    while (g_running) {
        int key = getch();
        if (key == 'q' || key == 'Q') {
            rc_set_state(EXITING);
            break;
        }

        /* always redraw when log has new data or dirty flag set */
        if (g_dirty) {
            g_dirty = 0;
            redraw();
        } else {
            /* redraw log panel even without a dirty flag so messages appear promptly */
            draw_log(g_log_row);
            refresh();
        }

        usleep(50000);  /* 20 Hz */
    }

    endwin();
    delscreen(scr);
    return NULL;
}

/* ── public API ─────────────────────────────────────────────────── */

int display_init(void)
{
    display_config_t *D = &g_debug_config.display;

    if (!D->sbus_tx && !D->pid && !D->encoders &&
        !D->imu && !D->motors && !D->system) {
        g_enabled = 0;
        return 0;
    }

    /* redirect stdout/stderr before starting ncurses */
    redirect_output();

    g_enabled = 1;
    g_running = 1;
    g_dirty   = 0;

    memset(g_log, 0, sizeof(g_log));

    if (pthread_create(&g_thread, NULL, display_thread, NULL) != 0) {
        perror("display_init: pthread_create");
        restore_output();
        g_running = 0;
        g_enabled = 0;
        return -1;
    }

    return 0;
}

void display_update(void)
{
    if (g_enabled)
        g_dirty = 1;
}

void display_cleanup(void)
{
    if (!g_enabled) return;
    g_running = 0;
    pthread_join(g_thread, NULL);
    restore_output();
    g_enabled = 0;
}
