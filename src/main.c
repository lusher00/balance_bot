/**
 * @file main.c
 * @brief Main entry point for balance_bot
 *
 * Parses command-line arguments and initialises all subsystems.
 */

#include "balance_bot.h"
#include "motor_hal.h"
#include "display.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

/* ── input mode ─────────────────────────────────────────────────── */

typedef enum
{
    INPUT_NONE,
    INPUT_EXT, /* generic UART packet input (--input ext) */
    INPUT_XBOX,
    INPUT_SBUS
} input_mode_t;

static input_mode_t input_mode = INPUT_NONE;
static char *pid_config_file = NULL;
static char *xbox_device = NULL;
static char *sbus_device = NULL;

/* UART packet input */
static char *ext_uart_device = NULL;
static int ext_uart_baud = 115200;
static int ext_uart_timeout = 500; /* ms */

/* RoboClaw motor driver — always used */
static char *roboclaw_device = NULL; /* default: /dev/ttyO1 */
static int roboclaw_baud = 460800;

static bool quiet_mode = false;

/* Display block enables (-d flags) */
static int disp_sbus = 0;
static int disp_pid = 0;
static int disp_enc = 0;
static int disp_imu = 0;
static int disp_motors = 0;
static int disp_system = 100; /* on by default */

/* ── usage ──────────────────────────────────────────────────────── */

static void print_usage(void)
{
    printf("\n");
    printf("balance_bot — self-balancing robot\n");
    printf("\n");
    printf("Usage: balance_bot [options]\n");
    printf("\n");
    printf("Input options:\n");
    printf("  -i <mode>          Input mode:\n");
    printf("                       none  — balance only, no drive input (default)\n");
    printf("                       ext   — generic UART packet input\n");
    printf("                       xbox  — Xbox controller\n");
    printf("                       sbus  — FrSky R-XSR via UART (T16 transmitter)\n");
    printf("  -u <device>        UART device for SBUS or EXT input\n");
    printf("                     (default: /dev/ttyO5 for SBUS, /dev/ttyO1 for EXT)\n");
    printf("  -b <baud>          Baud rate for EXT UART input (default: 115200)\n");
    printf("  -t <ms>            Stale-packet timeout for EXT input (default: 500 ms)\n");
    printf("\n");
    printf("Motor options:\n");
    printf("  -m <device>        RoboClaw UART device (default: /dev/ttyO1)\n");
    printf("  -B <baud>          RoboClaw baud rate   (default: 38400)\n");
    printf("\n");
    printf("Other options:\n");
    printf("  -p <file>          PID config file (default: pidconfig.txt)\n");
    printf("  -d <block>         Enable console display block (repeatable):\n");
    printf("                       sbus  pid  enc  imu  mot  sys  all  none\n");
    printf("  -q                 Quiet mode (warnings only)\n");
    printf("  -h                 Show this help\n");
    printf("\n");
    printf("Examples:\n");
    printf("  balance_bot                            # Balance only\n");
    printf("  balance_bot -i sbus                    # T16 via R-XSR on /dev/ttyO5\n");
    printf("  balance_bot -i sbus -u /dev/ttyO5      # Same, explicit device\n");
    printf("  balance_bot -i ext  -u /dev/ttyO1      # Generic UART packet input\n");
    printf("  balance_bot -m /dev/ttyO1              # Use RoboClaw for motors\n");
    printf("  balance_bot -i sbus -m /dev/ttyO1      # SBUS + RoboClaw\n");
    printf("  balance_bot -i xbox /dev/input/js0     # Xbox controller\n");
    printf("  balance_bot -d imu -d pid              # Show IMU + PID display panels\n");
    printf("\n");
}

/* ── argument parsing ───────────────────────────────────────────── */

static int parse_args(int argc, char *argv[])
{
    int opt;

    while ((opt = getopt(argc, argv, "i:p:u:b:t:m:B:d:qh")) != -1)
    {
        switch (opt)
        {
        case 'i':
            if (strcmp(optarg, "ext") == 0)
                input_mode = INPUT_EXT;
            else if (strcmp(optarg, "xbox") == 0)
                input_mode = INPUT_XBOX;
            else if (strcmp(optarg, "sbus") == 0)
                input_mode = INPUT_SBUS;
            else if (strcmp(optarg, "none") == 0)
                input_mode = INPUT_NONE;
            else
            {
                fprintf(stderr, "Error: unknown input mode '%s'\n", optarg);
                print_usage();
                return -1;
            }
            break;

        case 'u':
            sbus_device = ext_uart_device = optarg;
            break;
        case 'b':
            ext_uart_baud = atoi(optarg);
            break;
        case 't':
            ext_uart_timeout = atoi(optarg);
            break;

        case 'm':
            roboclaw_device = optarg;
            break;
        case 'B':
            roboclaw_baud = atoi(optarg);
            break;

        case 'p':
            pid_config_file = optarg;
            break;
        case 'q':
            quiet_mode = true;
            break;

        case 'd':
            if (strcmp(optarg, "sbus") == 0)
                disp_sbus = 10;
            else if (strcmp(optarg, "pid") == 0)
                disp_pid = 10;
            else if (strcmp(optarg, "enc") == 0)
                disp_enc = 10;
            else if (strcmp(optarg, "imu") == 0)
                disp_imu = 10;
            else if (strcmp(optarg, "mot") == 0)
                disp_motors = 10;
            else if (strcmp(optarg, "sys") == 0)
                disp_system = 100;
            else if (strcmp(optarg, "all") == 0)
            {
                disp_sbus = disp_pid = disp_enc =
                    disp_imu = disp_motors = disp_system = 10;
            }
            else if (strcmp(optarg, "none") == 0)
            {
                disp_sbus = disp_pid = disp_enc =
                    disp_imu = disp_motors = disp_system = 0;
            }
            else
            {
                fprintf(stderr, "Error: unknown display block '%s'\n", optarg);
                print_usage();
                return -1;
            }
            break;

        case 'h':
            print_usage();
            exit(0);

        default:
            print_usage();
            return -1;
        }
    }

    /* Xbox controller: last positional arg is device path */
    if (input_mode == INPUT_XBOX)
    {
        xbox_device = (optind < argc) ? argv[optind] : "/dev/input/js0";
    }

    return 0;
}

/* ── main ───────────────────────────────────────────────────────── */

int main(int argc, char *argv[])
{
    pid_config_file_t pid_config;

    if (parse_args(argc, argv) < 0)
        return -1;

    /* Logging level */
    g_debug_config.logging.level = quiet_mode ? LOG_LEVEL_WARN : LOG_LEVEL_INFO;

    printf("\n");
    printf("╔═══════════════════════════════════════════════╗\n");
    printf("║         balance_bot v1.0                      ║\n");
    printf("║    Self-Balancing Robot + iPhone App          ║\n");
    printf("╚═══════════════════════════════════════════════╝\n");
    printf("\n");

    printf("Configuration:\n");
    switch (input_mode)
    {
    case INPUT_EXT:
        printf("  Input:      UART packet input (%s, %d baud, %d ms timeout)\n",
               ext_uart_device ? ext_uart_device : "/dev/ttyO1",
               ext_uart_baud, ext_uart_timeout);
        break;
    case INPUT_XBOX:
        printf("  Input:      Xbox controller (%s)\n", xbox_device);
        break;
    case INPUT_SBUS:
        printf("  Input:      SBUS / FrSky R-XSR (%s)\n",
               sbus_device ? sbus_device : "/dev/ttyO5");
        break;
    case INPUT_NONE:
        printf("  Input:      Balance only (no drive input)\n");
        break;
    }
    printf("  Motors:     RoboClaw on %s at %d baud\n",
           roboclaw_device ? roboclaw_device : "/dev/ttyO1", roboclaw_baud);

    printf("  PID config: %s\n", pid_config_file ? pid_config_file : "pidconfig.txt");
    printf("  Log level:  %s\n", quiet_mode ? "WARN" : "INFO");
    printf("\n");

    /* Debug / display config */
    g_debug_config = get_default_debug_config();
    g_debug_config.display.sbus_tx = disp_sbus;
    g_debug_config.display.pid = disp_pid;
    g_debug_config.display.encoders = disp_enc;
    g_debug_config.display.imu = disp_imu;
    g_debug_config.display.motors = disp_motors;
    g_debug_config.display.system = disp_system;

    /* PID config */
    if (pid_config_load_or_default(pid_config_file, &pid_config) < 0)
    {
        fprintf(stderr, "Error: Failed to load PID config\n");
        return -1;
    }
    if (!quiet_mode)
        pid_config_print(&pid_config);

    /* Telemetry */
    LOG_INFO("Initialising telemetry...");
    if (telemetry_init() < 0)
    {
        fprintf(stderr, "Error: telemetry init failed\n");
        return -1;
    }

    /* IPC server (Node.js / iPhone bridge) */
    LOG_INFO("Starting IPC server...");
    if (ipc_server_init() < 0)
    {
        fprintf(stderr, "Error: IPC server init failed\n");
        return -1;
    }

    LOG_INFO("Initialising motor HAL (RoboClaw)...");
    if (motor_hal_init(roboclaw_device, roboclaw_baud) < 0)
    {
        fprintf(stderr, "Error: motor HAL init failed\n");
        ipc_server_cleanup();
        return -1;
    }

    /* Robot hardware (IMU, encoders, buttons) */
    LOG_INFO("Initialising robot hardware...");
    if (robot_init() < 0)
    {
        fprintf(stderr, "Error: robot init failed\n");
        motor_hal_cleanup();
        ipc_server_cleanup();
        return -1;
    }

    pid_config_apply(&pid_config);

    motor_config_t motor_config;
    motor_config_load_or_default(pid_config_file, &motor_config);
    motor_config_apply(&motor_config);

    /* Input subsystem */
    switch (input_mode)
    {
    case INPUT_EXT:
        LOG_INFO("Initialising UART packet input...");
        if (uart_input_init(ext_uart_device ? ext_uart_device : "/dev/ttyO1",
                            ext_uart_baud, ext_uart_timeout) < 0)
        {
            fprintf(stderr, "Error: UART input init failed\n");
            robot_cleanup();
            ipc_server_cleanup();
            return -1;
        }
        break;

    case INPUT_XBOX:
        LOG_INFO("Initialising Xbox controller...");
        if (xbox_init(xbox_device) < 0)
        {
            fprintf(stderr, "Error: Xbox controller init failed\n");
            robot_cleanup();
            ipc_server_cleanup();
            return -1;
        }
        break;

    case INPUT_SBUS:
        LOG_INFO("Initialising SBUS (FrSky R-XSR)...");
        if (sbus_init(sbus_device) < 0)
        {
            fprintf(stderr, "Error: SBUS init failed on %s\n",
                    sbus_device ? sbus_device : "/dev/ttyO5");
            robot_cleanup();
            ipc_server_cleanup();
            return -1;
        }
        break;

    case INPUT_NONE:
        LOG_INFO("No drive input — balance only");
        break;
    }

    printf("\n");
    printf("═══════════════════════════════════════════════\n");
    printf("  System ready. Press MODE button to arm.\n");
    printf("═══════════════════════════════════════════════\n");
    printf("\n");

    if (!quiet_mode)
    {
        char desc[256];
        telemetry_get_config_description(desc, sizeof(desc));
        printf("%s\n\n", desc);
    }

    /* ncurses display thread */
    if (display_init() < 0)
        LOG_WARN("Display thread failed — continuing without live display");

    /* Blocking main loop */
    robot_run();

    /* ── cleanup (reverse init order) ── */
    LOG_INFO("Shutting down...");
    display_cleanup();

    switch (input_mode)
    {
    case INPUT_EXT:
        uart_input_cleanup();
        break;
    case INPUT_XBOX:
        xbox_cleanup();
        break;
    case INPUT_SBUS:
        sbus_cleanup();
        break;
    default:
        break;
    }

    robot_cleanup();

    ipc_server_cleanup();

    printf("\nbalance_bot stopped.\n\n");
    return 0;
}
